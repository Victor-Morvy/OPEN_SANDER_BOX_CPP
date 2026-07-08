#include "FlyByWire.h"

// ─────────────────────────────────────────────────────────────────────────────

void FlyByWire::reset()
{
    _elevInteg    = 0.f;
    _prevPitchErr = 0.f;
    _cstarDem     = 1.f;
    _cstarAct     = 1.f;
    _alphaFloor   = false;
    _targetBank   = 0.f;
    _prevRollErr  = 0.f;
    _bankProt     = false;
    _prevYawRate  = 0.f;
    _betaFilt     = 0.f;
    _betaInteg    = 0.f;
    _initialized  = false;
}

// ─────────────────────────────────────────────────────────────────────────────

void FlyByWire::update(float dt, const PilotInput& inp,
                       const AircraftState& st, SurfaceCmd& out)
{
    if (dt < 1e-6f) return;

    auto clamp01 = [](float v) { return std::clamp(v, 0.f, 1.f); };
    auto clamp1  = [](float v) { return std::clamp(v, -1.f, 1.f); };

    // ── Throttle + reversor (passam direto para FADEC/FDM) ──────────────────
    out.throttle[0] = clamp01(inp.throttle[0]);
    out.throttle[1] = clamp01(inp.throttle[1]);
    // Reversor: trava de solo — só deploya com peso nas rodas (como no real)
    out.reverser    = inp.reverser && st.wow;

    // ── Flaps (7 deflexões SLAT/FLAP: 0..0.75 em controls/flight/flaps) ─────
    out.flaps = clamp01(inp.flaps) * 0.75f;

    // ── Freios — somente em terra ─────────────────────────────────────────────
    if (st.wow) {
        out.brakeL = out.brakeR = clamp01(inp.brake);
    } else {
        out.brakeL = out.brakeR = 0.f;
    }

    // ── Trem de pouso ─────────────────────────────────────────────────────────
    _gearDown    = inp.gearCmd;
    out.gearDown = _gearDown;

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // ARFAGEM — Lei C* + Pitch Envelope Protection
    //
    //  C* = Nz + (Vo/g) × q      (Vo ≈ 225 fps → Vo/g ≈ 7 s)
    //  Sinal elevador:  master negativo = cabrar (Cmde = -0.9)
    //
    //  Pitch Envelope (idêntico ao computePitchEnvelope do E195-E2Sim):
    //    · teto: +30°  piso: -15°
    //    · ativo apenas acima de 200 ft AGL
    //    · pushback proporcional (Kp=0.02) quando stick está neutro (|col|<0.05)
    //    · piloto com stick ativo sempre tem autoridade total (proteção é "soft")
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    if (law == Law::Direct) {
        // ── DIRECT LAW: stick → profundor, ganho fixo, sem aumentação ─────────
        out.elevLH = out.elevRH = clamp1(-inp.column);
        _elevInteg    = 0.f;
        _prevPitchErr = 0.f;
        _initialized  = false;
        _alphaFloor   = false;
    } else {
        float q_rps = st.pitchRateDegS * DEG2RAD;
        _cstarAct = st.loadFactorNz + gains.cstarK * q_rps;

        if (!_initialized) {
            _cstarDem     = _cstarAct;
            _prevPitchErr = 0.f;
            _initialized  = true;
        }

        float columnMod = inp.column;

        // Pitch envelope: DEMANDA DE TAXA de recuperação fechada na taxa real
        // (não pushback aberto proporcional à excursão). O pushback antigo
        // saturava o profundor sem nenhum amortecimento: o avião ganhava taxa
        // de arfagem enorme, cruzava o limite com toda a rotação sobrando,
        // ativava a proteção do lado oposto — e dava looping (bug do Victor).
        // Agora: qDem = clamp(0.6/s × excursão, ±6 °/s) → errEnv em unidades
        // de C* = cstarK × (qDem − q). Como qDem → 0 na fronteira, a lei
        // FREIA a rotação ANTES de satisfazer a proteção — o profundor
        // corrige/solta exatamente quando o pitch volta pro envelope, sem
        // taxa residual pra causar overshoot. Não alimenta o integrador
        // (transitório ≠ trim — ver fix anterior).
        float envErr = 0.f;
        if (st.altAgl > 200.f) {
            constexpr float PITCH_CEIL = 30.f, PITCH_FLOOR = -15.f;
            constexpr float ENV_DZ = 0.05f;
            constexpr float ENV_RATE_KP  = 0.6f;   // (°/s) por ° de excursão
            constexpr float ENV_MAX_RATE = 6.f;    // taxa máx de recuperação °/s
            if (std::abs(columnMod) < ENV_DZ) {
                float over = 0.f;                   // + = precisa cabrar
                if      (st.pitchDeg > PITCH_CEIL)  over = PITCH_CEIL  - st.pitchDeg;
                else if (st.pitchDeg < PITCH_FLOOR) over = PITCH_FLOOR - st.pitchDeg;
                if (over != 0.f) {
                    float qDem = std::clamp(ENV_RATE_KP * over,
                                            -ENV_MAX_RATE, ENV_MAX_RATE);
                    envErr = gains.cstarK * (qDem - st.pitchRateDegS) * DEG2RAD;
                }
            }
        }

        // Estabilidade de velocidade (soft, atua mesmo com stick ativo):
        //   overspeed  (>330 kt) → comanda nariz para cima
        //   underspeed (<150 kt) → comanda nariz para baixo, SOMENTE se:
        //     gear UP  E  (flaps ≤ curso 1  OU  speed brake aberto)
        //   Em configuração de pouso (gear down / flaps estendidos) não empurra
        //   o nariz — voo lento é intencional nessa configuração.
        bool sbkOpen  = !st.wow && inp.brake > 0.05f && inp.flaps < (1.f/3.f);
        bool cleanCfg = inp.flaps <= (1.f/6.f + 0.01f) || sbkOpen;
        if (st.casKt > gains.spdVHi)
            columnMod += std::min(0.5f, gains.spdStabK * (st.casKt - gains.spdVHi));
        else if (st.casKt < gains.spdVLo && !inp.gearCmd && cleanCfg)
            columnMod -= std::min(0.5f, gains.spdStabK * (gains.spdVLo - st.casKt));

        columnMod = std::clamp(columnMod, -1.f, 1.f);

        // Em terra: modo solo — stick → profundor DIRETO (permite a rotação na
        // decolagem; C* não faz sentido com rodas no chão). Ao decolar
        // (wow=false) a lei C* reinicializa capturando o estado atual.
        if (st.wow) {
            out.elevLH = out.elevRH = clamp1(-inp.column);
            _elevInteg    = 0.f;
            _prevPitchErr = 0.f;
            _initialized  = false;  // reinicializa quando decolar
        } else {
            _cstarDem = _cstarAct + columnMod * gains.maxDemand + envErr;
            _alphaFloor = false;

            // err (P/D) inclui a demanda do envelope — regulada na taxa real,
            // zera quando o pitch volta pro envelope com a rotação já freada.
            // errInteg (I) NÃO inclui: o integrador é o trim de longo prazo e
            // só deve responder ao piloto/speed-stab.
            float err      = _cstarDem - _cstarAct;
            float errInteg = columnMod * gains.maxDemand;
            _elevInteg += errInteg * dt;
            _elevInteg  = std::clamp(_elevInteg, -2.f, 2.f);

            float derr = (inp.column != 0.f) ? (err - _prevPitchErr) / dt : 0.f;
            _prevPitchErr = err;

            float elev = -(gains.pitchKp * err
                         + gains.pitchKi * _elevInteg
                         + gains.pitchKd * derr);
            out.elevLH = out.elevRH = clamp1(elev);
        }
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // ROLAGEM — Rate demand / attitude hold com bank angle protection
    //
    //  Stick ativo  → demanda taxa de rolagem proporcional (max 22°/s)
    //                 PD controla a taxa real para atingir a demanda
    //  Stick neutro → attitude hold no banco capturado no momento da soltura
    //                 Proteção: se banco > 33°, retorna a 33°
    //  Limite rígido: 67° (BANK_NORM_LIM) — trava o aileron que aprofundaria
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    if (law == Law::Direct) {
        // ── DIRECT LAW: stick → aileron, ganho fixo ───────────────────────────
        out.aileronL =  clamp1(inp.wheel);
        out.aileronR = -clamp1(inp.wheel);
        _targetBank  = st.rollDeg;
        _bankProt    = false;
    } else {
        float ailCmd;

        if (std::abs(inp.wheel) > ROLL_DEADBAND) {
            // Rate demand: P puro na taxa — sem derivativo (evita amplificação de ruído)
            float demRate = inp.wheel * gains.maxRollRateDegS;
            float rateErr = demRate - st.rollRateDegS;
            ailCmd        = gains.rollKp * rateErr;
            _targetBank   = st.rollDeg;
            _bankProt     = false;
        } else {
            // Attitude hold: cascata banco → taxa → aileron (sem derivativo)
            float target = _targetBank;
            if (law == Law::Normal && std::abs(target) > BANK_HOLD_LIM) {
                // Proteção: acima de 33° retorna a 33° (apenas Normal Law)
                target    = std::copysign(BANK_HOLD_LIM, target);
                _bankProt = true;
            } else {
                _bankProt = false;
            }
            float bankErr = target - st.rollDeg;
            float demRate = std::clamp(gains.holdKp * bankErr * gains.maxRollRateDegS,
                                       -gains.maxRollRateDegS, gains.maxRollRateDegS);
            float rateErr = demRate - st.rollRateDegS;
            ailCmd        = gains.rollKp * rateErr;
        }
        _prevRollErr = 0.f;  // não usado mais, mantido para compatibilidade do reset()

        // Limite rígido em 67° (apenas Normal Law)
        if (law == Law::Normal && std::abs(st.rollDeg) >= BANK_NORM_LIM) {
            float sign = (st.rollDeg > 0.f) ? 1.f : -1.f;
            if (ailCmd * sign > 0.f) ailCmd = 0.f;
        }

        out.aileronL =  clamp1(ailCmd);
        out.aileronR = -clamp1(ailCmd);
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // GUINADA — Pedais diretos + yaw damper + auto-rudder (beta)
    //
    //  yawDamp  = -K × r     → amortecer taxa de guinada
    //  betaCorr = Kb × β     → auto-rudder para reduzir derrapagem lateral
    //  Steering do nariz usa apenas o sinal direto dos pedais (sem damper)
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    {
        if (st.wow || law == Law::Direct) {
            // No solo ou Direct Law: pedais diretos, sem aumentação
            out.rudder  = clamp1(inp.pedals);
            _betaFilt   = st.betaDeg;   // reseta filtro
            _betaInteg  = 0.f;
        } else {
            // Filtro passa-baixo: atenua ruído do sinal de beta sem prejudicar resposta OEI
            _betaFilt  = gains.betaFiltA * _betaFilt + (1.f - gains.betaFiltA) * st.betaDeg;
            // Integrador: acumula beta residual → força final que elimina derrapagem em regime
            _betaInteg += _betaFilt * dt;
            _betaInteg  = std::clamp(_betaInteg, -8.f, 8.f);   // anti-windup ±8 °·s
            float yawDamp  = -gains.yawDamperK * st.yawRateDegS;
            float betaCorr =  gains.betaKp * _betaFilt + gains.betaKi * _betaInteg;
            out.rudder     = clamp1(inp.pedals + yawDamp + betaCorr);
        }
        out.steerNoseDeg = inp.pedals * MAX_STEER_DEG;
        _prevYawRate     = st.yawRateDegS;
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // MFS — Multifunctional Spoilers (spoilerons + speed brake)
    //
    //  Roll-to-spoiler (MPS): idêntico ao FlightControlModule do E195-E2-main
    //    · deadzone ±0.2 no wheel (inp.wheel)
    //    · ganho por velocidade: 100% ≤ 180 kt, zero ≥ 280 kt
    //    · roll direita → spoilers direitos (mfs8-10) sobem
    //    · roll esquerda → spoilers esquerdos (mfs1-3) sobem
    //
    //  Speed brake: inp.brake quando no ar
    //    · inibido se flaps > notch 1 (inp.flaps ≥ 1/3 da escala)
    //    · somado ao comando de roll nos painéis MFS laterais
    //
    //  Ground spoilers (mfs4-7): auto-deploy no toque com motores em idle
    //    · zero em voo (NÃO é usado como speed brake — isso vai nos MFS laterais)
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    {
        // Deadzone ±0.2 (tabela _mpsDeadzoneTable do projeto principal)
        auto mpsDeadzone = [](float x) -> float {
            constexpr float DZ = 0.2f;
            if (x >  DZ) return (x - DZ) / (1.f - DZ);
            if (x < -DZ) return (x + DZ) / (1.f - DZ);
            return 0.f;
        };

        // Ganho por velocidade (tabela _mpsAirspeedGainTable do projeto principal)
        auto mpsGain = [](float cas) -> float {
            if (cas <= 180.f) return 1.f;
            if (cas >= 280.f) return 0.f;
            return 1.f - (cas - 180.f) / 100.f;
        };

        float mfsL = 0.f, mfsR = 0.f;

        if (!st.wow) {
            float dzOut      = mpsDeadzone(inp.wheel);   // +wheel = roll direita
            float gainOut    = mpsGain(st.casKt);
            float bankMaster = dzOut * gainOut;

            if (bankMaster > 0.f)
                mfsR = bankMaster;    // roll direita → spoilers direitos sobem
            else
                mfsL = -bankMaster;   // roll esquerda → spoilers esquerdos sobem
        }

        // Speed brake: inibido se flaps > notch 1  (notch1 = 1/6 ≈ 0.167)
        float sbk = 0.f;
        if (!st.wow && inp.flaps < (1.f / 3.f))
            sbk = clamp01(inp.brake);

        out.spoilerL = clamp01(mfsL + sbk);
        out.spoilerR = clamp01(mfsR + sbk);

        // Ground spoilers: auto-deploy no toque com motores em idle; zero em voo
        if (st.wow) {
            bool armed = (inp.throttle[0] < 0.05f && inp.throttle[1] < 0.05f);
            out.groundSpoiler = armed ? 1.f : 0.f;
        } else {
            out.groundSpoiler = 0.f;
        }
    }
}
