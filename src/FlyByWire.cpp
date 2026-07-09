#include "FlyByWire.h"

// ─────────────────────────────────────────────────────────────────────────────

void FlyByWire::reset()
{
    _elevInteg    = 0.f;
    _prevPitchErr = 0.f;
    _cstarDem     = 1.f;
    _cstarAct     = 1.f;
    _alphaFloor   = false;
    _envActive    = 0;
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
        _envActive    = 0;
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
        // (não pushback aberto — o pushback proporcional saturava o profundor
        // sem damping, o avião cruzava o limite com rotação enorme sobrando e
        // dava looping entre as duas proteções).
        //
        // COM ESTADO + HISTERESE (_envActive): liga/desliga exatamente na
        // fronteira fazia o profundor "brigar" (chattering) pairando em
        // 30°/−15° — qualquer cruzamento minúsculo alternava entre um chute
        // de cstarK×(qDem−q) e zero. Agora: engaja ao cruzar o limite, mas o
        // ALVO de recuperação fica ENV_MARGIN dentro do envelope, e só solta
        // quando chegou no alvo COM a rotação freada (|q| < 1°/s) — o ponto
        // de desligamento fica longe da superfície de chaveamento e o
        // desengate acontece com envErr ≈ 0 (sem degrau no profundor).
        // Não alimenta o integrador (transitório ≠ trim).
        float envErr = 0.f;
        {
            constexpr float PITCH_CEIL = 30.f, PITCH_FLOOR = -15.f;
            constexpr float ENV_DZ       = 0.05f;
            constexpr float ENV_MARGIN   = 2.f;    // alvo: 2° dentro do envelope
            constexpr float ENV_RATE_KP  = 0.6f;   // (°/s) por ° até o alvo
            constexpr float ENV_MAX_RATE = 6.f;    // taxa máx de recuperação °/s

            bool inhibit = st.altAgl <= 200.f || std::abs(columnMod) >= ENV_DZ;
            if (inhibit) {
                _envActive = 0;                     // piloto tem autoridade total
            } else if (_envActive == 0) {
                if      (st.pitchDeg > PITCH_CEIL)  _envActive = +1;
                else if (st.pitchDeg < PITCH_FLOOR) _envActive = -1;
            }

            if (_envActive != 0) {
                float target = (_envActive > 0) ? PITCH_CEIL  - ENV_MARGIN
                                                : PITCH_FLOOR + ENV_MARGIN;
                float qDem = std::clamp(ENV_RATE_KP * (target - st.pitchDeg),
                                        -ENV_MAX_RATE, ENV_MAX_RATE);
                envErr = gains.cstarK * (qDem - st.pitchRateDegS) * DEG2RAD;

                bool reached = (_envActive > 0) ? (st.pitchDeg <= target)
                                                : (st.pitchDeg >= target);
                if (reached && std::abs(st.pitchRateDegS) < 1.f) {
                    _envActive = 0;
                    envErr     = 0.f;
                }
            }
        }

        // Estabilidade de velocidade (soft, atua mesmo com stick ativo):
        //   overspeed  (>spdVHi) → comanda nariz para cima
        //   underspeed (<spdVLo) → comanda nariz para baixo, SOMENTE se:
        //     gear UP  E  (flaps ≤ curso 1  OU  speed brake aberto)
        //   Em configuração de pouso (gear down / flaps estendidos) não empurra
        //   o nariz — voo lento é intencional nessa configuração.
        // Em var SEPARADA que NÃO alimenta o integrador (mesma lição do pitch
        // envelope): integrada, virava trim acumulado que continuava puxando
        // o nariz muito depois da velocidade voltar pro envelope.
        float spdStab = 0.f;
        bool sbkOpen  = !st.wow && inp.brake > 0.05f && inp.flaps < (1.f/3.f);
        bool cleanCfg = inp.flaps <= (1.f/6.f + 0.01f) || sbkOpen;
        if (st.casKt > gains.spdVHi)
            spdStab =  std::min(0.25f, gains.spdStabK * (st.casKt - gains.spdVHi));
        else if (st.casKt < gains.spdVLo && !inp.gearCmd && cleanCfg)
            spdStab = -std::min(0.25f, gains.spdStabK * (gains.spdVLo - st.casKt));

        columnMod = std::clamp(columnMod, -1.f, 1.f);

        // Em terra: modo solo — stick → profundor DIRETO (permite a rotação na
        // decolagem; C* não faz sentido com rodas no chão). Ao decolar
        // (wow=false) a lei C* reinicializa capturando o estado atual.
        if (st.wow) {
            out.elevLH = out.elevRH = clamp1(-inp.column);
            _elevInteg    = 0.f;
            _prevPitchErr = 0.f;
            _initialized  = false;  // reinicializa quando decolar
            _envActive    = 0;
        } else {
            // Alpha floor — proteção de estol. Diferente do spdStab (soft,
            // desligado em config de pouso porque voo lento ali é
            // intencional), esta é uma proteção DURA: fica sempre ativa,
            // mesmo com trem/flap de pouso e mesmo com stick puxado —
            // sem ela, a aeronave podia perder velocidade em final de
            // aproximação sem NENHUMA proteção de arfagem (reportado:
            // avião "se perdeu" — rolou sozinho — em baixa velocidade perto
            // da pista, alpha alto sem nenhum sistema segurando o nariz).
            // COM HISTERESE (mesma lição do envelope de pitch acima —
            // liga/desliga exatamente na fronteira fazia o profundor
            // "brigar"/chattering): engaja ao cruzar ALPHA_PROT, só
            // desengaja quando o alpha já caiu ALPHA_HYST abaixo do limiar
            // E a rotação amorteceu — sem isso, a própria correção derrubava
            // o alpha, desligava, o nariz subia de novo e reengajava sem
            // parar (reportado: profundor "brigando", -1 quase sempre).
            // Demanda taxa de arfagem negativa crescendo com o quanto o
            // alpha passou do limiar; não alimenta o integrador (é
            // transitório, como o envelope de pitch acima).
            float alphaErr = 0.f;
            {
                constexpr float ALPHA_HYST     = 0.035f;  // ~2° de folga pra desengajar
                constexpr float ALPHA_RATE_KP  = 2.0f;    // (°/s) por ° acima do limiar
                constexpr float ALPHA_MAX_RATE = 5.f;     // taxa máx de recuperação °/s

                if (!_alphaFloor && st.alphaRad > ALPHA_PROT)
                    _alphaFloor = true;
                else if (_alphaFloor && st.alphaRad < ALPHA_PROT - ALPHA_HYST
                                      && std::abs(st.pitchRateDegS) < 1.f)
                    _alphaFloor = false;

                if (_alphaFloor) {
                    float excessDeg = (st.alphaRad - (ALPHA_PROT - ALPHA_HYST)) / DEG2RAD;
                    float qDem = -std::clamp(ALPHA_RATE_KP * std::max(excessDeg, 0.f),
                                             0.f, ALPHA_MAX_RATE);
                    alphaErr = gains.cstarK * (qDem - st.pitchRateDegS) * DEG2RAD;
                }
            }

            _cstarDem = _cstarAct + (columnMod + spdStab) * gains.maxDemand + envErr + alphaErr;

            // err (P/D) inclui envelope + speed-stab — proteções transitórias
            // que zeram sozinhas quando o estado volta pro envelope.
            // errInteg (I) é SÓ o piloto: o integrador é o trim de longo prazo.
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
        if (st.wow || law == Law::Direct || !_ydOn) {
            // No solo, Direct Law ou YD desligado: pedais diretos, sem aumentação
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

        // Ground spoilers: auto-deploy no toque com motores em idle OU com
        // reversor deployado — reverso não é empuxo pra frente; a manete a
        // 50% do auto-reverso derrubava a condição de idle e RECOLHIA os
        // spoilers exatamente na hora que mais precisam estar abertos.
        // Com eles, os MFS laterais (speedbrake) também abrem full — painéis
        // todos em cima matando sustentação no rollout, como no real.
        if (st.wow) {
            bool armed = (inp.throttle[0] < 0.05f && inp.throttle[1] < 0.05f)
                       || out.reverser;
            out.groundSpoiler = armed ? 1.f : 0.f;
            if (armed) out.spoilerL = out.spoilerR = 1.f;
        } else {
            out.groundSpoiler = 0.f;
        }
    }
}
