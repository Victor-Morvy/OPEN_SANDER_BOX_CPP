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
    out.reverser    = inp.reverser;

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
    {
        float q_rps = st.pitchRateDegS * DEG2RAD;
        _cstarAct = st.loadFactorNz + gains.cstarK * q_rps;

        if (!_initialized) {
            _cstarDem     = _cstarAct;
            _prevPitchErr = 0.f;
            _initialized  = true;
        }

        // Pitch envelope: pushback no column quando além dos limites e stick neutro
        float columnMod = inp.column;
        if (st.altAgl > 200.f) {
            constexpr float PITCH_CEIL  = 30.f, PITCH_FLOOR = -15.f;
            constexpr float ENV_KP = 0.02f,  ENV_DZ = 0.05f;
            if (std::abs(columnMod) < ENV_DZ) {
                if (st.pitchDeg > PITCH_CEIL)
                    columnMod -= std::clamp(ENV_KP * (st.pitchDeg - PITCH_CEIL),  0.f, 0.5f);
                else if (st.pitchDeg < PITCH_FLOOR)
                    columnMod += std::clamp(ENV_KP * (PITCH_FLOOR - st.pitchDeg), 0.f, 0.5f);
            }
        }

        _cstarDem = _cstarAct + columnMod * gains.maxDemand;
        _alphaFloor = false;  // proteção desabilitada por ora

        float err  = _cstarDem - _cstarAct;
        _elevInteg += err * dt;
        _elevInteg  = std::clamp(_elevInteg, -2.f, 2.f);  // clamp largo

        float derr = (inp.column != 0.f) ? (err - _prevPitchErr) / dt : 0.f;
        _prevPitchErr = err;

        // master negativo = cabrar (Cmde=-0.9: pos-rad neg → Cm positivo)
        float elev = -(gains.pitchKp * err
                     + gains.pitchKi * _elevInteg
                     + gains.pitchKd * derr);

        out.elevLH = out.elevRH = clamp1(elev);
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
    {
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
            if (std::abs(target) > BANK_HOLD_LIM) {
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

        // Limite rígido em 67°: bloqueia aileron que aprofundaria além do limite
        if (std::abs(st.rollDeg) >= BANK_NORM_LIM) {
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
        float yawDamp  = -gains.yawDamperK * st.yawRateDegS;
        float betaCorr =  gains.betaKp     * st.betaDeg;
        out.rudder       = clamp1(inp.pedals + yawDamp + betaCorr);
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
