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
    _initialized  = false;
}

// ─────────────────────────────────────────────────────────────────────────────

void FlyByWire::update(float dt, const PilotInput& inp,
                       const AircraftState& st, SurfaceCmd& out)
{
    if (dt < 1e-6f) return;

    auto clamp01 = [](float v) { return std::clamp(v, 0.f, 1.f); };
    auto clamp1  = [](float v) { return std::clamp(v, -1.f, 1.f); };

    // ── Throttle (passa direto para FADEC) ───────────────────────────────────
    out.throttle[0] = clamp01(inp.throttle[0]);
    out.throttle[1] = clamp01(inp.throttle[1]);

    // ── Flaps (7 deflexões SLAT/FLAP: 0..0.75 em controls/flight/flaps) ─────
    out.flaps = clamp01(inp.flaps) * 0.75f;

    // ── Freios / Speed brake — depende de WOW ────────────────────────────────
    if (st.wow) {
        out.brakeL = out.brakeR = clamp01(inp.brake);   // em terra: freios de roda
    } else {
        out.brakeL  = out.brakeR = 0.f;                  // em voo: sem freio de roda
        // Speed brake controlado pelo B (ground spoilers em voo = aerobrake)
        out.groundSpoiler = clamp01(inp.brake);
    }

    // ── Trem de pouso ─────────────────────────────────────────────────────────
    _gearDown    = inp.gearCmd;
    out.gearDown = _gearDown;

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // ARFAGEM — Lei C*  (sem proteções de envelope — melhorar depois)
    //
    //  C* = Nz + (Vo/g) × q      (Vo ≈ 225 fps → Vo/g ≈ 7 s)
    //  Sinal elevador:  master negativo = cabrar (Cmde = -0.9)
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    {
        float q_rps = st.pitchRateDegS * DEG2RAD;
        _cstarAct = st.loadFactorNz + gains.cstarK * q_rps;

        if (!_initialized) {
            _cstarDem     = _cstarAct;
            _prevPitchErr = 0.f;
            _initialized  = true;
        }

        _cstarDem = _cstarAct + inp.column * gains.maxDemand;
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
    // ROLAGEM — Rate demand / attitude hold  (sem limites de bank por ora)
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    {
        float ailCmd = 0.f;
        _bankProt = false;

        if (std::fabsf(inp.wheel) > ROLL_DEADBAND) {
            float demRate = inp.wheel * gains.maxRollRateDegS;
            float err     = demRate - st.rollRateDegS;
            float derr    = (err - _prevRollErr) / dt;
            _prevRollErr  = err;
            ailCmd        = gains.rollKp * err + gains.rollKd * derr;
            _targetBank   = st.rollDeg;
        } else {
            _prevRollErr = 0.f;
            float err = _targetBank - st.rollDeg;
            ailCmd    = gains.holdKp * err;
        }

        ailCmd = clamp1(ailCmd);
        out.aileronL =  ailCmd;
        out.aileronR = -ailCmd;
        out.spoilerL = 0.f;
        out.spoilerR = 0.f;
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // GUINADA — Pedais diretos (yaw damper desativado: setas só controlam aileron)
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    {
        if (st.wow) {
            // Em terra: pedais dirigem o nariz, rudder com autoridade total
            out.rudder       = clamp1(inp.pedals);
            out.steerNoseDeg = inp.pedals * MAX_STEER_DEG;
        } else {
            // Em voo: rudder com 50% de autoridade de pedal (FBW limita)
            out.rudder       = clamp1(inp.pedals * 0.5f);
            out.steerNoseDeg = 0.f;
        }
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // SPOILERS DE SOLO — auto-deploy no toque (WOW + throttle idle)
    // Só sobrescreve se ainda não setado pelo speed brake acima
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    if (st.wow) {
        bool gndSpoilArmed = (inp.throttle[0] < 0.05f && inp.throttle[1] < 0.05f);
        if (gndSpoilArmed) out.groundSpoiler = 1.f;
    }
}
