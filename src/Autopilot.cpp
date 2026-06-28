#include "Autopilot.h"
#include <algorithm>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────

void Autopilot::engage(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    targetPitchDeg = st.pitchDeg;
    targetBankDeg  = st.rollDeg;
    _pitchInteg    = 0.f;
    fbw.setTargetBank(targetBankDeg);
    mode = Mode::AttitudeHold;
}

void Autopilot::engageAlt(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    targetAltFt    = st.altBaro;
    targetBankDeg  = st.rollDeg;
    targetPitchDeg = st.pitchDeg;   // ponto de partida; sobrescrito a cada frame
    _pitchInteg    = 0.f;
    fbw.setTargetBank(targetBankDeg);
    mode = Mode::AltitudeHold;
}

void Autopilot::disengage()
{
    mode        = Mode::Off;
    _pitchInteg = 0.f;
}

// ─────────────────────────────────────────────────────────────────────────────

bool Autopilot::update(float dt, const FlyByWire::AircraftState& st,
                       FlyByWire::PilotInput& inp, FlyByWire& fbw)
{
    if (mode == Mode::Off) return false;

    // Auto-desconexão por intervenção do piloto
    if (std::abs(inp.column) > DISC_THRESH || std::abs(inp.wheel) > DISC_THRESH) {
        disengage();
        return false;
    }

    // ── Altitude Hold: cascata alt → VS → pitch ───────────────────────────────
    if (mode == Mode::AltitudeHold) {
        constexpr float KP_ALT = 2.0f;    // ft/min por ft de erro
        constexpr float KP_VS  = 0.008f;  // graus de pitch por ft/min de erro de VS

        float altErr    = targetAltFt - st.altBaro;
        float vsDemand  = std::clamp(KP_ALT * altErr, -MAX_VS_FPM, MAX_VS_FPM);
        float vsErr     = vsDemand - st.vsFpm;
        targetPitchDeg  = std::clamp(KP_VS * vsErr, -MAX_PITCH_AP, MAX_PITCH_AP);
    }

    // ── Loop externo de pitch: theta_error → column → lei C* ─────────────────
    // (compartilhado entre AttitudeHold e AltitudeHold)
    {
        constexpr float KP = 0.050f;
        constexpr float KI = 0.006f;
        float err      = targetPitchDeg - st.pitchDeg;
        _pitchInteg   += err * dt;
        _pitchInteg    = std::clamp(_pitchInteg, -20.f, 20.f);
        inp.column     = std::clamp(KP * err + KI * _pitchInteg, -0.8f, 0.8f);
    }

    // ── Roll: wheel neutro → FBW mantém _targetBank ───────────────────────────
    inp.wheel = 0.f;

    return true;
}
