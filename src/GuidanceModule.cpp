#include "GuidanceModule.h"
#include <algorithm>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────

void GuidanceModule::engageAttitude(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    targets.pitchDeg = st.pitchDeg;
    targets.bankDeg  = st.rollDeg;
    _pitchInteg      = 0.f;
    fbw.setTargetBank(targets.bankDeg);
    mode.vert = VertMode::AttitudeHold;
}

void GuidanceModule::engageAltitude(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    targets.altFt    = st.altBaro;
    targets.bankDeg  = st.rollDeg;
    targets.pitchDeg = st.pitchDeg;
    _pitchInteg      = 0.f;
    fbw.setTargetBank(targets.bankDeg);
    mode.vert = VertMode::AltitudeHold;
}

void GuidanceModule::engageHeading(const FlyByWire::AircraftState& st)
{
    targets.headingDeg = st.hdgDeg;
    mode.lat = LatMode::HeadingHold;
}

void GuidanceModule::engageSpeed(const FlyByWire::AircraftState& st, float currentThrottle)
{
    targets.speedKt  = st.casKt;
    _baseThrottle    = currentThrottle;
    _throttleInteg   = 0.f;
    mode.thr = ThrMode::SpeedHold;
}

void GuidanceModule::disengageVert()
{
    mode.vert   = VertMode::Off;
    _pitchInteg = 0.f;
}

void GuidanceModule::disengageLat()
{
    mode.lat = LatMode::Off;
}

void GuidanceModule::disengageThrottle()
{
    mode.thr        = ThrMode::Off;
    _throttleInteg  = 0.f;
}

void GuidanceModule::disengageAll()
{
    disengageVert();
    disengageLat();
    disengageThrottle();
}

// ─────────────────────────────────────────────────────────────────────────────

bool GuidanceModule::update(float dt, const FlyByWire::AircraftState& st,
                            FlyByWire::PilotInput& inp, FlyByWire& fbw, Output& out)
{
    out = {};

    // ── Auto-desconexão por intervenção do piloto (vert + lat) ───────────────
    if (std::abs(inp.column) > DISC_THRESH || std::abs(inp.wheel) > DISC_THRESH) {
        disengageVert();
        disengageLat();
    }

    // ── Heading Hold: hdg_error → bank_demand → FBW attitude hold ───────────
    if (mode.lat == LatMode::HeadingHold) {
        constexpr float KP_HDG = 3.0f;
        float hdgErr = targets.headingDeg - st.hdgDeg;
        // normaliza para -180..+180
        while (hdgErr >  180.f) hdgErr -= 360.f;
        while (hdgErr < -180.f) hdgErr += 360.f;
        float bankDemand = std::clamp(KP_HDG * hdgErr, -25.f, 25.f);
        targets.bankDeg  = bankDemand;
        fbw.setTargetBank(bankDemand);
        inp.wheel = 0.f;
    }

    // ── Altitude Hold: cascata alt → VS → pitch ───────────────────────────────
    if (mode.vert == VertMode::AltitudeHold) {
        constexpr float KP_ALT = 2.0f;
        constexpr float KP_VS  = 0.008f;
        float altErr   = targets.altFt - st.altBaro;
        float vsDemand = std::clamp(KP_ALT * altErr, -MAX_VS_FPM, MAX_VS_FPM);
        float vsErr    = vsDemand - st.vsFpm;
        targets.pitchDeg = std::clamp(KP_VS * vsErr, -MAX_PITCH_AP, MAX_PITCH_AP);
    }

    // ── Loop externo de pitch (compartilhado att + alt) ──────────────────────
    if (mode.vert != VertMode::Off) {
        constexpr float KP = 0.050f;
        constexpr float KI = 0.006f;
        float err    = targets.pitchDeg - st.pitchDeg;
        _pitchInteg += err * dt;
        _pitchInteg  = std::clamp(_pitchInteg, -20.f, 20.f);
        inp.column   = std::clamp(KP * err + KI * _pitchInteg, -0.8f, 0.8f);
        inp.wheel    = 0.f;
    }

    // ── Speed Hold (autothrottle) ─────────────────────────────────────────────
    if (mode.thr == ThrMode::SpeedHold) {
        constexpr float KP_SPD = 0.005f;
        constexpr float KI_SPD = 0.002f;
        float spdErr     = targets.speedKt - st.casKt;
        _throttleInteg  += spdErr * dt;
        _throttleInteg   = std::clamp(_throttleInteg, -0.5f, 0.5f);
        float thr        = _baseThrottle + KP_SPD * spdErr + KI_SPD * _throttleInteg;
        thr              = std::clamp(thr, 0.f, 1.f);
        out.throttle[0]  = thr;
        out.throttle[1]  = thr;
        out.overrideThrottle = true;
    }

    // ── Flight Director ───────────────────────────────────────────────────────
    fd.pitchBar = targets.pitchDeg;
    fd.bankBar  = targets.bankDeg;
    fd.active   = isEngaged() || athrEngaged();

    out.column = inp.column;
    out.wheel  = inp.wheel;

    return isEngaged() || athrEngaged();
}
