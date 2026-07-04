#include "GuidanceModule.h"
#include <algorithm>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────

void GuidanceModule::engageAttitude(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    targets.pitchDeg = st.pitchDeg;
    targets.bankDeg  = st.rollDeg;
    _pitchInteg      = 0.f;
    _columnFilt      = 0.f;
    fbw.setTargetBank(targets.bankDeg);
    mode.vert = VertMode::AttitudeHold;
}

void GuidanceModule::engageAltitude(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    targets.altFt    = st.altBaro;
    targets.bankDeg  = st.rollDeg;
    targets.pitchDeg = st.pitchDeg;
    _pitchInteg      = 0.f;
    _columnFilt      = 0.f;
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
    _thrBoost        = 0.f;
    _errTimer        = 0.f;
    _lastSpdErr      = 0.f;
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
    _thrBoost       = 0.f;
    _errTimer       = 0.f;
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
        constexpr float KP_ALT      = 1.6f;
        constexpr float KP_VS       = 0.009f;
        constexpr float PITCH_RATE  = 4.0f;   // °/s

        float vsDemand;
        if (targets.vsManual) {
            vsDemand = std::clamp(targets.vsFpm, -MAX_VS_FPM, MAX_VS_FPM);
            float altErr = targets.altFt - st.altBaro;
            if (std::abs(altErr) < 200.f)
                targets.vsManual = false;
        } else {
            float altErr = targets.altFt - st.altBaro;
            vsDemand = std::clamp(KP_ALT * altErr, -MAX_VS_FPM, MAX_VS_FPM);
        }

        float vsErr     = vsDemand - st.vsFpm;
        float rawPitch  = std::clamp(KP_VS * vsErr, -MAX_PITCH_AP, MAX_PITCH_AP);
        // Rate limiter: pitch target não pula — máximo PITCH_RATE °/s
        float delta     = std::clamp(rawPitch - targets.pitchDeg, -PITCH_RATE * dt, PITCH_RATE * dt);
        targets.pitchDeg += delta;
    }

    // ── Loop externo de pitch (compartilhado att + alt) ──────────────────────
    if (mode.vert != VertMode::Off) {
        constexpr float KP      = 0.022f;  // reduzido: evita oscilação 1 Hz
        constexpr float KI      = 0.003f;
        constexpr float COL_LP  = 0.70f;   // filtro passa-baixo: α alto = mais suave
        float err      = targets.pitchDeg - st.pitchDeg;
        _pitchInteg   += err * dt;
        _pitchInteg    = std::clamp(_pitchInteg, -12.f, 12.f);
        float rawCol   = std::clamp(KP * err + KI * _pitchInteg, -0.8f, 0.8f);
        _columnFilt    = COL_LP * _columnFilt + (1.f - COL_LP) * rawCol;
        inp.column     = _columnFilt;
        inp.wheel      = 0.f;
    }

    // ── Speed Hold (autothrottle) ─────────────────────────────────────────────
    if (mode.thr == ThrMode::SpeedHold) {
        constexpr float KP_SPD    = 0.025f;  // P underspeed [throttle/kt]
        constexpr float KP_OVR    = 0.015f;  // P overspeed — mais suave (piso é prioridade)
        constexpr float KI_SPD    = 0.010f;  // I underspeed [throttle/(kt·s)]
        constexpr float STEP_GAIN = 0.018f;  // throttle adicionado por kt de erro a cada 1 s
        float spdErr = targets.speedKt - st.casKt;

        // Integrador em unidades de throttle: elimina erro em regime nos DOIS sentidos.
        // Underspeed integra 2× mais rápido (nunca ficar abaixo do alvo é prioridade).
        float kI = (spdErr > 0.f) ? KI_SPD : KI_SPD * 0.5f;
        _throttleInteg += spdErr * kI * dt;
        _throttleInteg  = std::clamp(_throttleInteg, -0.6f, 0.6f);

        // Boost de persistência: a cada 1 segundo que o erro não melhora,
        // somar o erro diretamente ao thrBoost (degrau de throttle)
        _errTimer += dt;
        if (_errTimer >= 1.0f) {
            _errTimer = 0.f;
            if (spdErr > 0.5f) {
                // Erro ainda presente e não melhorou mais que 0.5 kt → boost
                float improvement = _lastSpdErr - spdErr;  // positivo = melhorou
                if (improvement < 0.5f)
                    _thrBoost += spdErr * STEP_GAIN;
            } else if (spdErr < -0.5f) {
                // Overspeed: remove boost mais rápido (era 0.02 — não descia nunca)
                _thrBoost -= 0.05f;
            }
            _lastSpdErr = spdErr;
            _thrBoost   = std::clamp(_thrBoost, 0.f, 0.35f);
        }

        // P bidirecional: underspeed forte, overspeed mais suave
        float pTerm = (spdErr >= 0.f) ? spdErr * KP_SPD : spdErr * KP_OVR;
        float thr   = _baseThrottle + pTerm + _throttleInteg + _thrBoost;
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
