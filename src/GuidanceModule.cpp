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
    _vsInteg         = st.pitchDeg;  // parte do pitch atual — engate sem degrau
    _columnFilt      = 0.f;
    fbw.setTargetBank(targets.bankDeg);
    mode.vert = VertMode::AltitudeHold;
}

void GuidanceModule::engageHeading(const FlyByWire::AircraftState& st)
{
    targets.headingDeg = st.hdgDeg;
    mode.lat = LatMode::HeadingHold;
}

void GuidanceModule::engageLNAV()
{
    if (fplan.empty()) return;
    if (activeWpt >= (int)fplan.size()) activeWpt = 0;
    mode.lat = LatMode::Nav;
}

void GuidanceModule::engageFlch(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    // Alvo de altitude vem de targets.altFt (campo ALT do painel);
    // alvo de velocidade vem de targets.speedKt (campo SPD do painel)
    targets.bankDeg  = st.rollDeg;
    targets.pitchDeg = st.pitchDeg;
    _flchInteg       = st.pitchDeg;   // parte do pitch atual — engate sem degrau
    _pitchInteg      = 0.f;
    _columnFilt      = 0.f;
    fbw.setTargetBank(targets.bankDeg);
    mode.vert = VertMode::Flch;
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
    _vsInteg    = 0.f;
    _flchInteg  = 0.f;
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

    // ── LNAV: bearing ao waypoint ativo → heading target ─────────────────────
    if (mode.lat == LatMode::Nav) {
        if (activeWpt >= (int)fplan.size()) {
            // Fim do plano: mantém a proa atual (vira HeadingHold)
            targets.headingDeg = st.hdgDeg;
            mode.lat = LatMode::HeadingHold;
        } else {
            constexpr double D2R = 3.14159265358979 / 180.0;
            const Waypoint& w = fplan[activeWpt];
            double dNorth = (w.lat - st.latDeg) * 60.0;                        // NM
            double dEast  = (w.lon - st.lonDeg) * 60.0 * std::cos(st.latDeg * D2R);
            navDistNm = (float)std::sqrt(dNorth*dNorth + dEast*dEast);
            navBrgDeg = (float)std::fmod(std::atan2(dEast, dNorth) / D2R + 360.0, 360.0);

            // Sequenciamento: dentro de 1.5 NM avança para o próximo waypoint
            if (navDistNm < 1.5f)
                activeWpt++;
            else
                targets.headingDeg = navBrgDeg;
        }
    }

    // ── Heading Hold: hdg_error → bank_demand → FBW attitude hold ───────────
    if (mode.lat == LatMode::HeadingHold || mode.lat == LatMode::Nav) {
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
        constexpr float KI_VS       = 0.0006f; // integrador: elimina erro de regime do VS
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

        // PI no VS: P amortece, I acumula o pitch de trim necessário para
        // sustentar o VS demandado (P puro deixava ~30% de déficit: 543/800)
        // Anti-windup POR SATURAÇÃO: congela o integrador apenas quando o pitch
        // demandado já saturou no mesmo sentido do erro. (Congelar por magnitude
        // do erro causava dois defeitos: windup mascarado na subida E deadlock
        // após captura com o integrador preso no trim errado.)
        float vsErr = vsDemand - st.vsFpm;
        float rawUn = KP_VS * vsErr + _vsInteg;
        if (std::abs(rawUn) < MAX_PITCH_AP || rawUn * vsErr < 0.f)
            _vsInteg += KI_VS * vsErr * dt;
        _vsInteg = std::clamp(_vsInteg, -6.f, 6.f);
        float rawPitch  = std::clamp(KP_VS * vsErr + _vsInteg, -MAX_PITCH_AP, MAX_PITCH_AP);
        // Rate limiter: pitch target não pula — máximo PITCH_RATE °/s
        float delta     = std::clamp(rawPitch - targets.pitchDeg, -PITCH_RATE * dt, PITCH_RATE * dt);
        targets.pitchDeg += delta;
    }

    // ── FLCH: throttle fixo (climb/idle), pitch controla a velocidade ────────
    if (mode.vert == VertMode::Flch) {
        constexpr float FLCH_KP    = 0.15f;   // ° por kt de erro
        constexpr float FLCH_KI    = 0.02f;   // °/s por kt (trim)
        constexpr float PITCH_RATE = 3.0f;    // °/s máx no target

        float altErr = targets.altFt - st.altBaro;
        if (std::abs(altErr) < 250.f) {
            // Captura: transição para Altitude Hold, devolve o throttle ao A/THR
            mode.vert        = VertMode::AltitudeHold;
            targets.vsManual = false;
            _vsInteg         = st.pitchDeg;
            if (mode.thr == ThrMode::SpeedHold) {
                _baseThrottle  = _flchThr;   // retoma speed hold do throttle atual
                _throttleInteg = 0.f;
                _thrBoost      = 0.f;
            }
        } else {
            bool climb = altErr > 0.f;
            _flchThr = climb ? 0.92f : 0.08f;

            // Speed-on-pitch: rápido → nariz sobe; lento → nariz desce
            float spdErr = st.casKt - targets.speedKt;   // + = rápido demais
            // Anti-windup por saturação: só congela quando o pitch demandado já
            // saturou NO MESMO sentido do erro (congelar por erro grande deixava
            // 27 kt de erro em regime — o integrador é mais necessário aí)
            float rawUnclamped = FLCH_KP * spdErr + _flchInteg;
            if (std::abs(rawUnclamped) < MAX_PITCH_AP || rawUnclamped * spdErr < 0.f)
                _flchInteg += FLCH_KI * spdErr * dt;
            _flchInteg = std::clamp(_flchInteg, -MAX_PITCH_AP, MAX_PITCH_AP);

            float rawPitch = std::clamp(FLCH_KP * spdErr + _flchInteg,
                                        -MAX_PITCH_AP, MAX_PITCH_AP);
            float delta = std::clamp(rawPitch - targets.pitchDeg,
                                     -PITCH_RATE * dt, PITCH_RATE * dt);
            targets.pitchDeg += delta;
        }
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

    // ── Speed Hold (autothrottle) — suspenso durante FLCH (throttle é fixo) ──
    if (mode.thr == ThrMode::SpeedHold && mode.vert != VertMode::Flch) {
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

    // ── FLCH: throttle fixo (climb power ou idle) ─────────────────────────────
    if (mode.vert == VertMode::Flch) {
        out.throttle[0]      = _flchThr;
        out.throttle[1]      = _flchThr;
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
