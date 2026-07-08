// ── Testes offline do AFCS (FlyByWire + GuidanceModule) ──────────────────────
// Modelo cinemático simples no lugar do JSBSim. Retorna != 0 se algo falhar.
#include "GuidanceModule.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>

static constexpr double D2R = 3.14159265358979 / 180.0;
static int gFail = 0;

static void check(bool ok, const char* what) {
    printf("  [%s] %s\n", ok ? "OK  " : "FAIL", what);
    if (!ok) ++gFail;
}

static float frand() { return (float)rand() / RAND_MAX * 2.f - 1.f; }

// ── 1. Estabilidade de velocidade (underspeed) por configuração ──────────────
static void testSpeedStability() {
    printf("\n== FBW: estabilidade de velocidade (spdVLo-10 kt, stick neutro) ==\n");
    struct Cfg { const char* name; bool gearDown; float flaps; float brake; bool expect; };
    const Cfg cfgs[] = {
        {"limpo (flaps 0, gear up)",      false, 0.f,     0.f, true },
        {"gear DOWN",                     true,  0.f,     0.f, false},
        {"flaps curso 1, gear up",        false, 1.f/6.f, 0.f, true },
        {"flaps curso 2, gear up",        false, 2.f/6.f, 0.f, false},
        {"flaps curso 1 + SPD BRK",       false, 1.f/6.f, 1.f, true },
    };
    for (const auto& c : cfgs) {
        FlyByWire fbw;
        FlyByWire::PilotInput inp;
        inp.gearCmd = c.gearDown; inp.flaps = c.flaps; inp.brake = c.brake;
        FlyByWire::AircraftState st;
        st.casKt = fbw.gains.spdVLo - 10.f;   // relativo ao limiar configurado
        st.altAgl = 5000.f; st.loadFactorNz = 1.f;
        FlyByWire::SurfaceCmd out;
        for (int i = 0; i < 50; i++) fbw.update(0.02f, inp, st, out);
        bool prot = out.elevLH > 0.05f;   // elevator + = picar
        check(prot == c.expect, c.name);
    }
}

// ── 2. Ruído no limiar de underspeed ──────────────────────────────────────────
static void testThresholdNoise() {
    printf("\n== FBW: chattering no limiar de spdVLo (ruido +-1 kt) ==\n");
    FlyByWire fbw;
    FlyByWire::PilotInput inp;
    float emin = 9.f, emax = -9.f;
    for (float t = 0.f; t < 30.f; t += 0.02f) {
        FlyByWire::AircraftState st;
        st.casKt = fbw.gains.spdVLo + frand();
        st.altAgl = 8000.f; st.loadFactorNz = 1.f + 0.02f*frand();
        st.pitchRateDegS = 0.2f*frand();
        FlyByWire::SurfaceCmd out;
        fbw.update(0.02f, inp, st, out);
        emin = std::fmin(emin, out.elevLH);
        emax = std::fmax(emax, out.elevLH);
    }
    printf("      faixa do elevator: %.4f\n", emax - emin);
    check(emax - emin < 0.05f, "sem chattering (faixa < 0.05)");
}

// ── 3. LNAV: sequenciamento de 2 waypoints ────────────────────────────────────
static void testLNAV() {
    printf("\n== LNAV: 2 waypoints, modelo cinematico ==\n");
    GuidanceModule gm;
    FlyByWire fbw;
    gm.fplan.push_back({-22.5, -42.8, "WPT1"});
    gm.fplan.push_back({-23.0, -42.3, "WPT2"});
    gm.engageLNAV();

    double lat = -22.81, lon = -43.25;
    float hdg = 0.f, bank = 0.f;
    const float V = 250.f, dt = 0.1f;

    for (float t = 0.f; t < 1800.f; t += dt) {
        FlyByWire::AircraftState st;
        st.latDeg = lat; st.lonDeg = lon;
        st.hdgDeg = hdg; st.rollDeg = bank;
        st.casKt = V; st.altBaro = 20000.f; st.altAgl = 15000.f;
        st.loadFactorNz = 1.f;
        FlyByWire::PilotInput inp;
        GuidanceModule::Output out;
        gm.update(dt, st, inp, fbw, out);

        bank += (fbw.targetBank() - bank) * dt / 1.5f;
        hdg = std::fmod(hdg + 1091.f * std::tan(bank * (float)D2R) / V * dt + 360.f, 360.f);
        double stepNm = V * dt / 3600.0;
        lat += stepNm * std::cos(hdg * D2R) / 60.0;
        lon += stepNm * std::sin(hdg * D2R) / 60.0 / std::cos(lat * D2R);

        if (gm.mode.lat != GuidanceModule::LatMode::Nav) break;
    }
    check(gm.mode.lat == GuidanceModule::LatMode::HeadingHold,
          "sequenciou os waypoints e finalizou em HDG HOLD");
    check(gm.activeWpt == 2, "activeWpt == 2 (plano completo)");
}

// ── 4. FLCH: subida e descida com captura ─────────────────────────────────────
static void runFlch(float alt0, float altTgt, float spdTgt, float engineK,
                    float* vMin, float* vMax, float* altFinal, bool* captured) {
    GuidanceModule gm;
    FlyByWire fbw;
    float alt = alt0, V = 250.f, pitch = 2.5f;

    FlyByWire::AircraftState st0;
    st0.casKt = V; st0.altBaro = alt; st0.pitchDeg = pitch;
    gm.engageSpeed(st0, 0.62f);
    gm.targets.speedKt = spdTgt;   // depois do engage (engage captura a atual)
    gm.targets.altFt   = altTgt;
    gm.engageFlch(st0, fbw);

    const float dt = 0.05f;
    *vMin = 9999.f; *vMax = 0.f; *captured = false;

    for (float t = 0.f; t < 900.f; t += dt) {
        FlyByWire::AircraftState st;
        st.casKt = V; st.altBaro = alt; st.pitchDeg = pitch;
        st.altAgl = alt - 20.f; st.loadFactorNz = 1.f;
        st.vsFpm = V * 1.68781f * std::sin((pitch - 2.5f) * (float)D2R) * 60.f;
        FlyByWire::PilotInput inp;
        inp.throttle[0] = inp.throttle[1] = 0.62f;
        GuidanceModule::Output out;
        gm.update(dt, st, inp, fbw, out);
        float thr = out.overrideThrottle ? out.throttle[0] : inp.throttle[0];

        pitch += (gm.targets.pitchDeg - pitch) * dt / 1.5f;
        float gamma = pitch - 2.5f;
        alt += V * 1.68781f * std::sin(gamma * (float)D2R) * dt;
        V   += (engineK * (thr - V / 400.f) - 0.55f * gamma) * dt;

        if (t > 10.f) { *vMin = std::fmin(*vMin, V); *vMax = std::fmax(*vMax, V); }
        if (gm.mode.vert == GuidanceModule::VertMode::AltitudeHold) *captured = true;
    }
    *altFinal = alt;
}

static void testFLCH() {
    printf("\n== FLCH: subida 20k->28k @ 250 kt ==\n");
    float vMin, vMax, altF; bool cap;
    runFlch(20000.f, 28000.f, 250.f, 9.f, &vMin, &vMax, &altF, &cap);
    printf("      alt final %.0f  V min/max %.1f/%.1f\n", altF, vMin, vMax);
    check(cap, "capturou (FLCH -> ALT HOLD)");
    check(std::fabs(altF - 28000.f) < 300.f, "altitude final < 300 ft do alvo");
    check(vMin > 225.f && vMax < 285.f, "velocidade segurada (225-285)");

    printf("\n== FLCH: descida 28k->12k @ 280 kt ==\n");
    runFlch(28000.f, 12000.f, 280.f, 9.f, &vMin, &vMax, &altF, &cap);
    printf("      alt final %.0f  V min/max %.1f/%.1f\n", altF, vMin, vMax);
    check(cap, "capturou (FLCH -> ALT HOLD)");
    check(std::fabs(altF - 12000.f) < 300.f, "altitude final < 300 ft do alvo");

    // Motor forte (baixa altitude, avião leve): mesmo com o pitch no teto a
    // velocidade fugia do alvo do A/THR — o throttle deve ceder para respeitar
    printf("\n== FLCH: subida 5k->15k @ 250 kt, MOTOR FORTE (2.5x) ==\n");
    runFlch(5000.f, 15000.f, 250.f, 22.f, &vMin, &vMax, &altF, &cap);
    printf("      alt final %.0f  V min/max %.1f/%.1f\n", altF, vMin, vMax);
    check(cap, "capturou (FLCH -> ALT HOLD)");
    check(vMax < 275.f, "velocidade respeitada mesmo com excesso de potencia (<275)");
}

int main() {
    printf("=== AFCS offline test suite ===\n");
    testSpeedStability();
    testThresholdNoise();
    testLNAV();
    testFLCH();
    printf("\n%s (%d falhas)\n", gFail == 0 ? "TODOS PASSARAM" : "HOUVE FALHAS", gFail);
    return gFail;
}
