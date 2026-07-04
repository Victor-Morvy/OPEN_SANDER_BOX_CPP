#pragma once
#include "FlyByWire.h"
#include <vector>
#include <string>

// ── GuidanceModule ────────────────────────────────────────────────────────────
//
// AFCS exportável — sem dependências de ImGui/GLFW/FDM.
// Compatível com E195-E2Sim: instanciar, chamar update() por frame.
//
// Eixos independentes:
//   Vertical  (vert): Off | AttitudeHold | AltitudeHold
//   Lateral   (lat):  Off | HeadingHold
//   Throttle  (thr):  Off | SpeedHold
//
// Auto-desconexão de vert+lat quando piloto move column/wheel > DISC_THRESH.
// Throttle permanece ativo até desengajamento explícito.

class GuidanceModule {
public:
    enum class LatMode  { Off, HeadingHold, Nav };
    enum class VertMode { Off, AttitudeHold, AltitudeHold, Flch };
    enum class ThrMode  { Off, SpeedHold };

    struct ModeState {
        LatMode  lat  = LatMode::Off;
        VertMode vert = VertMode::Off;
        ThrMode  thr  = ThrMode::Off;
    } mode;

    // Targets editáveis pelo painel a qualquer momento
    struct Targets {
        float altFt      = 10000.f;
        float headingDeg = 0.f;
        float speedKt    = 250.f;
        float pitchDeg   = 0.f;
        float bankDeg    = 0.f;
        float vsFpm      = 0.f;    // VS alvo manual (usado quando vsManual=true)
        bool  vsManual   = false;  // true = usa vsFpm em vez da cascata alt→VS
    } targets;

    // Flight Director — ativo enquanto qualquer modo ligado
    struct FlightDirector {
        float pitchBar = 0.f;
        float bankBar  = 0.f;
        bool  active   = false;
    } fd;

    // ── LNAV: flight plan ─────────────────────────────────────────────────────
    struct Waypoint {
        double      lat = 0.0, lon = 0.0;
        std::string name;       // ex: ident ICAO
    };
    std::vector<Waypoint> fplan;
    int   activeWpt = 0;
    float navDistNm = 0.f;      // distância ao waypoint ativo (leitura p/ HUD)
    float navBrgDeg = 0.f;      // bearing ao waypoint ativo

    struct Output {
        float column           = 0.f;
        float wheel            = 0.f;
        float throttle[2]      = {0.f, 0.f};
        bool  overrideThrottle = false;
    };

    bool isEngaged()   const { return mode.lat  != LatMode::Off ||
                                      mode.vert != VertMode::Off; }
    bool athrEngaged() const { return mode.thr  != ThrMode::Off; }

    void engageAttitude(const FlyByWire::AircraftState& st, FlyByWire& fbw);
    void engageAltitude(const FlyByWire::AircraftState& st, FlyByWire& fbw);
    void engageHeading (const FlyByWire::AircraftState& st);
    void engageSpeed   (const FlyByWire::AircraftState& st, float currentThrottle);
    void engageLNAV    ();                                          // requer fplan
    void engageFlch    (const FlyByWire::AircraftState& st, FlyByWire& fbw);

    void disengageVert();
    void disengageLat();
    void disengageThrottle();
    void disengageAll();

    // Permite forçar o throttle base (climb power boost)
    void setBaseThrottle(float thr) { _baseThrottle = thr; }
    float getBaseThrottle() const   { return _baseThrottle; }

    // FCU override: atualiza targets sem desengajar
    void overridePitch(float deg) { targets.pitchDeg = deg; }
    void overrideBank (float deg, FlyByWire& fbw) {
        targets.bankDeg = deg;
        fbw.setTargetBank(deg);
    }

    // Roda guidance a cada frame. Modifica inp e preenche out.
    // Retorna false se auto-desconexão ocorreu.
    bool update(float dt, const FlyByWire::AircraftState& st,
                FlyByWire::PilotInput& inp, FlyByWire& fbw, Output& out);

private:
    static constexpr float DISC_THRESH  = 0.15f;
    static constexpr float MAX_VS_FPM   = 3000.f;
    static constexpr float MAX_PITCH_AP = 12.f;

    float _pitchInteg    = 0.f;
    float _vsInteg       = 0.f;  // integrador VS→pitch (pitch de trim da subida)
    float _flchInteg     = 0.f;  // integrador FLCH: pitch de trim p/ manter CAS
    float _flchThr       = 0.f;  // throttle do FLCH (climb/idle + trim dinâmico)
    float _flchThrTrim   = 0.f;  // cede potência quando pitch satura e CAS foge
    float _columnFilt    = 0.f;
    float _throttleInteg = 0.f;
    float _baseThrottle  = 0.f;
    float _thrBoost      = 0.f;   // acúmulo por persistência (degrau a cada segundo)
    float _errTimer      = 0.f;   // cronômetro para o degrau de 1 s
    float _lastSpdErr    = 0.f;   // erro na última checagem
};
