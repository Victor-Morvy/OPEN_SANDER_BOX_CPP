#pragma once
#include "FlyByWire.h"
#include <memory>
#include <string>
#include <vector>

namespace JSBSim { class FGFDMExec; }

// ── Telemetria ────────────────────────────────────────────────────────────────

struct Telemetry {
    double pitch    = 0;   // rad
    double roll     = 0;   // rad
    double yaw      = 0;   // rad (psi, North=0 crescente CW)
    double vNorth   = 0;   // fps
    double vEast    = 0;   // fps
    double vDown    = 0;   // fps
    double altAgl   = 0;   // ft
    double altBaro  = 0;   // ft (MSL baro)
    double airspeed = 0;   // kts (TAS)
    double cas      = 0;   // kts (CAS)
    double mach     = 0;
    double vsFpm    = 0;   // fpm (positivo = subindo)
    double throttle = 0;   // 0..1 (média motores)
    double rpm      = 0;   // C172P
    double n1[2]    = {};  // E195 — N1 por motor [%]
    double n2[2]    = {};  // E195 — N2 por motor [%]
    double flapPos  = 0;   // 0..1
    double gearPos  = 1;   // 0..1
    double loadNz   = 1;   // fator de carga [g]
    double alphaRad = 0;   // ângulo de ataque [rad]
    double betaDeg  = 0;   // ângulo de derrapagem lateral [°] — + = deslize para direita
};

// ── Condições de controle (C172P — interface legada) ─────────────────────────

struct Controls {
    double aileron  = 0;
    double elevator = 0;
    double rudder   = 0;
    double throttle = 0.75;
    double flap     = 0;
    double brake    = 0;
};

// ── Condições iniciais ────────────────────────────────────────────────────────

struct InitialConditions {
    double latDeg      = -22.8090;
    double lonDeg      = -43.2510;
    double altFt       =  3500.0;
    double headingDeg  =    97.0;
    double airspeedKts =   120.0;
};

// ── FDM wrapper ───────────────────────────────────────────────────────────────

class FDM {
public:
    FDM();
    ~FDM();

    // ── C172P (interface original) ───────────────────────────────────────────
    bool init(const std::string& aircraftPath,
              const std::string& enginePath,
              const std::string& systemsPath,
              const std::string& aircraftId,
              const InitialConditions& ic = {});

    void setControls(const Controls& c);

    // ── E195 com FBW ────────────────────────────────────────────────────────
    // Inicia o E195, semeia todas as propriedades externas exigidas pelo
    // flight-control.xml antes de LoadModel(), e liga os dois motores CF34.
    bool initE195(const std::string& aircraftPath,
                  const std::string& enginePath,
                  const InitialConditions& ic = {});

    // Escreve comandos de superfície no JSBSim (property names do E195)
    void setControlsE195(const FlyByWire::SurfaceCmd& cmd);

    // Lê estado do JSBSim → AircraftState para o FlyByWire
    FlyByWire::AircraftState getStateForFBW() const;

    // ── Comum ────────────────────────────────────────────────────────────────
    void step();
    void setTerrainElevation(double ft);
    Telemetry getTelemetry() const;
    bool ready() const { return _ready; }

    // ── Pausa / Reposicionamento ─────────────────────────────────────────────
    void pause();
    void resume();                  // restaura vUVW antes de despausa (ver deleteme.txt)
    bool isPaused() const;

    // Parâmetros de reposicionamento (editáveis na UI enquanto pausado)
    struct Reposition {
        double latDeg     = -22.809;
        double lonDeg     = -43.251;
        double altFt      =  3500.0;
        double headingDeg =    97.0;
        float  speedKcas  =   200.0f;
        float  pitchDeg   =     0.f;
        float  rollDeg    =     0.f;
    };

    // Aplica o reposicionamento diretamente no estado JSBSim
    // (funciona pausado ou em voo; ideal quando pausado para inspecionar antes de resumir)
    void repositionInPlace(const Reposition& r);

    double getLatDeg()  const;
    double getLonDeg()  const;
    double getAltFt()   const;
    double getHdgDeg()  const;

    struct GearPoint {
        float rx, ry, rz;
        bool  wow;
        const char* name;
    };
    std::vector<GearPoint> getContactPoints() const;

private:
    std::unique_ptr<JSBSim::FGFDMExec> _exec;
    bool   _ready     = false;
    bool   _isE195    = false;

    // Velocidade no frame do corpo salva no momento da pausa
    // (restaurada no unpause para que mudança de heading não gere alpha extremo)
    double _pausedU = 0, _pausedV = 0, _pausedW = 0;
    double _pausedP = 0, _pausedQ = 0, _pausedR = 0;

    double getD(const char* key) const;
    void   setD(const char* key, double val);

    void seedE195Properties();
    void startE195Engines();
    void applyOrientationToJSBSim(double psiRad, double thtRad, double phiRad);
    double casToTasFps(float kcas) const;
};
