#pragma once
#include "FlyByWire.h"

// ── Autopilot ─────────────────────────────────────────────────────────────────
//
// Modos:
//   AttitudeHold — mantém pitch e bank capturados no engage (tecla Z / Select)
//   AltitudeHold — mantém altitude; cascata:
//                    alt_error → VS_demand → pitch_target → lei C* (FBW)
//                    roll: igual ao AttitudeHold (banco capturado)
//
// Auto-desconexão: coluna ou volante > DISC_THRESH desengaja qualquer modo.

class Autopilot {
public:
    enum class Mode { Off, AttitudeHold, AltitudeHold };

    Mode  mode           = Mode::Off;
    float targetPitchDeg = 0.f;
    float targetBankDeg  = 0.f;
    float targetAltFt    = 0.f;

    bool isEngaged() const { return mode != Mode::Off; }

    // Engaja Attitude Hold: captura pitch e bank atuais
    void engage(const FlyByWire::AircraftState& st, FlyByWire& fbw);

    // Engaja Altitude Hold: captura altitude atual; roll = bank atual
    void engageAlt(const FlyByWire::AircraftState& st, FlyByWire& fbw);

    void disengage();

    // Modifica inp antes do FBW. Retorna true se ainda engajado.
    bool update(float dt, const FlyByWire::AircraftState& st,
                FlyByWire::PilotInput& inp, FlyByWire& fbw);

private:
    static constexpr float DISC_THRESH  = 0.15f;
    static constexpr float MAX_VS_FPM   = 1500.f;  // VS demand máximo [ft/min]
    static constexpr float MAX_PITCH_AP =  8.f;     // pitch target máximo do AP [°]

    float _pitchInteg = 0.f;
};
