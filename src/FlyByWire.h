#pragma once
#include <algorithm>
#include <cmath>

// ── Fly-By-Wire Normal Law — Embraer E195-E2 ─────────────────────────────────
//
// Independente do JSBSim. Recebe inputs brutos do piloto + estado lido do FDM,
// produz comandos de superfícies normalizados. Totalmente reutilizável para
// autopiloto / testador / calibrador automático.
//
// Leis implementadas:
//   Arfagem  — C* (fator de carga + taxa de arfagem blend)
//   Rolagem  — Rate demand / attitude hold com bank angle protection
//   Guinada  — Pedais diretos + yaw damper ativo
//   Proteção — Alpha floor, overspeed, bank limit 67°

class FlyByWire {
public:
    // ── Entradas do piloto ────────────────────────────────────────────────────
    struct PilotInput {
        float column    = 0.f;        // arfagem:  puxar+ / empurrar- [-1..+1]
        float wheel     = 0.f;        // rolagem:  direita+ / esquerda- [-1..+1]
        float pedals    = 0.f;        // guinada:  direita+ / esquerda- [-1..+1]
        float throttle[2] = {0.f,0.f};// potência por motor [0..1]
        float flaps     = 0.f;        // deflexão [0..1] → 7 passos (0=clean)
        float brake     = 0.f;        // freios [0..1]
        bool  gearCmd   = false;      // comando de trem (true=baixar)
        bool  reverser  = false;      // reversor ativo (botão Y)
    };

    // ── Estado da aeronave (lido do FDM a cada passo) ────────────────────────
    struct AircraftState {
        float pitchDeg      = 0.f;    // ângulo de arfagem [°]
        float rollDeg       = 0.f;    // ângulo de rolagem [°]
        float pitchRateDegS = 0.f;    // q [°/s]
        float rollRateDegS  = 0.f;    // p [°/s]
        float yawRateDegS   = 0.f;    // r [°/s]
        float loadFactorNz  = 1.f;    // Nz [g]
        float alphaRad      = 0.f;    // ângulo de ataque (asa) [rad]
        float betaDeg       = 0.f;    // ângulo de derrapagem lateral [°] — + = deslize para direita
        float casKt         = 0.f;    // velocidade calibrada [kt]
        float mach          = 0.f;
        float altAgl        = 0.f;    // altitude AGL [ft] — usado por pitch envelope / TSA
        bool  wow           = false;   // weight on wheels (main gear)
    };

    // ── Comandos de superfícies (escritos no JSBSim E195 cada frame) ─────────
    struct SurfaceCmd {
        float elevLH        = 0.f;    // fcs/elevator-lh-master       [-1..+1]
        float elevRH        = 0.f;    // fcs/elevator-rh-master       [-1..+1]
        float aileronL      = 0.f;    // fcs/left-aileron-master      [-1..+1]
        float aileronR      = 0.f;    // fcs/right-aileron-master     [-1..+1]
        float rudder        = 0.f;    // fcs/rudder-master            [-1..+1]
        float throttle[2]   = {0.f,0.f}; // fadec/throttle-cmd[N]    [0..1]
        float spoilerL      = 0.f;    // fcs/mfs{1,2,3}-pos-norm      [0..1]
        float spoilerR      = 0.f;    // fcs/mfs{8,9,10}-pos-norm     [0..1]
        float groundSpoiler = 0.f;    // fcs/mfs{4,5,6,7}-pos-norm    [0..1]
        float flaps         = 0.f;    // controls/flight/flaps        [0..0.75]
        float brakeL        = 0.f;
        float brakeR        = 0.f;
        float steerNoseDeg  = 0.f;    // fcs/steer-nose-deg[0]
        bool  gearDown      = false;  // gear/unit[N]/pos-norm → 1.0
        bool  reverser      = false;  // propulsion/engine[N]/reverser-angle-rad → π
    };

    // ── Parâmetros tuneáveis (expostos para autopiloto / calibrador) ──────────
    struct Gains {
        // Lei C* (arfagem)
        float pitchKp   = 1.8f;
        float pitchKi   = 0.30f;
        float pitchKd   = 0.15f;
        float cstarK    = 7.0f;   // Vo/g  [s] — peso da taxa de arfagem vs Nz
        float maxDemand = 1.5f;   // Nz demand máximo (acima/abaixo de 1g)

        // Rate demand / attitude hold (rolagem)
        // Sem derivativo: drateErr/dt amplifica ruído do sinal de phidot do JSBSim
        float rollKp         = 0.12f;  // P na taxa de rolagem (era 0.55 — saturava)
        float holdKp         = 0.03f;  // P para attitude hold (ganho do loop externo)
        float maxRollRateDegS= 22.f;   // taxa de rolagem máxima comandada

        // Yaw damper + auto-rudder (coordenação de curva)
        float yawDamperK = 0.06f;  // era 0.40 — 2.5°/s já mandava rudder cheio
        float betaKp     = 0.02f;  // correção de beta reduzida (sinal ruidoso)
    } gains;

    // ── Interface principal ───────────────────────────────────────────────────

    // Atualiza as leis e preenche 'out'. Chamar 1× por passo de sim.
    void update(float dt, const PilotInput& inp,
                const AircraftState& st, SurfaceCmd& out);

    // Reseta estado interno (integradores, etc.) — usar ao trocar de modo
    void reset();

    // ── Estado de diagnóstico (leitura para HUD / autopiloto) ────────────────
    float cstarDemand() const { return _cstarDem; }
    float cstarActual() const { return _cstarAct; }
    float pitchError()  const { return _cstarDem - _cstarAct; }
    float elevInteg()   const { return _elevInteg; }
    float targetBank()  const { return _targetBank; }
    bool  alphaFloorActive() const { return _alphaFloor; }
    bool  bankProtActive()   const { return _bankProt; }

private:
    static constexpr float DEG2RAD = 3.14159265f / 180.f;

    // Proteções de envelope
    static constexpr float ALPHA_PROT     = 0.22f;  // 12.6° — alpha floor início
    static constexpr float ALPHA_MAX      = 0.30f;  // 17.2° — alpha máximo estrutural
    static constexpr float BANK_NORM_LIM  = 67.f;   // deg — limite de bank em Normal Law
    static constexpr float BANK_HOLD_LIM  = 33.f;   // deg — acima retorna a 33°
    static constexpr float ROLL_DEADBAND  = 0.04f;
    static constexpr float MAX_STEER_DEG  = 70.f;   // steering do nariz na pista

    // Estado C* (arfagem)
    float _elevInteg    = 0.f;
    float _prevPitchErr = 0.f;
    float _cstarDem     = 1.f;
    float _cstarAct     = 1.f;
    bool  _alphaFloor   = false;

    // Estado rolagem
    float _targetBank   = 0.f;
    float _prevRollErr  = 0.f;
    bool  _bankProt     = false;

    // Yaw damper
    float _prevYawRate  = 0.f;

    // Gear
    bool _gearDown      = false;

    // Arranque — evita erro inicial quando Nz != 1g no IC
    bool _initialized   = false;
};
