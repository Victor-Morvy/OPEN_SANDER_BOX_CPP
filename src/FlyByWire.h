#pragma once
#include <algorithm>
#include <cmath>

// ── Fly-By-Wire Normal Law — Embraer E195-E2 ─────────────────────────────────
//
// Independente do JSBSim. Recebe inputs brutos do piloto + estado lido do FDM,
// produz comandos de superfícies normalizados. Totalmente reutilizável para
// autopiloto / testador / calibrador automático.
//
// Leis implementadas (E195-E2 tem apenas duas — reversão Normal ↔ Direct):
//   NORMAL — C* pitch + envelope (+30°/−15°), rate demand roll + bank
//            protection, yaw damper + auto-rudder beta, e estabilidade de
//            velocidade: overspeed (>330 kt) → nariz sobe; underspeed
//            (<150 kt com gear UP) → nariz desce
//   DIRECT — stick → superfície puro (ganho fixo), sem aumentação

class FlyByWire {
public:
    // ── Lei ativa (reversão) ──────────────────────────────────────────────────
    enum class Law { Normal, Direct };
    Law law = Law::Normal;

    const char* lawName() const {
        return law == Law::Normal ? "NORMAL" : "DIRECT";
    }
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
        float altBaro       = 0.f;    // altitude barométrica MSL [ft]
        float vsFpm         = 0.f;    // velocidade vertical [ft/min] + = subindo
        float hdgDeg        = 0.f;    // proa verdadeira [°, 0=Norte, crescente CW]
        double latDeg       = 0.0;    // latitude  [°] — usado pelo LNAV
        double lonDeg       = 0.0;    // longitude [°] — usado pelo LNAV
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
        float yawDamperK = 0.10f;  // era 0.06 — aumentado para melhor amortecimento
        float betaKp     = 0.20f;  // P: 5°→1.0 rudder (aumentado de 0.14)
        float betaKi     = 0.06f;  // I: integrador elimina beta residual em regime
        float betaFiltA  = 0.80f;  // alfa do filtro passa-baixo do beta (0=sem filtro)

        // Estabilidade de velocidade (Normal Law) — envelope alargado e ganho
        // suavizado a pedido do Victor (330/150 com K=0.008 integrado puxava
        // o nariz de forma intrusiva); atua só no caminho P/D, sem trim.
        float spdVHi   = 350.f;   // kt — acima disso, comanda nariz para cima
        float spdVLo   = 130.f;   // kt — abaixo (com gear UP), nariz para baixo
        float spdStabK = 0.004f;  // column por kt além do limiar (soft, ±0.25 máx)
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

    // Permite ao autopiloto definir o banco-alvo diretamente
    void setTargetBank(float deg) { _targetBank = deg; }

    // ── Yaw Damper (YD) — toggle independente do AP/FGCS ──────────────────────
    // Doc AFCS: "YD funciona independentemente do AP e do flight guidance";
    // desligar o YD tira o amortecimento de Dutch roll E o auto-rudder (beta),
    // caindo no mesmo caminho de pedal direto usado no solo/Direct Law.
    void setYawDamper(bool on) { _ydOn = on; }
    bool yawDamperOn() const   { return _ydOn; }

    // ── Autobrake — OFF ou RTO (rejected takeoff) ──────────────────────────────
    // RTO arma freio máximo automático: se as manetes forem pra idle ainda no
    // solo em alta velocidade (decolagem abortada), aplica full brake sozinho
    // — mesmo gatilho (throttle idle + wow) que já auto-deploya o ground
    // spoiler, então os dois disparam juntos como no avião real. Não há níveis
    // de autobrake de pouso (1/2/3/MAX) — fora do escopo pedido.
    enum class Autobrake { Off, Rto };
    Autobrake autobrake = Autobrake::Off;
    bool rtoActive() const { return _rtoActive; }   // leitura p/ HUD (anunciação "RTO")

private:
    bool _ydOn = true;   // real: normalmente ligado por padrão até o piloto desligar
    bool _rtoActive = false;
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
    int   _envActive    = 0;     // pitch envelope engajado: +1 teto, -1 piso, 0 não

    // Estado rolagem
    float _targetBank   = 0.f;
    float _prevRollErr  = 0.f;
    bool  _bankProt     = false;

    // Yaw damper + beta correction
    float _prevYawRate  = 0.f;
    float _betaFilt     = 0.f;   // beta filtrado (passa-baixo)
    float _betaInteg    = 0.f;   // integrador de beta (elimina resíduo em regime)

    // Gear
    bool _gearDown      = false;

    // Arranque — evita erro inicial quando Nz != 1g no IC
    bool _initialized   = false;
};
