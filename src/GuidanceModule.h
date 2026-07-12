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
    enum class LatMode  { Off, HeadingHold, Nav, Approach };
    enum class VertMode { Off, AttitudeHold, AltitudeHold, Flch, Approach, Vnav, Flare, GoAround };
    enum class ThrMode  { Off, SpeedHold };

    struct ModeState {
        LatMode  lat  = LatMode::Off;
        VertMode vert = VertMode::Off;
        ThrMode  thr  = ThrMode::Off;
    } mode;

    // ── AP coupling — separado da seleção de modo ────────────────────────────
    // Os modos (HDG/NAV/APP/ALT/VNAV/FLCH/ATT HOLD) SEMPRE computam o alvo
    // (fd.pitchBar/fd.bankBar), mesmo com apCoupled=false — isso é o Flight
    // Director: barras de comando calculadas o tempo todo, servindo de guia
    // pro piloto voar manualmente. Só quando apCoupled=true o GuidanceModule
    // escreve de fato em inp.column/inp.wheel (servos do AP). A/THR (mode.thr)
    // fica de fora dessa trava — no avião real ele é independente do AP.
    bool apCoupled = false;

    // Limita o bank comandado pelo AP (HDG/NAV/APP) a 15° em vez de 25°.
    // Não afeta override manual de bank (FCU) nem a autoridade bruta do FBW.
    bool lowBank = false;

    // ILS: main.cpp preenche estes campos a cada frame ANTES de chamar
    // update() (mesma convenção de sinal do HUD/PFD — ver computeApproachDevs
    // em main.cpp). valid=false → APP fica "armado" (asas niveladas / pitch
    // parado), aguardando o sinal entrar no cone.
    struct IlsData {
        bool  valid     = false;
        float locDevDeg = 0.f;   // + = voar PARA A DIREITA
        float gsDevDeg  = 0.f;   // + = voar PARA CIMA (abaixo do glide)
        float courseDeg = 0.f;   // proa de pouso (true) — necessário pro intercept de LOC
    } ils;

    // Targets editáveis pelo painel a qualquer momento
    struct Targets {
        float altFt      = 10000.f;
        float headingDeg = 0.f;
        float speedKt    = 250.f;
        float pitchDeg   = 0.f;
        float bankDeg    = 0.f;
        float vsFpm      = 0.f;    // VS alvo manual (usado quando vsManual=true)
        bool  vsManual   = false;  // true = usa vsFpm em vez da cascata alt→VS

        // IAS/Mach: qual unidade o A/THR e o FLCH perseguem. speedKt continua
        // sendo o alvo em nós quando speedIsMach=false; machTarget é o alvo
        // em Mach quando speedIsMach=true (independentes — trocar o toggle
        // não converte um no outro, o piloto redigita).
        bool  speedIsMach = false;
        float machTarget  = 0.78f;
    } targets;

    // Flight Director — pitchBar/bankBar são recalculados TODO frame que um
    // modo estiver selecionado, mesmo sem AP acoplado (active = fd armado,
    // NÃO "AP voando"). Ver apCoupled acima.
    struct FlightDirector {
        float pitchBar = 0.f;
        float bankBar  = 0.f;
        bool  active   = false;
    } fd;

    // ── LNAV/VNAV: flight plan ────────────────────────────────────────────────
    struct Waypoint {
        double      lat = 0.0, lon = 0.0;
        std::string name;       // ex: ident ICAO
        bool        hasAlt = false;
        float       altFt  = 0.f;   // restrição de altitude ("AT", sem ABOVE/BELOW)
    };
    std::vector<Waypoint> fplan;
    int   activeWpt = 0;
    float navDistNm = 0.f;      // distância ao waypoint ativo (leitura p/ HUD)
    float navBrgDeg = 0.f;      // bearing ao waypoint ativo
    int   vnavTargetWpt = -1;   // índice do próximo waypoint com hasAlt (leitura p/ HUD/painel)

    struct Output {
        float column           = 0.f;
        float wheel            = 0.f;
        float throttle[2]      = {0.f, 0.f};
        bool  overrideThrottle = false;
    };

    // FD armado (modo selecionado) — independente de apCoupled.
    bool isEngaged()   const { return mode.lat  != LatMode::Off ||
                                      mode.vert != VertMode::Off; }
    bool athrEngaged() const { return mode.thr  != ThrMode::Off; }

    void engageAttitude(const FlyByWire::AircraftState& st, FlyByWire& fbw);
    void engageAltitude(const FlyByWire::AircraftState& st, FlyByWire& fbw);
    void engageHeading (const FlyByWire::AircraftState& st);
    void engageSpeed   (const FlyByWire::AircraftState& st, float currentThrottle);
    void engageLNAV    ();                                          // requer fplan
    void engageFlch    (const FlyByWire::AircraftState& st, FlyByWire& fbw);
    void engageApproach(const FlyByWire::AircraftState& st, FlyByWire& fbw);  // acopla LOC+GS
    void engageVnav    (const FlyByWire::AircraftState& st, FlyByWire& fbw);  // requer fplan c/ altitude

    // TOGA / go-around: interrompe approach/pouso, nivela asas na proa atual
    // e sobe com potência e atitude fixas até a altitude de missed approach.
    void engageGoAround(const FlyByWire::AircraftState& st, FlyByWire& fbw);

    // Acopla/desacopla o AP aos modos já selecionados (não muda a seleção).
    // engage: se nenhum modo estiver selecionado, cai no básico (ATT HOLD).
    void engageAP (const FlyByWire::AircraftState& st, FlyByWire& fbw);
    void disengageAP() { apCoupled = false; }

    void disengageVert();
    void disengageLat();
    void disengageThrottle();
    void disengageAll();

    // Permite forçar o throttle base (climb power boost)
    void setBaseThrottle(float thr) { _baseThrottle = thr; }
    float getBaseThrottle() const   { return _baseThrottle; }

    // Leitura para HUD/painel: rampa do GS já capturada (tracking) vs armada
    // (nivelado esperando interceptar) — ver update() / engageApproach().
    bool  gsCaptured()       const { return _gsCaptured; }
    float goAroundTargetAlt() const { return _goAroundTargetAlt; }

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

    bool  _prevApCoupled = false;  // detecta borda de subida p/ reset bump-free do servo

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
    float _wowTimer      = 0.f;   // segundos consecutivos com peso nas rodas (auto-desconexão do A/THR)

    // Glideslope: arm→capture (só de baixo pra cima) e alvo de missed approach
    bool  _gsCaptured        = false;
    float _gsPrevDev         = 999.f;  // desvio do frame anterior — detecta a borda de captura
    float _goAroundTargetAlt = 0.f;    // altBaro alvo do TOGA (engage + 1500 ft)
};
