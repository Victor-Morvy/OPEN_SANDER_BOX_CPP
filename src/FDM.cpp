#include "FDM.h"

#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGPropulsion.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>
#include <models/propulsion/FGTurbine.h>
#include <input_output/FGPropertyManager.h>
#include <math/FGMatrix33.h>
#include <math/FGColumnVector3.h>
#include <math/FGQuaternion.h>

#include <cstdio>
#include <cmath>
#include <algorithm>

static constexpr double DEG2RAD = M_PI / 180.0;

// ── Helpers ──────────────────────────────────────────────────────────────────

double FDM::getD(const char* key) const {
    auto* n = _exec->GetPropertyManager()->GetNode(key);
    return n ? n->getDoubleValue() : 0.0;
}

void FDM::setD(const char* key, double val) {
    auto* n = _exec->GetPropertyManager()->GetNode(key, /*create=*/true);
    if (n) n->setDoubleValue(val);
}

// ── Construtor / destrutor ───────────────────────────────────────────────────

FDM::FDM()  : _exec(std::make_unique<JSBSim::FGFDMExec>()) {
    _exec->SetDebugLevel(0);
}
FDM::~FDM() = default;

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// C172P — interface original (inalterada)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

bool FDM::init(const std::string& aircraftPath,
               const std::string& enginePath,
               const std::string& systemsPath,
               const std::string& aircraftId,
               const InitialConditions& ic)
{
    _isE195 = false;
    _exec->SetAircraftPath(SGPath(aircraftPath));
    _exec->SetEnginePath  (SGPath(enginePath));
    _exec->SetSystemsPath (SGPath(systemsPath));

    if (!_exec->LoadModel(aircraftId)) {
        fprintf(stderr, "[FDM] Falha ao carregar '%s'\n", aircraftId.c_str());
        return false;
    }

    auto icio = _exec->GetIC();
    icio->SetLatitudeDegIC (ic.latDeg);
    icio->SetLongitudeDegIC(ic.lonDeg);
    icio->SetAltitudeAGLFtIC(ic.altFt);
    icio->SetPsiDegIC      (ic.headingDeg);
    icio->SetVcalibratedKtsIC(ic.airspeedKts);

    _exec->RunIC();
    _exec->GetPropulsion()->InitRunning(-1);

    setD("fcs/throttle-cmd-norm[0]", 0.75);
    setD("fcs/throttle-cmd-norm[1]", 0.75);

    _ready = true;
    printf("[FDM] C172P carregado. Alt=%.0f ft, IAS=%.0f kt, HDG=%.0f°\n",
           ic.altFt, ic.airspeedKts, ic.headingDeg);
    return true;
}

void FDM::setControls(const Controls& c) {
    setD("fcs/aileron-cmd-norm",       c.aileron);
    setD("fcs/elevator-cmd-norm",      c.elevator);
    setD("fcs/rudder-cmd-norm",       -c.rudder);
    setD("fcs/throttle-cmd-norm[0]",   c.throttle);
    setD("fcs/flap-cmd-norm",          c.flap);
    setD("fcs/brake-left-cmd-norm",    c.brake);
    setD("fcs/brake-right-cmd-norm",   c.brake);
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// E195 — inicialização
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

// Pre-declara todas as propriedades externas que o flight-control.xml
// referencia mas que não existem dentro do próprio JSBSim.
// Deve ser chamado ANTES de LoadModel().
void FDM::seedE195Properties()
{
    auto seed = [this](const char* path, double val) { setD(path, val); };

    // FBW elevator (escrito pelo FDM a cada frame; zero inicial)
    seed("fcs/elevator-cmd-fbw",                       0.0);

    // Autoflight / autopilot (zero até o módulo AP ser implementado)
    seed("it-autoflight/internal/aileron",             0.0);
    seed("it-autoflight/internal/rudder",              0.0);
    seed("autopilot/autobrake/output-brake",           0.0);
    seed("autopilot/autobrake/engaged",                0.0);

    // FADEC
    seed("fadec/throttle-cmd[0]",                      0.0);
    seed("fadec/throttle-cmd[1]",                      0.0);

    // Hidráulico (pressurizado por padrão)
    seed("systems/hydraulic/system[0]/pressurized",    1.0);
    seed("systems/hydraulic/system[1]/pressurized",    1.0);
    seed("systems/hydraulic/system[0]/pressure-psi", 3000.0);
    seed("systems/hydraulic/system[1]/pressure-psi", 3000.0);
    seed("systems/hydraulic/system[0]/emergency-accumulator", 0.0);
    seed("systems/hydraulic/system[1]/emergency-accumulator", 0.0);

    // PCUs de aileron (4 atuadores, todos ativos)
    seed("systems/actuators/aileron/pcu[0]/active",    1.0);
    seed("systems/actuators/aileron/pcu[1]/active",    1.0);
    seed("systems/actuators/aileron/pcu[2]/active",    1.0);
    seed("systems/actuators/aileron/pcu[3]/active",    1.0);

    // PCUs de profundor
    seed("systems/actuators/elevator/pcu[0]/active",   1.0);
    seed("systems/actuators/elevator/pcu[1]/active",   1.0);
    seed("systems/actuators/elevator/pcu[2]/active",   1.0);
    seed("systems/actuators/elevator/pcu[3]/active",   1.0);

    // PCUs de leme
    seed("systems/actuators/rudder/pcu[0]/active",     1.0);
    seed("systems/actuators/rudder/pcu[1]/active",     1.0);

    // MFS / spoilers (posições iniciais = recolhidos)
    seed("fcs/mfs-left-outboard-cmd-norm",             0.0);
    seed("fcs/mfs-left-inboard-cmd-norm",              0.0);
    seed("fcs/mfs-right-outboard-cmd-norm",            0.0);
    seed("fcs/mfs-right-inboard-cmd-norm",             0.0);

    // Serviços de solo / autopush (não usados)
    seed("services/chocks",                            0.0);
    seed("sim/model/autopush/connected",               0.0);
    seed("sim/model/autopush/autopush-cmd-norm",       0.0);
    seed("options/controls/realistic-nose-wheel-steering", 0.0);

    // Controles de voo FlightGear (escritos pelo FDM a cada frame)
    seed("controls/flight/rudder",                     0.0);
    seed("controls/flight/flaps",                      0.0);

    // Trem / freios / tiller
    seed("controls/gear/brake-left",                   0.0);
    seed("controls/gear/brake-right",                  0.0);
    seed("controls/gear/brake-parking",                0.0);
    seed("controls/gear/enable-tiller",                0.0);
    seed("controls/gear/tiller-pushed",                0.0);
    seed("controls/gear/tiller-cmd-norm",              0.0);

    // Propriedades de ground reactions criadas depois de RunIC
    seed("velocities/groundspeed-kt",                  0.0);
    seed("gear/unit[0]/castered",                      0.0);
    seed("gear/unit[0]/steering-angle-deg",            0.0);
}

void FDM::startE195Engines()
{
    auto prop = _exec->GetPropulsion();
    for (int i = 0; i < 2; ++i) {
        auto turb = std::dynamic_pointer_cast<JSBSim::FGTurbine>(prop->GetEngine(i));
        if (!turb) continue;
        turb->SetStarved(0);
        turb->SetRunning(true);
        turb->SetCutoff(false);
        prop->InitRunning(i);
    }
    _exec->SetPropertyValue("/fadec/throttle-cmd[0]", 0.50);
    _exec->SetPropertyValue("/fadec/throttle-cmd[1]", 0.50);
}

bool FDM::initE195(const std::string& aircraftPath,
                   const std::string& enginePath,
                   const InitialConditions& ic)
{
    _isE195 = false;
    _exec->SetAircraftPath(SGPath(aircraftPath));
    _exec->SetEnginePath  (SGPath(enginePath));
    _exec->SetSystemsPath (SGPath(aircraftPath));  // sistemas dentro da pasta aircraft

    // Semear propriedades externas ANTES de LoadModel()
    seedE195Properties();

    if (!_exec->LoadModel("E195")) {
        fprintf(stderr, "[FDM] Falha ao carregar E195\n");
        return false;
    }
    printf("[FDM] E195 carregado.\n");

    auto icio = _exec->GetIC();
    icio->SetLatitudeDegIC  (ic.latDeg);
    icio->SetLongitudeDegIC (ic.lonDeg);
    icio->SetAltitudeAGLFtIC(ic.altFt);
    icio->SetPsiDegIC       (ic.headingDeg);
    icio->SetVcalibratedKtsIC(ic.airspeedKts);

    _exec->RunIC();

    // Trem recolhido (partida no ar)
    setD("gear/unit[0]/pos-norm", 0.0);
    setD("gear/unit[1]/pos-norm", 0.0);
    setD("gear/unit[2]/pos-norm", 0.0);

    startE195Engines();

    _isE195 = true;
    _ready  = true;
    printf("[FDM] E195 pronto. Alt=%.0f ft, CAS=%.0f kt, HDG=%.0f°\n",
           ic.altFt, ic.airspeedKts, ic.headingDeg);
    return true;
}

// ── E195: escrita de comandos no JSBSim ──────────────────────────────────────

void FDM::setControlsE195(const FlyByWire::SurfaceCmd& cmd)
{
    auto clamp1  = [](double v){ return std::clamp(v, -1.0, 1.0); };
    auto clamp01 = [](double v){ return std::clamp(v,  0.0, 1.0); };

    // Profundor (LH e RH independentes)
    setD("fcs/elevator-lh-master", clamp1(cmd.elevLH));
    setD("fcs/elevator-rh-master", clamp1(cmd.elevRH));

    // Ailerons (diferencial: left=+cmd, right=-cmd para bank à direita)
    setD("fcs/left-aileron-master",  clamp1(cmd.aileronL));
    setD("fcs/right-aileron-master", clamp1(cmd.aileronR));

    // Leme — negado: convenção JSBSim E195 é oposta ao sinal aerodinâmico padrão
    setD("fcs/rudder-master",        clamp1(-cmd.rudder));
    setD("controls/flight/rudder",   clamp1(-cmd.rudder));

    // Throttle + reversor
    // reverser-angle-rad = π ativa a cascata de reverso no JSBSim E195;
    // o throttle-cmd controla a magnitude do empuxo reverso normalmente.
    double thr0 = clamp01(cmd.throttle[0]);
    double thr1 = clamp01(cmd.throttle[1]);
    _exec->SetPropertyValue("/fadec/throttle-cmd[0]", thr0);
    _exec->SetPropertyValue("/fadec/throttle-cmd[1]", thr1);
    const double revAngle = cmd.reverser ? M_PI : 0.0;
    _exec->SetPropertyValue("propulsion/engine[0]/reverser-angle-rad", revAngle);
    _exec->SetPropertyValue("propulsion/engine[1]/reverser-angle-rad", revAngle);

    // Spoilers laterais (MFS 1-3 esquerda, 8-10 direita)
    double sL = clamp01(cmd.spoilerL);
    setD("fcs/mfs1-pos-norm",  sL);
    setD("fcs/mfs2-pos-norm",  sL);
    setD("fcs/mfs3-pos-norm",  sL);
    double sR = clamp01(cmd.spoilerR);
    setD("fcs/mfs8-pos-norm",  sR);
    setD("fcs/mfs9-pos-norm",  sR);
    setD("fcs/mfs10-pos-norm", sR);

    // Ground spoilers (MFS 4-7)
    double gS = clamp01(cmd.groundSpoiler);
    setD("fcs/mfs4-pos-norm", gS);
    setD("fcs/mfs5-pos-norm", gS);
    setD("fcs/mfs6-pos-norm", gS);
    setD("fcs/mfs7-pos-norm", gS);

    // Flaps: 7 posições [0..0.75] (flight-control.xml lê controls/flight/flaps)
    setD("controls/flight/flaps", std::clamp((double)cmd.flaps, 0.0, 0.75));

    // Freios
    _exec->GetFCS()->SetLBrake(clamp01(cmd.brakeL));
    _exec->GetFCS()->SetRBrake(clamp01(cmd.brakeR));

    // Steering do nariz
    setD("fcs/steer-nose-deg[0]", cmd.steerNoseDeg);

    // Trem de pouso
    double gp = cmd.gearDown ? 1.0 : 0.0;
    setD("gear/unit[0]/pos-norm", gp);
    setD("gear/unit[1]/pos-norm", gp);
    setD("gear/unit[2]/pos-norm", gp);

    // Mantém motores ligados
    auto prop = _exec->GetPropulsion();
    for (int i = 0; i < 2; ++i) {
        auto turb = std::dynamic_pointer_cast<JSBSim::FGTurbine>(prop->GetEngine(i));
        if (!turb) continue;
        turb->SetStarved(0);
        turb->SetCutoff(false);
        if (!prop->GetEngine(i)->GetRunning())
            prop->InitRunning(i);
    }
}

// ── E195: leitura de estado para o FlyByWire ────────────────────────────────

FlyByWire::AircraftState FDM::getStateForFBW() const
{
    FlyByWire::AircraftState st;
    st.pitchDeg      = (float)getD("attitude/theta-deg");
    st.rollDeg       = (float)getD("attitude/phi-deg");
    st.pitchRateDegS = (float)(getD("velocities/thetadot-rad_sec") / DEG2RAD);
    st.rollRateDegS  = (float)(getD("velocities/phidot-rad_sec")   / DEG2RAD);
    st.yawRateDegS   = (float)(getD("velocities/r-aero-rad_sec")   / DEG2RAD);
    st.loadFactorNz  = (float)getD("accelerations/Nz");
    st.alphaRad      = (float)getD("aero/alpha-wing-rad");
    st.betaDeg       = (float)(getD("aero/beta-rad") / DEG2RAD);
    st.casKt         = (float)getD("velocities/vc-kts");
    st.mach          = (float)getD("velocities/mach");
    st.altAgl        = (float)getD("position/h-agl-ft");
    st.wow           = (getD("gear/unit[1]/WOW") > 0.5) ||
                       (getD("gear/unit[2]/WOW") > 0.5);
    return st;
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Passo de simulação (comum C172P / E195)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void FDM::step() {
    _exec->Run();
}

void FDM::setTerrainElevation(double ft) {
    setD("position/terrain-elevation-asl-ft", ft);
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Telemetria
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Telemetry FDM::getTelemetry() const
{
    Telemetry t;
    t.pitch    = getD("attitude/pitch-rad");
    t.roll     = getD("attitude/roll-rad");
    t.yaw      = getD("attitude/psi-rad");
    t.vNorth   = getD("velocities/v-north-fps");
    t.vEast    = getD("velocities/v-east-fps");
    t.vDown    = getD("velocities/v-down-fps");
    t.altAgl   = getD("position/h-agl-ft");
    t.altBaro  = getD("position/h-sl-ft");
    t.airspeed = getD("velocities/vt-fps") * 0.592484;
    t.cas      = getD("velocities/vc-kts");
    t.mach     = getD("velocities/mach");
    t.vsFpm    = -t.vDown * 60.0;
    t.loadNz   = getD("accelerations/Nz");
    t.alphaRad = getD("aero/alpha-wing-rad");
    t.betaDeg  = getD("aero/beta-rad") / DEG2RAD;
    t.flapPos  = getD("fcs/flap-pos-norm");

    if (_isE195) {
        t.gearPos  = getD("gear/unit[0]/pos-norm");
        t.throttle = (getD("fadec/throttle-cmd[0]") + getD("fadec/throttle-cmd[1]")) * 0.5;
        auto prop = _exec->GetPropulsion();
        for (int i = 0; i < 2; ++i) {
            auto turb = std::dynamic_pointer_cast<JSBSim::FGTurbine>(prop->GetEngine(i));
            t.n1[i] = turb ? turb->GetN1() : 0.0;
            t.n2[i] = turb ? turb->GetN2() : 0.0;
        }
    } else {
        t.gearPos  = getD("gear/gear-pos-norm");
        t.throttle = getD("fcs/throttle-cmd-norm[0]");
        t.rpm      = getD("propulsion/engine[0]/rpm");
    }

    return t;
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Pontos de contato (C172P)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

static constexpr double IN2M = 0.0254;
static constexpr double CG_X = 41.0, CG_Z = 36.5;

static FDM::GearPoint fromStruct(double sx, double sy, double sz,
                                  bool wow, const char* name) {
    return {
        (float)(  sy          * IN2M),
        (float)( (sz - CG_Z)  * IN2M),
        (float)( (sx - CG_X)  * IN2M),
        wow, name
    };
}

std::vector<FDM::GearPoint> FDM::getContactPoints() const
{
    if (_isE195) return {};   // modelo 3D do E195 ainda não implementado

    bool w0 = getD("gear/unit[0]/WOW") > 0.5;
    bool w1 = getD("gear/unit[1]/WOW") > 0.5;
    bool w2 = getD("gear/unit[2]/WOW") > 0.5;

    return {
        fromStruct( -6.8,     0.0, -19.5, w0, "NOSE"),
        fromStruct( 58.2,   -43.0, -15.5, w1, "LEFT_MAIN"),
        fromStruct( 58.2,    43.0, -15.5, w2, "RIGHT_MAIN"),
        fromStruct( 43.0, -214.8,  55.0, false, "LEFT_TIP"),
        fromStruct( 43.0,  214.8,  55.0, false, "RIGHT_TIP"),
    };
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Pausa / Reposicionamento
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void FDM::pause()
{
    if (!_exec) return;
    auto prop = _exec->GetPropagate();
    const auto& vs = prop->GetVState();
    _pausedU = vs.vUVW(1); _pausedV = vs.vUVW(2); _pausedW = vs.vUVW(3);
    _pausedP = vs.vPQR(1); _pausedQ = vs.vPQR(2); _pausedR = vs.vPQR(3);
    _exec->Hold();
}

void FDM::resume()
{
    if (!_exec) return;
    // Restaura vUVW/vPQR antes de despausa (ver deleteme.txt / updateEntity).
    // Se o heading mudou enquanto pausado, vInertialVelocity ainda é do heading antigo.
    // SetUVW() recomputa vInertialVelocity usando a atitude atual → sem alpha extremo.
    auto prop = _exec->GetPropagate();
    prop->SetUVW(1, _pausedU); prop->SetUVW(2, _pausedV); prop->SetUVW(3, _pausedW);
    prop->SetPQR(1, _pausedP); prop->SetPQR(2, _pausedQ); prop->SetPQR(3, _pausedR);
    _exec->Resume();
}

bool FDM::isPaused() const
{
    return _exec && _exec->Holding();
}

// Converte ângulos de Euler (psi, theta, phi) no frame local NED → quatérnion ECI
// e aplica no JSBSim via SetInertialOrientation().
// Derivação idêntica à do deleteme.txt (FlightDynamicsModule::setAttitude):
//   localNED → ECEF (via Ti2ec, rotação em torno do eixo polar) → ECI.
void FDM::applyOrientationToJSBSim(double psiRad, double thtRad, double phiRad)
{
    auto prop = _exec->GetPropagate();

    JSBSim::FGColumnVector3 euler;
    euler(JSBSim::FGJSBBase::ePhi) = phiRad;
    euler(JSBSim::FGJSBBase::eTht) = thtRad;
    euler(JSBSim::FGJSBBase::ePsi) = psiRad;
    JSBSim::FGQuaternion qLocal(euler);

    const double ea  = prop->GetEarthPositionAngle();
    const double cea = std::cos(ea), sea = std::sin(ea);
    JSBSim::FGMatrix33 Ti2ec( cea, sea, 0.0,
                              -sea, cea, 0.0,
                               0.0, 0.0, 1.0 );
    JSBSim::FGMatrix33 Tl2ec = prop->GetVState().vLocation.GetTl2ec();
    JSBSim::FGMatrix33 Ti2l  = Tl2ec.Transposed() * Ti2ec;
    prop->SetInertialOrientation(Ti2l.GetQuaternion() * qLocal);
}

// CAS [kt] → TAS [fps] usando MachFromVcalibrated (mesmo método do deleteme.txt).
double FDM::casToTasFps(float kcas) const
{
    if (kcas <= 0.f || !_exec) return 0.0;
    const double P        = _exec->GetAtmosphere()->GetPressure();   // PSF
    const double a        = _exec->GetAtmosphere()->GetSoundSpeed(); // FPS
    const double vcas_fps = static_cast<double>(kcas) / 0.592484;   // kt → fps
    const double mach     = _exec->GetAuxiliary()->MachFromVcalibrated(vcas_fps, P);
    return mach * a;
}

// Reposiciona a aeronave diretamente no estado JSBSim (funciona pausado ou em voo).
// Após chamar este método, chame resume() para despausa — o vUVW salvo aqui
// será restaurado antes do primeiro step, garantindo vInertialVelocity coerente.
void FDM::repositionInPlace(const Reposition& r)
{
    if (!_exec) return;
    auto prop = _exec->GetPropagate();

    // 1. Posição
    prop->SetLatitudeDeg (r.latDeg);
    prop->SetLongitudeDeg(r.lonDeg);
    prop->SetAltitudeASL (r.altFt);   // pés MSL

    // 2. Atitude: localNED Euler → ECI quaternion
    applyOrientationToJSBSim(
        r.headingDeg * DEG2RAD,
        static_cast<double>(r.pitchDeg) * DEG2RAD,
        static_cast<double>(r.rollDeg)  * DEG2RAD
    );

    // 3. Velocidade: CAS → TAS → frame do corpo (voo reto e nivelado)
    const double Vtas = casToTasFps(r.speedKcas);
    prop->SetUVW(1, Vtas); prop->SetUVW(2, 0.0); prop->SetUVW(3, 0.0);
    prop->SetPQR(1, 0.0);  prop->SetPQR(2, 0.0); prop->SetPQR(3, 0.0);

    // Salva vUVW/PQR para o resume() — após mudança de heading não gera alpha extremo
    _pausedU = Vtas; _pausedV = 0.0; _pausedW = 0.0;
    _pausedP = 0.0;  _pausedQ = 0.0; _pausedR = 0.0;
}

// ── Getters de estado (posição/atitude atual no JSBSim) ─────────────────────

double FDM::getLatDeg()  const { if (!_exec) return 0.0; return _exec->GetPropagate()->GetLatitudeDeg(); }
double FDM::getLonDeg()  const { if (!_exec) return 0.0; return _exec->GetPropagate()->GetLongitudeDeg(); }
double FDM::getAltFt()   const { if (!_exec) return 0.0; return _exec->GetPropagate()->GetAltitudeASL(); }
double FDM::getHdgDeg()  const { return _exec ? getD("attitude/psi-deg") : 0.0; }
