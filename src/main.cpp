// WebFlight C++ — Fase 3b
// Terrain real: AWS Terrarium elevation + ESRI satellite texture via TileManager.
// Coordenadas: X=Leste, Y=cima, Z=Sul(aft); North=-Z
// Avião na origem do render; câmera chase segue atrás.
// FBW Normal Law — Embraer E195-E2 (C* arfagem, rate demand rolagem, yaw damper)

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "FDM.h"
#include "GuidanceModule.h"
#include "Sky.h"
#include "TileManager.h"
#include "GeoProj.h"
#include "Clouds.h"
#include "AirportManager.h"
#include "OSMManager.h"
#include "AcModel.h"
#include "PostFX.h"
#include "MiniMap.h"

#include <cstdio>
#include <cmath>
#include <algorithm>
#include <vector>

#ifndef AIRCRAFT_PATH
#  define AIRCRAFT_PATH "aircraft"
#endif
#ifndef ENGINE_PATH
#  define ENGINE_PATH "engine"
#endif
#ifndef SYSTEMS_PATH
#  define SYSTEMS_PATH "systems"
#endif
#ifndef MODELS_PATH
#  define MODELS_PATH "data/models"
#endif

static constexpr int    JSB_HZ    = 120;
static constexpr double RAD2DEG   = 180.0 / 3.14159265358979;
static constexpr double FT2M      = 0.3048;
static constexpr float  CTRL_RATE = 1.5f;
static constexpr float  CTRL_RET  = 3.0f;

// ── Shaders ───────────────────────────────────────────────────────────────────

// Shader do modelo 3D E195 — pos/nrm/uv, textura + diffuse flat
static const char* MODEL_VERT = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec2 aUV;
out vec3 vNrm;
out vec2 vUV;
uniform mat4 uMVP;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vNrm = aNrm;
    vUV  = aUV;
}
)glsl";

static const char* MODEL_FRAG = R"glsl(
#version 330 core
in vec3 vNrm; in vec2 vUV;
out vec4 F;
uniform sampler2D uTex;
uniform int       uHasTex;
uniform float     uAlpha;
void main() {
    vec3 sun = normalize(vec3(0.4, 1.0, 0.3));
    float diff = max(dot(normalize(vNrm), sun), 0.0) * 0.6 + 0.4;
    vec4 base = (uHasTex == 1) ? texture(uTex, vUV) : vec4(0.8, 0.82, 0.85, 1.0);
    F = vec4(base.rgb * diff, base.a * uAlpha);
}
)glsl";

// Shader de partes animadas — igual ao estático mas rotaciona em torno de pivot
static const char* ANIM_VERT = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec2 aUV;
out vec3 vNrm; out vec2 vUV;
uniform mat4  uMVP;
uniform vec3  uPivot;
uniform vec3  uAxis;
uniform float uAngle;
vec3 rotAround(vec3 p) {
    vec3 d = p - uPivot;
    float c = cos(uAngle), s = sin(uAngle);
    return uPivot + d*c + cross(uAxis,d)*s + uAxis*dot(uAxis,d)*(1.0-c);
}
void main() {
    gl_Position = uMVP * vec4(rotAround(aPos), 1.0);
    float c = cos(uAngle), s = sin(uAngle);
    vNrm = aNrm*c + cross(uAxis,aNrm)*s + uAxis*dot(uAxis,aNrm)*(1.0-c);
    vUV  = aUV;
}
)glsl";

// ANIM_FRAG shares MODEL_FRAG — same texture/lighting logic (set below)

// Shader genérico colorido — usado para pontos de contato
static const char* COL_VERT = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aCol;
out vec3 vCol;
uniform mat4 uMVP;
uniform float uPtSize;
void main() {
    gl_Position  = uMVP * vec4(aPos,1.0);
    gl_PointSize = uPtSize;
    vCol = aCol;
}
)glsl";

static const char* COL_FRAG = R"glsl(
#version 330 core
in vec3 vCol; out vec4 F;
uniform bool uCircle;
void main() {
    if(uCircle){
        vec2 c = gl_PointCoord * 2.0 - 1.0;
        if(dot(c,c)>1.0) discard;
    }
    F = vec4(vCol,1.0);
}
)glsl";

// ── C172P: wireframe estrutural (IN) → render (m) ────────────────────────────
// Frame render: X=direita, Y=cima, Z=cauda(aft/Sul) — nariz aponta para -Z
// Conversão (relativo ao CG=(41,0,36.5)):
//   rX = struct_Y * IN2M
//   rY = (struct_Z - 36.5) * IN2M
//   rZ = (struct_X - 41.0) * IN2M
//
// Pontos (render metros):
//   NARIZ TIP:     (-35,0,36)  → (0.000, -0.013, -1.930)
//   NARIZ GEAR:    (-6.8,0,-19.5) → (0.000, -1.422, -1.219)
//   LEFT MAIN:     (58.2,-43,-15.5) → (-1.092, -1.321, +0.437)
//   RIGHT MAIN:    (58.2,+43,-15.5) → (+1.092, -1.321, +0.437)
//   LEFT TIP:      (43,-214.8,55) → (-5.456, +0.470, +0.051)
//   RIGHT TIP:     (43,+214.8,55) → (+5.456, +0.470, +0.051)
//   TAIL:          (155,0,50)  → (0.000, +0.343, +2.896)
//   FIN TOP:       (158,0,88)  → (0.000, +1.308, +2.972)
//   LHSTAB:        (155,-60,50) → (-1.524, +0.343, +2.896)
//   RHSTAB:        (155,+60,50) → (+1.524, +0.343, +2.896)
//   PROP_L:        (-35,-37,36) → (-0.940, -0.013, -1.930)
//   PROP_R:        (-35,+37,36) → (+0.940, -0.013, -1.930)

static const float AC_V[] = {
//     rX        rY       rZ      R     G     B
    0.000f, -0.013f, -1.930f, .90f,.90f,.95f,  // 0  nariz
    0.000f,  0.000f, -0.800f, .80f,.80f,.85f,  // 1  cabine (nariz)
    0.000f,  0.000f, +1.500f, .75f,.75f,.80f,  // 2  cabine (cauda)
    0.000f, +0.343f, +2.896f, .60f,.62f,.65f,  // 3  cauda
    0.000f, +1.308f, +2.972f, .70f,.72f,.75f,  // 4  topo da deriva
   -5.456f, +0.470f, +0.051f,  0.f,.85f,.15f,  // 5  ponta porto   (verde nav)
   +5.456f, +0.470f, +0.051f, 1.0f,.10f,.10f,  // 6  ponta boreste (vermelho nav)
   -1.100f, +0.470f, +0.051f, .70f,.70f,.75f,  // 7  raiz asa porto
   +1.100f, +0.470f, +0.051f, .70f,.70f,.75f,  // 8  raiz asa boreste
   -1.524f, +0.343f, +2.896f, .65f,.65f,.70f,  // 9  H-stab porto
   +1.524f, +0.343f, +2.896f, .65f,.65f,.70f,  // 10 H-stab boreste
   -0.940f, -0.013f, -1.930f, .88f,.88f,.92f,  // 11 hélice porto
   +0.940f, -0.013f, -1.930f, .88f,.88f,.92f,  // 12 hélice boreste
};
static const unsigned short AC_I[] = {
    0,1, 1,2, 2,3,       // fuselagem (nariz → cabine → cauda)
    3,4,                  // deriva
    3,9, 3,10, 9,10,     // estabilizador horizontal
    5,7, 7,8, 8,6,       // asa: tip-raiz-raiz-tip
    0,11, 11,12, 12,0,   // hélice (triângulo)
};
static constexpr int AC_NI = (int)(sizeof(AC_I)/sizeof(AC_I[0]));

// ── GL helpers ────────────────────────────────────────────────────────────────

static GLuint makeShader(GLenum t, const char* s){
    GLuint sh=glCreateShader(t);
    glShaderSource(sh,1,&s,nullptr); glCompileShader(sh);
    GLint ok=0; glGetShaderiv(sh,GL_COMPILE_STATUS,&ok);
    if(!ok){char b[512];glGetShaderInfoLog(sh,512,nullptr,b);fprintf(stderr,"[GL] %s\n",b);}
    return sh;
}
static GLuint makeProgram(const char* vs,const char* fs){
    GLuint v=makeShader(GL_VERTEX_SHADER,vs),f=makeShader(GL_FRAGMENT_SHADER,fs);
    GLuint p=glCreateProgram();
    glAttachShader(p,v); glAttachShader(p,f); glLinkProgram(p);
    glDeleteShader(v); glDeleteShader(f); return p;
}

// ── Input ─────────────────────────────────────────────────────────────────────

struct Axes {
    float ail=0, elv=0, rdr=0, thr=0.50f, brk=0, flaps=0;
    float trimElev=0, trimAil=0;
    bool gear=false;
    bool pauseBtn=false;
    bool apBtn=false;     // Select — Attitude Hold
    bool altBtn=false;    // Circle/B  — Altitude Hold
    bool reverser=false;
    // ── Câmera orbital (ângulos persistentes; L2+R2 para ajustar)
    float camYaw   = 0.f;    // 0=atrás, π=nariz; persiste ao soltar gatilhos
    float camPitch = 0.26f;  // ~15° acima; clampado em ±80°
    bool  trigsHeld = false; // L2+R2 seguros agora (suprime stick nas outras funções)
    // ── Cockpit: olhar com a cabeça (stick dir. sem gatilhos)
    float headYaw   = 0.f;  // yaw local relativo ao nariz
    float headPitch = 0.f;  // pitch local
    // ── Toggle câmera (Círculo/B)
    bool cockpit = false;
};

static bool gGearPrev = false;
static bool gGearDown = false;
static float gFlapsStep = 0.f;   // 0..1 em passos de 1/6

// ── Joystick ──────────────────────────────────────────────────────────────────
// Mapeamento atual (right stick gamepad):
//   Eixo 0 = elevator  (pitch,  +puxar/cabrar)   ← right stick Y
//   Eixo 1 = aileron   (roll,   +direita)         ← right stick X
//   Eixo 2 = throttle  (taxa: +1=cima → sobe, −1=baixo → desce)
//   Eixo 3 = rudder    (yaw,   +direita)
// Ajuste as constantes abaixo se os eixos forem diferentes.

static constexpr int   JS_AIL = 0, JS_ELV = 1, JS_THR = 3, JS_RDR = 2;
static constexpr float JS_DZ  = 0.08f;   // deadzone 8%
static constexpr float JS_THR_RATE = 0.6f;  // unidades/s com stick a fundo

static float applyDZ(float v){
    if(fabsf(v) < JS_DZ) return 0.f;
    float s = v > 0.f ? 1.f : -1.f;
    return s * (fabsf(v) - JS_DZ) / (1.f - JS_DZ);
}

static bool readJoystick(Axes& a, float dt){
    // Encontra o primeiro joystick conectado
    int jid = -1;
    for(int i = GLFW_JOYSTICK_1; i <= GLFW_JOYSTICK_LAST; ++i)
        if(glfwJoystickPresent(i)){ jid = i; break; }
    if(jid < 0) return false;

    // Imprime nome e eixos na primeira detecção
    static int lastJid = -1;
    if(jid != lastJid){
        lastJid = jid;
        printf("[Joy] Conectado: %s\n", glfwGetJoystickName(jid));
        int ac; const float* ax = glfwGetJoystickAxes(jid, &ac);
        printf("[Joy] %d eixos detectados\n", ac);
        for(int i=0;i<ac;++i) printf("  eixo[%d] = %.3f\n", i, ax[i]);
    }

    int ac; const float* ax = glfwGetJoystickAxes(jid, &ac);
    if(!ax) return false;

    // ── Gatilhos L2/R2 (eixos 4 e 5: -1=solto → +1=fundo) ──────────────────────
    float l2val = (ac > 4) ? ax[4] : -1.f;
    float r2val = (ac > 5) ? ax[5] : -1.f;
    a.trigsHeld = (l2val > 0.f && r2val > 0.f);

    // Stick esquerdo: sempre voo
    if(JS_AIL < ac) a.ail = applyDZ(ax[JS_AIL]);
    if(JS_ELV < ac) a.elv = applyDZ(ax[JS_ELV]);

    // Stick direito: comportamento depende do modo
    float rX = (ac > 2) ? applyDZ(ax[2]) : 0.f;
    float rY = (ac > 3) ? applyDZ(ax[3]) : 0.f;

    if (a.trigsHeld) {
        // L2+R2 seguros → orbita câmera externa (ângulos persistem ao soltar)
        constexpr float CAM_ROT = 1.8f;
        a.camYaw   += -rX * CAM_ROT * dt;
        a.camPitch  = std::clamp(a.camPitch + rY * CAM_ROT * dt, -1.4f, 1.4f);
    } else if (a.cockpit) {
        // Modo cockpit (sem gatilhos) → olhar com a cabeça
        constexpr float HEAD_ROT = 1.5f;
        a.headYaw   = std::clamp(a.headYaw   + rX * HEAD_ROT * dt,  -1.2f,  1.2f);
        a.headPitch = std::clamp(a.headPitch + (-rY)* HEAD_ROT * dt, -0.6f,  0.5f);
    } else {
        // Voo normal → leme + manete
        a.rdr = rX;
        a.thr = std::clamp(a.thr + (-rY) * JS_THR_RATE * dt, 0.f, 1.f);
    }

    int bc; const unsigned char* bt = glfwGetJoystickButtons(jid, &bc);
    if(bt){
        auto B = [&](int i) -> bool { return i < bc && bt[i]; };

        // ── Superfícies ──────────────────────────────────────────────────────
        a.brk     = B(0) ? 1.f : 0.f;                    // A / Cruz = freio

        static bool prevGear=false;
        if(B(2) && !prevGear) gGearDown = !gGearDown;    // X / □ = trem
        prevGear = B(2);

        // ── Flaps (LB/L1=subir, RB/R1=descer) ───────────────────────────────
        static bool prevLB=false, prevRB=false;
        if(B(4) && !prevLB) gFlapsStep = std::max(0.f, gFlapsStep - 1.f/6.f);
        if(B(5) && !prevRB) gFlapsStep = std::min(1.f, gFlapsStep + 1.f/6.f);
        prevLB = B(4); prevRB = B(5);
        a.flaps = gFlapsStep;

        // ── Trim (D-pad: ↑↓ = pitch, ←→ = roll) ─────────────────────────────
        constexpr float JT = 0.25f;
        if(B(10)) a.trimElev = std::max(-1.f, a.trimElev - JT*dt); // ↑ = picar
        if(B(12)) a.trimElev = std::min( 1.f, a.trimElev + JT*dt); // ↓ = cabrar
        if(B(13)) a.trimAil  = std::max(-1.f, a.trimAil  - JT*dt); // ← = roll esq
        if(B(11)) a.trimAil  = std::min( 1.f, a.trimAil  + JT*dt); // → = roll dir

        // ── Start = pausa | Select = ATT | Circle/B = ALT ────────────────────
        a.pauseBtn = B(7);
        a.apBtn    = B(6);
        a.altBtn   = B(1);

        // ── Y/△ = reversor (toggle; só deploya no solo — gate no FBW) ────────
        static bool prevRev = false;
        if(B(3) && !prevRev) a.reverser = !a.reverser;
        prevRev = B(3);

        // ── R3 click = resetar câmera ao padrão ──────────────────────────────
        // B(9)=R3 Xbox/DInput  B(11)=R3 PS4/DualSense
        static bool prevR3 = false;
        bool r3 = B(9) || B(11);
        if(r3 && !prevR3){
            a.camYaw   = 0.f;  a.camPitch  = 0.26f;  // órbita → padrão
            a.headYaw  = 0.f;  a.headPitch = 0.f;     // cabeça → frente
        }
        prevR3 = r3;

        // ── Círculo/B = alternar câmera cockpit / externa ─────────────────────
        static bool prevCircle = false;
        bool circle = B(1);
        if(circle && !prevCircle) a.cockpit = !a.cockpit;
        prevCircle = circle;
    }
    return true;
}

static void updateAxes(Axes& a, GLFWwindow* w, float dt){
    auto K=[&](int k){return glfwGetKey(w,k)==GLFW_PRESS;};

    bool hasJoy = readJoystick(a, dt);

    // Teclado só atua nos eixos que o joystick não cobriu
    if(!hasJoy){
        // Rolagem (wheel)
        if(K(GLFW_KEY_RIGHT)) a.ail=std::min(1.f,a.ail+CTRL_RATE*dt);
        else if(K(GLFW_KEY_LEFT)) a.ail=std::max(-1.f,a.ail-CTRL_RATE*dt);
        else a.ail*=std::max(0.f,1.f-CTRL_RET*dt);

        // Arfagem (column) — DOWN = puxar = nariz sobe (convenção stick)
        if(K(GLFW_KEY_DOWN)) a.elv=std::min(1.f,a.elv+CTRL_RATE*dt);
        else if(K(GLFW_KEY_UP)) a.elv=std::max(-1.f,a.elv-CTRL_RATE*dt);
        else a.elv*=std::max(0.f,1.f-CTRL_RET*dt);

        // Pedais (rudder / nariz)
        if(K(GLFW_KEY_D)) a.rdr=std::min(1.f,a.rdr+CTRL_RATE*dt);
        else if(K(GLFW_KEY_A)) a.rdr=std::max(-1.f,a.rdr-CTRL_RATE*dt);
        else a.rdr*=std::max(0.f,1.f-CTRL_RET*dt);

        // Potência
        if(K(GLFW_KEY_W)) a.thr=std::min(1.f,a.thr+.5f*dt);
        if(K(GLFW_KEY_S)) a.thr=std::max(0.f,a.thr-.5f*dt);
    } else {
        // Com joystick: teclado W/S ainda ajusta throttle incrementalmente
        if(K(GLFW_KEY_W)) a.thr=std::min(1.f,a.thr+.3f*dt);
        if(K(GLFW_KEY_S)) a.thr=std::max(0.f,a.thr-.3f*dt);
    }

    a.brk = (a.brk > 0.f) ? a.brk : (K(GLFW_KEY_B)?1.f:0.f);

    // Trem de pouso (G — toggle)
    bool gNow = K(GLFW_KEY_G);
    if(gNow && !gGearPrev) gGearDown = !gGearDown;
    gGearPrev = gNow;
    a.gear = gGearDown;

    // Trim (numpad 8=picar, 5=cabrar — mesmo sentido das setas: 8=UP=nariz desce)
    constexpr float TRIM_RATE = 0.25f;
    if(K(GLFW_KEY_KP_8)) a.trimElev=std::max(-1.f,a.trimElev-TRIM_RATE*dt);
    if(K(GLFW_KEY_KP_5)) a.trimElev=std::min( 1.f,a.trimElev+TRIM_RATE*dt);
    if(K(GLFW_KEY_KP_4)) a.trimAil =std::max(-1.f,a.trimAil -TRIM_RATE*dt);
    if(K(GLFW_KEY_KP_6)) a.trimAil =std::min( 1.f,a.trimAil +TRIM_RATE*dt);

    // Flaps (F = subir, V = descer, em 6 passos)
    static bool fPrev=false, vPrev=false;
    bool fNow=K(GLFW_KEY_F), vNow=K(GLFW_KEY_V);
    if(fNow&&!fPrev) gFlapsStep=std::max(0.f,gFlapsStep-1.f/6.f);
    if(vNow&&!vPrev) gFlapsStep=std::min(1.f,gFlapsStep+1.f/6.f);
    fPrev=fNow; vPrev=vNow;
    a.flaps = gFlapsStep;

    // Reversor (R — toggle; só deploya no solo — gate no FBW)
    static bool rPrev=false;
    bool rNow=K(GLFW_KEY_R);
    if(rNow && !rPrev) a.reverser = !a.reverser;
    rPrev = rNow;
}

// ── Posição mundial (double precision) ───────────────────────────────────────
// Convenção: X=Leste(East), Y=altitude(acima terreno), Z=-Norte(North=-Z)

struct WorldPos { double x=0, y=0, z=0; };

static void integratePos(WorldPos& p, const Telemetry& t, double dt){
    p.x += t.vEast  * FT2M * dt;    // East  → +X
    p.z -= t.vNorth * FT2M * dt;    // North → -Z  (convenção render)
    p.y  = t.altAgl * FT2M;         // AGL direto — sem drift
}

// ── Câmera chase ──────────────────────────────────────────────────────────────
// Frame render: Z+ = cauda(Sul), Z- = nariz(Norte)
// Para heading Norte (yaw=0): nariz aponta para -Z → câmera fica em +Z

static glm::vec3 aircraftForward(const Telemetry& t){
    // Vetor "nariz" no espaço render (X=leste, Y=cima, Z=-Norte)
    // yaw=0(N): fwd=(0,0,-1); yaw=90°(E): fwd=(1,0,0); yaw=180°(S): fwd=(0,0,1)
    float y = (float)t.yaw;
    float p = (float)t.pitch;
    return glm::normalize(glm::vec3(
         sinf(y)*cosf(p),   //  East component
         sinf(p),            //  Up component (pitch)
        -cosf(y)*cosf(p)    // -North component (North=-Z → forward = -cos)
    ));
}

// ── Câmera orbital (substitui chaseView) ─────────────────────────────────────
// yaw=0 → atrás do avião, yaw=π → nariz. pitch=0.26 rad (~15°) = padrão chase.
// Ângulos são PERSISTENTES: não voltam ao soltar L2+R2.
// forward é suavizado (alpha=0.80) para eliminar tremor em viradas.
static glm::mat4 orbitView(const Telemetry& t, float yaw, float pitch){
    static glm::vec3 sFwd{0.f, 0.f, -1.f};
    static bool sInit = false;
    glm::vec3 fwdTarget = aircraftForward(t);
    if (!sInit){ sFwd = fwdTarget; sInit = true; }
    sFwd = glm::normalize(glm::mix(sFwd, fwdTarget, 0.80f));

    glm::vec3 worldUp(0,1,0);
    glm::vec3 right = glm::normalize(glm::cross(sFwd, worldUp));
    glm::vec3 acUp  = glm::normalize(glm::cross(right, sFwd));

    constexpr float DIST = 30.f;
    glm::vec3 hDir  = -sFwd * std::cos(yaw) + right * std::sin(yaw);
    glm::vec3 camPos = hDir * std::cos(pitch) * DIST + acUp * std::sin(pitch) * DIST;
    glm::vec3 target = sFwd * 5.f;
    return glm::lookAt(camPos, target, worldUp);
}

// ── Câmera cockpit com movimento de cabeça ────────────────────────────────────
// eyeLocal: posição do olho do capitão no body-frame do modelo (GL: X=lat, Y=up, Z=fwd=-Z)
// headYaw/headPitch: desvio do olhar em relação ao nariz (stick direito no cockpit)
static glm::mat4 cockpitView(const Telemetry& t, float headYaw, float headPitch){
    // Atitude do avião → world frame
    glm::mat4 R(1.f);
    R = glm::rotate(R, -(float)t.yaw,   glm::vec3(0,1,0));
    R = glm::rotate(R,  (float)t.pitch, glm::vec3(1,0,0));
    R = glm::rotate(R, -(float)t.roll,  glm::vec3(0,0,1));

    // Olho do capitão no body-frame → world
    glm::vec3 eye   = glm::vec3(R * glm::vec4(0.35f, 2.3f, -9.5f, 1.f));
    glm::vec3 fwdW  = glm::vec3(R * glm::vec4(0.f, 0.f, -1.f, 0.f));
    glm::vec3 upW   = glm::vec3(R * glm::vec4(0.f, 1.f,  0.f, 0.f));
    glm::vec3 rightW = glm::normalize(glm::cross(fwdW, upW));

    // Aplica movimento de cabeça:
    // headYaw  → rotaciona fwdW em torno do upW (olhar esq/dir)
    // headPitch→ rotaciona em torno do rightW (olhar cima/baixo)
    glm::mat4 Ryaw   = glm::rotate(glm::mat4(1.f), headYaw,   upW);
    fwdW  = glm::normalize(glm::vec3(Ryaw * glm::vec4(fwdW,  0.f)));
    rightW= glm::normalize(glm::cross(fwdW, upW));
    glm::mat4 Rpitch = glm::rotate(glm::mat4(1.f), headPitch, rightW);
    fwdW  = glm::normalize(glm::vec3(Rpitch * glm::vec4(fwdW, 0.f)));

    return glm::lookAt(eye, eye + fwdW * 50.f, upW);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(){
    if(!glfwInit()){fprintf(stderr,"[GLFW]\n");return 1;}
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES,4);

    GLFWwindow* win=glfwCreateWindow(1280,720,
        "WebFlight C++ — Fase 2b (coord corretas, geo C172P)",nullptr,nullptr);
    if(!win){glfwTerminate();return 1;}
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);
    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)){fprintf(stderr,"[GLAD]\n");return 1;}
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_PROGRAM_POINT_SIZE);    // permite gl_PointSize no shader
    printf("[GL] %s\n",glGetString(GL_VERSION));

    // ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui::GetIO().IniFilename=nullptr;
    ImGui_ImplGlfw_InitForOpenGL(win,true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Condições iniciais (usadas também pelo TileManager)
    InitialConditions ic;
    const double ORIGIN_LAT = ic.latDeg;
    const double ORIGIN_LON = ic.lonDeg;

    // Módulos visuais
    Sky         sky; sky.init();

    // LOD 2 níveis:
    // 3 camadas LOD de terreno:
    //   ultraFarTiles: zoom  9, 9×9 tiles × ~39 km = ~156 km raio
    //   farTiles     : zoom 12, 17×17 × ~10 km     =  ~78 km raio
    //   closeTiles   : zoom 15, 9×9 × 1.2 km       =  ~10 km raio
    TileManager ultraFarTiles, farTiles, closeTiles, nearTiles;
    ultraFarTiles.init(ORIGIN_LAT, ORIGIN_LON,  7, 4, 17, -1.0f); // zoom 7: 9×9=81 tiles ~1250km raio
    farTiles .init(ORIGIN_LAT, ORIGIN_LON, 12, 8, 33, -0.2f);     // zoom 12: 17×17=289 tiles ~78km
    closeTiles.init(ORIGIN_LAT, ORIGIN_LON, 15, 8, 65,  0.f);     // zoom 15: 17×17 ±9.4km — referência de altitude
    nearTiles.init(ORIGIN_LAT, ORIGIN_LON, 17, 4, 33,  0.f);      // zoom 17: 9×9 ±1.3km — 1.2 m/px no solo

    Clouds clouds; clouds.init();

    AirportManager airports;
    airports.load(DATA_PATH "/airports.csv", DATA_PATH "/runways.csv",
                  ORIGIN_LAT, ORIGIN_LON);

    OSMManager osm;
    {
        const char* appdata = std::getenv("LOCALAPPDATA");
        std::string osmCache = (appdata ? std::string(appdata) : ".") + "/webflight/osm_cache";
        osm.init(ORIGIN_LAT, ORIGIN_LON, osmCache);
    }

    PostFX postfx; postfx.init();

    // Shader de pontos de contato (wireframe removido — modelo 3D substituiu)
    GLuint prog = makeProgram(COL_VERT, COL_FRAG);
    GLint uMVP     = glGetUniformLocation(prog,"uMVP");
    GLint uPtSize  = glGetUniformLocation(prog,"uPtSize");
    GLint uCircle  = glGetUniformLocation(prog,"uCircle");

    // Shader + modelo 3D E195
    GLuint modelProg = makeProgram(MODEL_VERT, MODEL_FRAG);
    GLuint animProg  = makeProgram(ANIM_VERT,  MODEL_FRAG);
    AcModel e195;
    {
        std::string objPath = std::string(MODELS_PATH) + "/erj195.obj";
        if (!e195.load(objPath))
            fprintf(stderr, "[main] E195 model failed to load from %s\n", objPath.c_str());
    }
    // Cockpit (câmera interna) — modelo estático sem partes animadas
    AcModel cockpit;
    {
        std::string objPath = std::string(MODELS_PATH) + "/cockpit.obj";
        if (!cockpit.load(objPath))
            fprintf(stderr, "[main] Cockpit model failed to load from %s\n", objPath.c_str());
        else
            printf("[main] Cockpit loaded.\n");
    }
    float gearAnimPos = 1.f;   // 1=extended, 0=retracted
    static float fanAngle = 0.f;

    // VAO de pontos de contato (dinâmico — atualizado todo frame)
    // Formato: xyz rgb (6 floats por ponto)
    static const int MAX_PTS = 16;
    GLuint ptVAO,ptVBO;
    glGenVertexArrays(1,&ptVAO); glGenBuffers(1,&ptVBO);
    glBindVertexArray(ptVAO);
      glBindBuffer(GL_ARRAY_BUFFER,ptVBO);
      glBufferData(GL_ARRAY_BUFFER,MAX_PTS*6*sizeof(float),nullptr,GL_DYNAMIC_DRAW);
      glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(void*)0);
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(void*)(3*sizeof(float)));
      glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);

    // FDM — E195 com FBW
    FDM             fdm;
    FlyByWire       fbw;
    GuidanceModule  gm;
    bool            guidancePanelOpen = false;
    MiniMap         minimap;
    bool            showMinimap       = true;   // tecla M
    float           viewDistScale     = 1.0f;   // 1=dia claro real, >1=ar mais limpo
    FlyByWire::SurfaceCmd surfCmd;

    // Condições iniciais para o E195 (voo de cruzeiro ~3500ft, 200 kt)
    InitialConditions icE195;
    icE195.latDeg      = ic.latDeg;
    icE195.lonDeg      = ic.lonDeg;
    icE195.altFt       = 3500.0;
    icE195.headingDeg  = 97.0;
    icE195.airspeedKts = 200.0;

    bool fdmOk = fdm.initE195(AIRCRAFT_PATH, ENGINE_PATH, icE195);
    if(!fdmOk) fprintf(stderr,"[Main] FDM offline\n");

    Axes      axes;
    Telemetry tel;
    WorldPos  wpos;
    float     terrainElev_m = 0.f;
    double lastTime  = glfwGetTime();
    double simScale  = 1.0;
    float  localHour = 10.5f;
    bool   timeAuto  = false;
    bool   f2prev    = false;

    // Última saída do GuidanceModule (acessível no HUD fora do bloco de física)
    GuidanceModule::Output gmOut;

    // ── Pausa / Reposicionamento ────────────────────────────────────────────────
    bool paused = false;
    bool pPrev  = false;
    FDM::Reposition repoParams;   // editáveis na UI quando pausado
    float pauseTempC = 15.f;      // temperatura a aplicar (°C) na altitude de repoParams

    printf("[Main] E195 FBW pronto.\n"
           "  ↑↓ profundor | ←→ aileron | A/D leme | W/S potência | B freio\n"
           "  G=trem | F=flaps recolhe | V=flaps desce | T/Y hora | P=pausa | M=minimapa | ESC sair\n");

    while(!glfwWindowShouldClose(win)){
        glfwPollEvents();
        if(glfwGetKey(win,GLFW_KEY_ESCAPE)==GLFW_PRESS)
            glfwSetWindowShouldClose(win,true);

        double now=glfwGetTime();
        double dt=std::min(now-lastTime,0.10); lastTime=now;

        // Gear animation (8 s transit)
        {
            float gearTarget = gGearDown ? 1.f : 0.f;
            constexpr float GEAR_SPEED = 1.f/8.f;
            if (gearAnimPos < gearTarget) gearAnimPos = std::min(gearTarget, gearAnimPos + GEAR_SPEED*(float)dt);
            else                          gearAnimPos = std::max(gearTarget, gearAnimPos - GEAR_SPEED*(float)dt);
        }

        // Hora / sol
        if(glfwGetKey(win,GLFW_KEY_T)==GLFW_PRESS) localHour=fmodf(localHour+0.02f,24.f);
        if(glfwGetKey(win,GLFW_KEY_Y)==GLFW_PRESS) localHour=fmodf(localHour-0.02f+24.f,24.f);
        bool f2now=glfwGetKey(win,GLFW_KEY_F2)==GLFW_PRESS;
        if(f2now&&!f2prev) timeAuto=!timeAuto; f2prev=f2now;
        if(timeAuto) localHour=fmodf(localHour+(float)dt/60.f,24.f);

        glm::vec3 sunDir = Sky::sunDirection(localHour);
        float     day    = Sky::dayFactor(sunDir);

        // ── Toggle de motor individual (1=ENG1, 2=ENG2) ──────────────────────
        if (fdmOk) {
            static bool e1Prev=false, e2Prev=false;
            bool e1Now = glfwGetKey(win, GLFW_KEY_1) == GLFW_PRESS;
            bool e2Now = glfwGetKey(win, GLFW_KEY_2) == GLFW_PRESS;
            if (e1Now && !e1Prev) fdm.toggleEngineCutoff(0);
            if (e2Now && !e2Prev) fdm.toggleEngineCutoff(1);
            e1Prev = e1Now; e2Prev = e2Now;
        }

        // ── Reversão de lei FBW (L: NORMAL ↔ DIRECT) ─────────────────────────
        {
            static bool lPrev = false;
            bool lNow = glfwGetKey(win, GLFW_KEY_L) == GLFW_PRESS;
            if (lNow && !lPrev) {
                using Law = FlyByWire::Law;
                fbw.law = (fbw.law == Law::Normal) ? Law::Direct : Law::Normal;
                printf("[FBW] lei ativa: %s\n", fbw.lawName());
            }
            lPrev = lNow;
        }

        // ── Guidance (Z/Select = ATT hold | H/Circle = ALT hold | F1 = painel) ─
        {
            static bool zPrev = false, hPrev = false, f1Prev = false;

            // F1 — toggle painel AFCS (sempre disponível)
            bool f1Now = glfwGetKey(win, GLFW_KEY_F1) == GLFW_PRESS;
            if (f1Now && !f1Prev) guidancePanelOpen = !guidancePanelOpen;
            f1Prev = f1Now;

            if (fdmOk && !paused) {
                // Z / Select — toggle Attitude Hold
                bool zNow = glfwGetKey(win, GLFW_KEY_Z) == GLFW_PRESS || axes.apBtn;
                if (zNow && !zPrev) {
                    if (gm.mode.vert == GuidanceModule::VertMode::AttitudeHold) {
                        gm.disengageVert();
                    } else {
                        FlyByWire::AircraftState acStAp = fdm.getStateForFBW();
                        gm.engageAttitude(acStAp, fbw);
                    }
                }
                zPrev = zNow;

                // H / Circle — toggle Altitude Hold
                bool hNow = glfwGetKey(win, GLFW_KEY_H) == GLFW_PRESS || axes.altBtn;
                if (hNow && !hPrev) {
                    if (gm.mode.vert == GuidanceModule::VertMode::AltitudeHold) {
                        gm.disengageVert();
                    } else {
                        FlyByWire::AircraftState acStAp = fdm.getStateForFBW();
                        gm.engageAltitude(acStAp, fbw);
                    }
                }
                hPrev = hNow;
            }
        }

        // ── Minimapa HSD (M liga/desliga) ─────────────────────────────────────
        {
            static bool mPrev = false;
            bool mNow = glfwGetKey(win, GLFW_KEY_M) == GLFW_PRESS;
            if (mNow && !mPrev) showMinimap = !showMinimap;
            mPrev = mNow;
        }

        // ── Toggle de pausa (P) ────────────────────────────────────────────────
        {
            bool pNow = glfwGetKey(win, GLFW_KEY_P) == GLFW_PRESS || axes.pauseBtn;
            if (pNow && !pPrev && fdmOk) {
                if (!paused) {
                    fdm.pause();
                    // Semeia os parâmetros de repositionamento com o estado atual
                    repoParams.latDeg     = fdm.getLatDeg();
                    repoParams.lonDeg     = fdm.getLonDeg();
                    repoParams.altFt      = fdm.getAltFt();
                    repoParams.headingDeg = fdm.getHdgDeg();
                    repoParams.speedKcas  = (float)tel.cas;
                    repoParams.pitchDeg   = (float)(tel.pitch * RAD2DEG);
                    repoParams.rollDeg    = (float)(tel.roll  * RAD2DEG);
                    pauseTempC = fdm.getCurrentTempC();
                    minimap.centerPickerOn(repoParams.latDeg, repoParams.lonDeg);
                    paused = true;
                } else {
                    fdm.resume();
                    fbw.reset();
                    paused = false;
                }
            }
            pPrev = pNow;
        }

        // Input + física
        updateAxes(axes,win,(float)dt);

        // Posição world atual do avião (float, para TileManager)
        glm::vec3 acWorld((float)wpos.x,(float)wpos.y,(float)wpos.z);

        // Atualiza tiles e nuvens
        ultraFarTiles.update(acWorld, ORIGIN_LAT, ORIGIN_LON);
        farTiles .update(acWorld, ORIGIN_LAT, ORIGIN_LON);
        closeTiles.update(acWorld, ORIGIN_LAT, ORIGIN_LON);
        nearTiles.update(acWorld, ORIGIN_LAT, ORIGIN_LON);
        clouds.update(acWorld);

        // Elevação real: preferir zoom 15 (mais preciso), fallback zoom 13
        terrainElev_m = closeTiles.getElevAt(acWorld);
        if(terrainElev_m == 0.f) terrainElev_m = farTiles.getElevAt(acWorld);
        // Altitude MSL do avião = terreno + AGL
        float acMslM = terrainElev_m + (float)wpos.y;

        // Aeroportos (precisa acMslM para PAPI)
        airports.update(acWorld, acMslM, closeTiles, farTiles, nearTiles);
        // osm.update(fdm.getLatDeg(), fdm.getLonDeg(), closeTiles, farTiles); // pesado: ~40k draw calls — precisa de batching antes de reativar

        if(fdmOk && !paused){
            // 1. Monta input do piloto
            FlyByWire::PilotInput inp;
            inp.column = std::clamp(axes.elv + axes.trimElev, -1.f, 1.f);
            inp.wheel  = std::clamp(axes.ail + axes.trimAil,  -1.f, 1.f);
            inp.pedals      =  axes.rdr;
            inp.throttle[0] = inp.throttle[1] = axes.thr;
            inp.flaps       = axes.flaps;
            inp.brake       = axes.brk;
            inp.gearCmd     = axes.gear;
            inp.reverser    = axes.reverser;

            // 2. Lê estado atual do JSBSim → FBW
            FlyByWire::AircraftState acSt = fdm.getStateForFBW();

            // Reversor não arma em voo: limpa o toggle se sem peso nas rodas
            if (!acSt.wow) axes.reverser = false;

            // 2.5. GuidanceModule: modifica inp antes do FBW
            gm.update((float)dt, acSt, inp, fbw, gmOut);
            if (gmOut.overrideThrottle) {
                inp.throttle[0] = gmOut.throttle[0];
                inp.throttle[1] = gmOut.throttle[1];
            }

            // 3. Roda as leis FBW → comandos de superfície
            fbw.update((float)dt, inp, acSt, surfCmd);

            // 4. Escreve comandos no JSBSim E195
            fdm.setControlsE195(surfCmd);
            fdm.setTerrainElevation((double)terrainElev_m / FT2M);

            // 5. Avança simulação
            int steps = std::max(1, (int)std::round(simScale*dt*JSB_HZ));
            for(int i=0;i<steps;i++) fdm.step();

            tel = fdm.getTelemetry();
            integratePos(wpos, tel, dt);   // y = AGL
            // X/Z direto da lat/lon do JSBSim via projeção única — sem drift
            // de integração e sempre alinhado à imagem de satélite
            geo::toWorld(fdm.getLatDeg(), fdm.getLonDeg(),
                         ORIGIN_LAT, ORIGIN_LON, wpos.x, wpos.z);
        }

        // Pontos de contato (gear + wing tips)
        auto contacts = fdmOk ? fdm.getContactPoints() : std::vector<FDM::GearPoint>{};

        // Upload dos pontos ao VBO dinâmico
        {
            std::vector<float> ptData;
            for(const auto& gp : contacts){
                ptData.push_back(gp.rx); ptData.push_back(gp.ry); ptData.push_back(gp.rz);
                // Cor: gear-WOW=vermelho, gear-air=ciano, LEFT_TIP=verde, RIGHT_TIP=vermelho
                std::string nm(gp.name);
                if(nm=="LEFT_TIP")  { ptData.push_back(0.f); ptData.push_back(1.f); ptData.push_back(.2f); }
                else if(nm=="RIGHT_TIP") { ptData.push_back(1.f); ptData.push_back(.1f); ptData.push_back(.1f); }
                else if(gp.wow)     { ptData.push_back(1.f); ptData.push_back(.4f); ptData.push_back(0.f); } // laranja
                else                { ptData.push_back(.2f); ptData.push_back(1.f); ptData.push_back(.9f); } // ciano
            }
            glBindBuffer(GL_ARRAY_BUFFER,ptVBO);
            glBufferSubData(GL_ARRAY_BUFFER,0,(GLsizeiptr)(ptData.size()*sizeof(float)),ptData.data());
        }

        // ── Render ─────────────────────────────────────────────────────────────
        int fw,fh; glfwGetFramebufferSize(win,&fw,&fh);
        glViewport(0,0,fw,fh);
        float aspect = fw>0&&fh>0 ? (float)fw/(float)fh : 16.f/9.f;

        // ── Seleção de câmera ──────────────────────────────────────────────────
        glm::mat4 view;
        float fovDeg   = 70.f;
        float nearClip = 0.5f;
        if (axes.cockpit) {
            // Visão interna: FOV mais amplo, clip próximo para ver instrumentos
            view     = cockpitView(tel, axes.headYaw, axes.headPitch);
            fovDeg   = 80.f;
            nearClip = 0.05f;
        } else {
            // Visão externa: órbita ao redor do avião (ângulos persistentes)
            // camYaw=0, camPitch=0.26 → comportamento idêntico ao chase original
            view = orbitView(tel, axes.camYaw, axes.camPitch);
        }
        glm::mat4 proj = glm::perspective(glm::radians(fovDeg), aspect, nearClip, 2000000.f);

        // Redireciona render para FBO HDR (bloom será aplicado depois)
        postfx.bind(fw, fh);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);

        // 1. Sky
        sky.render(view,proj,sunDir,day);

        // 2. Terrain tiles reais (ultraFar → far → close → near, distante → próximo)
        ultraFarTiles.visScale = viewDistScale;
        farTiles .visScale     = viewDistScale;
        closeTiles.visScale    = viewDistScale;
        nearTiles.visScale     = viewDistScale;

        // Camada FINA primeiro, com depth write + stencil por camada: cada camada
        // grava seu "peso" no stencil onde desenhou; camadas mais grossas (peso
        // menor, GEQUAL falha) não sobrescrevem esses pixels. Assim near/close/far
        // têm oclusão real (montanhas escondem o outro lado) sem z-fighting entre
        // LODs e sem polygon offset (offsets grandes causavam a "barreira reta").
        // ultraFar (z7) continua pintor sem depth — só fundo de horizonte.
        ultraFarTiles.depthWrite = false;
        farTiles.depthWrite      = true;
        closeTiles.depthWrite    = true;
        nearTiles.depthWrite     = true;
        glEnable(GL_STENCIL_TEST);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilFunc(GL_GEQUAL, 4, 0xFF);
        nearTiles.render(proj*view, acWorld, acMslM, sunDir, day);
        glStencilFunc(GL_GEQUAL, 3, 0xFF);
        closeTiles.render(proj*view, acWorld, acMslM, sunDir, day);
        glStencilFunc(GL_GEQUAL, 2, 0xFF);
        farTiles .render(proj*view, acWorld, acMslM, sunDir, day);
        glStencilFunc(GL_GEQUAL, 1, 0xFF);
        ultraFarTiles.render(proj*view, acWorld, acMslM, sunDir, day);
        glDisable(GL_STENCIL_TEST);

        // 3. Aeroportos (pistas opacas renderizadas antes das nuvens)
        airports.render(proj*view, acWorld, acMslM, day);

        // 3b. OSM: prédios, estradas, água
        // osm.render(proj*view, acWorld, day, (float)glfwGetTime()); // pesado: ~40k draw calls — precisa de batching antes de reativar

        // 4. Nuvens — depois do terreno, antes do avião (opaco ganha depth test)
        clouds.render(view, proj, acWorld, acMslM, sunDir, day);

        // 4. Modelo 3D — exterior (E195) ou cockpit (visão interna)
        // Ordem YXZ: yaw → pitch → roll
        glm::mat4 acRot(1.f);
        acRot = glm::rotate(acRot, -(float)tel.yaw,   glm::vec3(0,1,0));
        acRot = glm::rotate(acRot,  (float)tel.pitch, glm::vec3(1,0,0));
        acRot = glm::rotate(acRot, -(float)tel.roll,  glm::vec3(0,0,1));
        glm::mat4 mvp = proj * view * acRot;

        // Fan speed: N1 0-100 → ~20 rad/s at full power (visual only)
        fanAngle += (float)(tel.n1[0] / 100.0 * 20.0 * dt);

        if (axes.cockpit) {
            // Visão interna: renderiza cockpit, omite exterior
            if (cockpit.ok()) {
                AcAnimState noAnim;
                cockpit.draw(modelProg, animProg, mvp, noAnim);
            }
        } else {
            // Visão externa: renderiza fuselagem + superfícies animadas
            if (e195.ok()) {
                AcAnimState animState;
                animState.aileronL = surfCmd.aileronL;
                animState.aileronR = surfCmd.aileronR;
                animState.elevL    = surfCmd.elevLH;
                animState.elevR    = surfCmd.elevRH;
                animState.rudder   = surfCmd.rudder;
                animState.flaps    = axes.flaps;
                animState.spoilerL      = surfCmd.spoilerL;
                animState.spoilerR      = surfCmd.spoilerR;
                animState.groundSpoiler = surfCmd.groundSpoiler;
                animState.gearPos  = gearAnimPos;
                animState.fanAngle = fanAngle;
                e195.draw(modelProg, animProg, mvp, animState);
            } else {
                glUseProgram(prog);
                glUniformMatrix4fv(uMVP,1,GL_FALSE,glm::value_ptr(mvp));
                glUniform1f(uPtSize,1.f);
                glUniform1i(uCircle,0);
            }
        }

        // 4. Pontos de contato (esferas grandes)
        if(!contacts.empty()){
            glUseProgram(prog);
            glUniformMatrix4fv(uMVP,1,GL_FALSE,glm::value_ptr(mvp));
            glUniform1f(uPtSize,12.f);
            glUniform1i(uCircle,1);
            glBindVertexArray(ptVAO);
            glDrawArrays(GL_POINTS,0,(int)contacts.size());
            glBindVertexArray(0);
        }

        // Bloom + tone-map → framebuffer default (antes do ImGui, que fica no screen)
        {
            float bloomStr = 0.08f + (1.f - day) * 0.55f;  // 0.08 dia → 0.63 noite
            float exposure = 0.68f + (1.f - day) * 0.72f;  // 0.68 dia → 1.40 noite
            float thresh   = 0.85f - (1.f - day) * 0.12f;  // 0.85 dia → 0.73 noite
            postfx.resolve(bloomStr, exposure, thresh);
        }

        // 5. HUD (renderiza sobre o frame final no default framebuffer)
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        minimap.processUploads();   // sobe tiles OSM baixados p/ GPU (thread principal)

        ImGui::SetNextWindowPos({8,8},ImGuiCond_Always);
        ImGui::SetNextWindowSize({260,560},ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(.75f);
        ImGui::Begin("##hud",nullptr,
            ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
            ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoSavedSettings);

        ImGui::TextColored({.3f,1.f,.4f,1.f},"WEBFLIGHT C++  E195-E2");
        ImGui::TextDisabled("FBW Normal Law — SBGL Rio");
        ImGui::Separator();

        // ── Velocidades
        ImGui::Text("CAS  %5.0f kt   TAS %5.0f kt", tel.cas, tel.airspeed);
        ImGui::Text("MACH %5.3f", tel.mach);

        // ── Altimetria
        ImGui::Text("ALT  %5.0f ft AGL / %5.0f MSL", tel.altAgl, tel.altBaro);
        ImGui::Text("VS   %+6.0f fpm", tel.vsFpm);
        ImGui::Text("TERR %+5.0f m MSL", terrainElev_m);

        ImGui::Separator();

        // ── Atitude
        ImGui::Text("HDG  %5.0f°", fmod(tel.yaw*RAD2DEG+360.0,360.0));
        ImGui::Text("PCH  %+5.1f°   ROL %+5.1f°",
                    tel.pitch*RAD2DEG, tel.roll*RAD2DEG);

        // ── AFCS status
        {
            auto vm = gm.mode.vert;
            auto lm = gm.mode.lat;
            auto tm = gm.mode.thr;
            if (vm == GuidanceModule::VertMode::AltitudeHold) {
                if (gm.targets.vsManual)
                    ImGui::TextColored({.2f,.8f,1.f,1.f},
                        "AP ALT  %5.0f ft  V/S%+.0f",
                        gm.targets.altFt, gm.targets.vsFpm);
                else
                    ImGui::TextColored({.2f,.8f,1.f,1.f},
                        "AP ALT  %5.0f ft  PCH%+.1f°",
                        gm.targets.altFt, gm.targets.pitchDeg);
            } else if (vm == GuidanceModule::VertMode::AttitudeHold) {
                ImGui::TextColored({.2f,1.f,.4f,1.f},
                    "AP ATT  PCH%+.1f°  ROL%+.1f°",
                    gm.targets.pitchDeg, gm.targets.bankDeg);
            } else if (vm == GuidanceModule::VertMode::Flch) {
                ImGui::TextColored({1.f,.6f,.9f,1.f},
                    "FLCH %s %5.0f ft @ %3.0f kt",
                    (gm.targets.altFt > tel.altBaro) ? "CLB" : "DES",
                    gm.targets.altFt, gm.targets.speedKt);
            } else {
                ImGui::TextColored({.5f,.5f,.5f,1.f},
                    "AP OFF   Z=ATT  H=ALT  F1=AFCS");
            }
            if (lm == GuidanceModule::LatMode::HeadingHold)
                ImGui::TextColored({.9f,.7f,.2f,1.f},
                    "HDG SEL  %03.0f°", gm.targets.headingDeg);
            if (lm == GuidanceModule::LatMode::Nav) {
                if (gm.activeWpt < (int)gm.fplan.size())
                    ImGui::TextColored({.4f,.9f,1.f,1.f},
                        "LNAV %s  BRG %03.0f°  %.1f NM",
                        gm.fplan[gm.activeWpt].name.c_str(),
                        gm.navBrgDeg, gm.navDistNm);
                else
                    ImGui::TextColored({.4f,.9f,1.f,1.f}, "LNAV  FIM DO PLANO");
            }
            if (tm == GuidanceModule::ThrMode::SpeedHold)
                ImGui::TextColored({.9f,.9f,.2f,1.f},
                    "A/THR  VMIN %3.0f kt", gm.targets.speedKt);
        }

        ImGui::Separator();

        // ── Motores CF34
        {
            auto engColor = [&](int n) -> ImVec4 {
                return fdm.engineCutoff(n) ? ImVec4{1.f,.2f,.2f,1.f}
                                           : ImVec4{.3f,1.f,.4f,1.f};
            };
            ImGui::TextColored(engColor(0), "ENG1 N1%4.1f%% %s",
                               tel.n1[0], fdm.engineCutoff(0)?"[CUT]":"[ON] ");
            ImGui::TextColored(engColor(1), "ENG2 N1%4.1f%% %s",
                               tel.n1[1], fdm.engineCutoff(1)?"[CUT]":"[ON] ");
            ImGui::Text(       "N2   L%4.1f%%  R%4.1f%%", tel.n2[0], tel.n2[1]);
            if (gmOut.overrideThrottle)
                ImGui::TextColored({.9f,.9f,.2f,1.f},
                    "THR  cmd%3.0f%%  lev%3.0f%%",
                    gmOut.throttle[0]*100.f, axes.thr*100.f);
            else
                ImGui::Text(   "THR  %3.0f%%", axes.thr*100.f);
            if (surfCmd.reverser)
                ImGui::TextColored({1.f,.55f,.1f,1.f}, "REV  DEPLOYED");
            else if (axes.reverser)
                ImGui::TextColored({.8f,.8f,.3f,1.f}, "REV  ARMED");
        }

        ImGui::Separator();

        // ── FBW
        float alphaDeg = (float)(tel.alphaRad * RAD2DEG);
        ImGui::Text("Nz   %+5.2f g", tel.loadNz);
        ImGui::Text("AOA  %+5.1f°", alphaDeg);

        // Lei ativa (L = alternar NORMAL ↔ DIRECT)
        if (fbw.law == FlyByWire::Law::Normal)
            ImGui::TextColored({.3f,1.f,.3f,1.f}, "FBW NORMAL");
        else
            ImGui::TextColored({1.f,.3f,.3f,1.f}, "FBW DIRECT");

        // Status proteções (apenas Normal Law)
        if(fbw.alphaFloorActive())
            ImGui::TextColored({1.f,.3f,.3f,1.f}, "!! ALPHA FLOOR");
        else if(fbw.bankProtActive())
            ImGui::TextColored({1.f,.8f,.2f,1.f}, "!! BANK PROT");

        ImGui::Text("C*  dem%+5.2f act%+5.2f", fbw.cstarDemand(), fbw.cstarActual());
        ImGui::Text("Elev integ %+5.3f", fbw.elevInteg());

        ImGui::Separator();

        // ── Configuração
        ImGui::Text("Flaps %3.0f%%  Gear %s",
                    gFlapsStep*100.f, gGearDown?"DOWN":"UP");
        ImGui::Separator();

        // ── Posição
        ImGui::Text("X%+8.0fm  Z%+8.0fm", wpos.x, wpos.z);
        ImGui::Text("LAT%.5f  LON%.5f",
                    fdmOk ? fdm.getLatDeg() : 0.0,
                    fdmOk ? fdm.getLonDeg() : 0.0);

        ImGui::Separator();
        int hh=(int)localHour, mm=(int)((localHour-hh)*60.f);
        ImGui::Text("Hora %02d:%02d  sol %+.0f°",hh,mm,
                    glm::degrees(asinf(sunDir.y)));
        ImGui::TextColored(fdmOk?ImVec4{.3f,1.f,.3f,1.f}:ImVec4{1.f,.4f,.4f,1.f},
            fdmOk?"FDM: E195 OK":"FDM: offline");
        if (paused)
            ImGui::TextColored({1.f,.4f,.1f,1.f}, "PAUSADO  --  P = retomar");
        else
            ImGui::TextDisabled("↓↑=arfagem | ←→=aileron | A/D=leme | W/S=thr | B=freio");
        ImGui::TextDisabled("G=trem | F/V=flaps | NP8/5=trim↑↓ | NP4/6=trimRoll");
        ImGui::End();

        // ── HUD direito — Superfícies de controle ─────────────────────────────
        {
            // Helper: barra colorida para valor [-1, +1]
            // Metade esquerda = negativo (vermelho), metade direita = positivo (azul)
            auto surfBar = [&](const char* label, float val){
                float norm = (val + 1.f) * 0.5f;   // [-1,+1] → [0,1]
                // Cor: negativo=vermelho/laranja, zero=cinza, positivo=ciano/azul
                ImVec4 col = (val > 0.01f)  ? ImVec4{.2f,.7f,1.f,1.f}
                           : (val < -0.01f) ? ImVec4{1.f,.4f,.2f,1.f}
                                            : ImVec4{.5f,.5f,.5f,1.f};
                char buf[16]; snprintf(buf,16,"%+.2f",val);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, col);
                ImGui::ProgressBar(norm, {110.f,12.f}, buf);
                ImGui::PopStyleColor();
                ImGui::SameLine(0,6);
                ImGui::Text("%s", label);
            };
            auto surf01 = [&](const char* label, float val){
                char buf[16]; snprintf(buf,16,"%.2f",val);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4{.9f,.8f,.1f,1.f});
                ImGui::ProgressBar(val, {110.f,12.f}, buf);
                ImGui::PopStyleColor();
                ImGui::SameLine(0,6);
                ImGui::Text("%s", label);
            };

            int hudW = 220;
            ImGui::SetNextWindowPos({(float)(fw - hudW - 8), 8.f}, ImGuiCond_Always);
            ImGui::SetNextWindowSize({(float)hudW, 440.f}, ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(.75f);
            ImGui::Begin("##surf", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoSavedSettings);

            ImGui::TextColored({.3f,1.f,.8f,1.f}, "SUPERFICIES");
            ImGui::Separator();

            surfBar("Elev LH",  surfCmd.elevLH);
            surfBar("Ailer L",  surfCmd.aileronL);
            surfBar("Ailer R",  surfCmd.aileronR);
            surfBar("Rudder",   surfCmd.rudder);
            // Slip-skid indicator — barra ±10°, texto em graus reais
            {
                float bN = std::clamp((float)tel.betaDeg / 10.f, -1.f, 1.f);
                float norm = (bN + 1.f) * 0.5f;
                ImVec4 col = (bN >  0.01f) ? ImVec4{.2f,.7f,1.f,1.f}
                           : (bN < -0.01f) ? ImVec4{1.f,.4f,.2f,1.f}
                                           : ImVec4{.5f,.5f,.5f,1.f};
                char buf[16]; snprintf(buf,16,"%+.1f\xc2\xb0",(float)tel.betaDeg);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, col);
                ImGui::ProgressBar(norm, {110.f,12.f}, buf);
                ImGui::PopStyleColor();
                ImGui::SameLine(0,6);
                ImGui::Text("Beta");
            }
            ImGui::Separator();
            surf01 ("Flap",     surfCmd.flaps / 0.75f);
            surf01 ("SpdBrk",   surfCmd.groundSpoiler);
            surf01 ("SpoilL",   surfCmd.spoilerL);
            surf01 ("SpoilR",   surfCmd.spoilerR);
            ImGui::Separator();

            ImGui::TextColored({.9f,.9f,.4f,1.f}, "TRIM");
            surfBar("Pitch",    axes.trimElev);
            surfBar("Roll",     axes.trimAil);
            ImGui::Separator();

            ImGui::TextColored({.9f,.9f,.4f,1.f}, "ENTRADA PILOTO");
            surfBar("Column",   axes.elv);
            surfBar("Wheel",    axes.ail);
            surfBar("Pedals",   axes.rdr);
            surf01 ("Thr(lev)", axes.thr);
            surf01 ("Thr(cmd)", (float)tel.throttle);

            ImGui::End();
        }

        // ── Minimapa HSD (canto inferior esquerdo, heading-up) ────────────────
        if (fdmOk && showMinimap && !paused) {
            const float mmSz = 250.f;
            ImGui::SetNextWindowPos({8.f, (float)fh - mmSz - 8.f}, ImGuiCond_Always);
            ImGui::SetNextWindowSize({mmSz, mmSz}, ImGuiCond_Always);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0.f, 0.f});
            ImGui::Begin("##hsd", nullptr,
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_NoMove     | ImGuiWindowFlags_NoSavedSettings |
                ImGuiWindowFlags_NoScrollbar| ImGuiWindowFlags_NoScrollWithMouse |
                ImGuiWindowFlags_NoBackground);
            minimap.drawHSD({mmSz, mmSz}, fdm.getLatDeg(), fdm.getLonDeg(),
                            (float)(tel.yaw * RAD2DEG));
            ImGui::End();
            ImGui::PopStyleVar();
        }

        // ── Painel de Reposicionamento (visivel apenas quando pausado) ──────────
        if (paused && fdmOk) {
            ImGui::SetNextWindowPos({(float)(fw/2 - 410), (float)(fh/2 - 235)},
                                    ImGuiCond_Always);
            ImGui::SetNextWindowSize({820, 470}, ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(.92f);
            ImGui::Begin("##reposition", nullptr,
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_NoMove     | ImGuiWindowFlags_NoSavedSettings);

            ImGui::TextColored({1.f, .55f, .1f, 1.f}, "SIMULACAO PAUSADA");
            ImGui::TextColored({1.f, .55f, .1f, 1.f}, "Reposicionamento");
            ImGui::Separator();

            // coluna esquerda: campos e botões
            ImGui::BeginChild("##repoL", {400.f, 0.f}, false);

            ImGui::Text("Latitude  (graus decimais)");
            ImGui::SetNextItemWidth(390.f);
            ImGui::InputDouble("##lat", &repoParams.latDeg, 0.0, 0.0, "%.6f");

            ImGui::Text("Longitude (graus decimais)");
            ImGui::SetNextItemWidth(390.f);
            ImGui::InputDouble("##lon", &repoParams.lonDeg, 0.0, 0.0, "%.6f");

            ImGui::Text("Altitude (ft MSL)");
            ImGui::SetNextItemWidth(185.f);
            ImGui::InputDouble("##alt", &repoParams.altFt, 0.0, 0.0, "%.0f");

            ImGui::SameLine(0.f, 20.f);

            ImGui::Text("Heading (graus)");
            ImGui::SetNextItemWidth(185.f);
            ImGui::InputDouble("##hdg", &repoParams.headingDeg, 0.0, 0.0, "%.1f");

            ImGui::Text("CAS (kt)");
            ImGui::SetNextItemWidth(120.f);
            ImGui::InputFloat("##cas", &repoParams.speedKcas, 0.f, 0.f, "%.0f");

            ImGui::SameLine(0.f, 16.f);
            ImGui::Text("Pitch (graus)");
            ImGui::SetNextItemWidth(110.f);
            ImGui::InputFloat("##pch", &repoParams.pitchDeg, 0.f, 0.f, "%.1f");

            ImGui::SameLine(0.f, 16.f);
            ImGui::Text("Roll (graus)");
            ImGui::SetNextItemWidth(110.f);
            ImGui::InputFloat("##rol", &repoParams.rollDeg,  0.f, 0.f, "%.1f");

            // ── Temperatura ───────────────────────────────────────────────────────
            ImGui::Separator();
            {
                double altRef = repoParams.altFt;
                double isaC   = altRef < 36089.0 ? 15.0 - 0.0019812 * altRef : -56.5;
                ImGui::Text("Temperatura @ %.0f ft", altRef);
                ImGui::SameLine(0.f, 8.f);
                ImGui::TextDisabled("(ISA = %.1f°C)", (float)isaC);
                ImGui::SetNextItemWidth(100.f);
                ImGui::InputFloat("##temp", &pauseTempC, 0.f, 0.f, "%.1f");
                ImGui::SameLine(0.f, 10.f);
                float dev = pauseTempC - (float)isaC;
                ImGui::TextColored(dev >= 0.f
                    ? ImVec4{1.f, .70f, .25f, 1.f}
                    : ImVec4{.4f, .85f, 1.0f, 1.f},
                    "ISA%+.1f°C", dev);
                ImGui::SameLine(0.f, 16.f);
                if (ImGui::Button("Aplicar"))
                    fdm.setTemperatureAt(pauseTempC, altRef);
            }

            ImGui::Separator();

            ImGui::PushStyleColor(ImGuiCol_Button,        {.9f, .4f, .0f, 1.f});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, {1.f, .55f,.1f, 1.f});
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  {1.f, .7f, .2f, 1.f});
            bool doTeleport = ImGui::Button("  TELEPORTAR  ", {187.f, 36.f});
            ImGui::PopStyleColor(3);

            ImGui::SameLine(0.f, 16.f);

            ImGui::PushStyleColor(ImGuiCol_Button,        {.1f, .6f, .2f, 1.f});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, {.2f, .8f, .3f, 1.f});
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  {.3f, 1.f, .4f, 1.f});
            bool doResume = ImGui::Button("  RETOMAR (P)  ", {187.f, 36.f});
            ImGui::PopStyleColor(3);

            if (doTeleport) {
                while (repoParams.headingDeg <   0.0) repoParams.headingDeg += 360.0;
                while (repoParams.headingDeg >= 360.0) repoParams.headingDeg -= 360.0;

                fdm.repositionInPlace(repoParams);
                fdm.setTemperatureAt(pauseTempC, repoParams.altFt);
                fbw.reset();
                // Posicao render a partir da lat/lon nova (projeção única)
                geo::toWorld(repoParams.latDeg, repoParams.lonDeg,
                             ORIGIN_LAT, ORIGIN_LON, wpos.x, wpos.z);
                wpos.y = repoParams.altFt * FT2M;
                tel = fdm.getTelemetry();
            }

            if (doResume) {
                fdm.resume();
                fbw.reset();
                paused = false;
            }

            ImGui::Separator();
            ImGui::TextDisabled("Edite os campos ou clique no mapa, depois");
            ImGui::TextDisabled("TELEPORTAR. Pressione P para continuar.");
            ImGui::EndChild();

            // coluna direita: mapa de seleção (clique define lat/lon)
            ImGui::SameLine();
            ImGui::BeginChild("##repoR", {0.f, 0.f}, false);
            {
                // aeroportos ao redor do centro atual do mapa
                static std::vector<MiniMap::Poi> mapPois;
                {
                    double cLat, cLon; minimap.pickerCenter(cLat, cLon);
                    double cwx, cwz;
                    geo::toWorld(cLat, cLon, ORIGIN_LAT, ORIGIN_LON, cwx, cwz);
                    static std::vector<AirportManager::ApMarker> pk;
                    airports.getNearby({(float)cwx, 0.f, (float)cwz}, 150000.f, pk);
                    mapPois.clear();
                    for (const auto& m : pk) {
                        MiniMap::Poi p; p.label = m.ident;
                        geo::toLatLon(m.wx, m.wz, ORIGIN_LAT, ORIGIN_LON, p.lat, p.lon);
                        mapPois.push_back(std::move(p));
                    }
                }
                ImVec2 avail = ImGui::GetContentRegionAvail();
                minimap.drawPicker({avail.x, avail.y - 26.f},
                                   repoParams.latDeg, repoParams.lonDeg,
                                   fdm.getLatDeg(), fdm.getLonDeg(),
                                   (float)fdm.getHdgDeg(), &mapPois);
                if (ImGui::SmallButton("Centrar no aviao"))
                    minimap.centerPickerOn(fdm.getLatDeg(), fdm.getLonDeg());
                ImGui::SameLine();
                ImGui::TextDisabled("clique = destino | arraste = pan | scroll = zoom");
            }
            ImGui::EndChild();
            ImGui::End();
        }

        // ── Marcadores de aeroportos (visíveis de longe) ──────────────────────
        if (fdmOk) {
            static std::vector<AirportManager::ApMarker> apMk;
            airports.getNearby(acWorld, 250000.f, apMk);   // até 250 km

            ImGuiIO&    mio = ImGui::GetIO();
            ImDrawList* dl  = ImGui::GetBackgroundDrawList();
            glm::mat4   VP  = proj * view;

            for (const auto& m : apMk) {
                if (m.distM < 4000.f) continue;   // perto: a pista já é visível
                // Posição render (avião na origem) com a mesma curvatura da Terra
                float dy = (m.distM * m.distM) / (2.f * 6371000.f);
                glm::vec4 clip = VP * glm::vec4(m.wx - acWorld.x,
                                                m.elevM - dy - acMslM,
                                                m.wz - acWorld.z, 1.f);
                if (clip.w <= 0.f) continue;      // atrás da câmera
                float sx = (clip.x/clip.w * 0.5f + 0.5f) * mio.DisplaySize.x;
                float sy = (1.f - (clip.y/clip.w * 0.5f + 0.5f)) * mio.DisplaySize.y;
                if (sx < -60.f || sx > mio.DisplaySize.x + 60.f ||
                    sy < -60.f || sy > mio.DisplaySize.y + 60.f) continue;

                // Waypoint ativo do LNAV em magenta; demais em ciano
                bool active = gm.mode.lat == GuidanceModule::LatMode::Nav &&
                              gm.activeWpt < (int)gm.fplan.size() &&
                              gm.fplan[gm.activeWpt].name == m.ident;
                ImU32 col = active ? IM_COL32(255, 90, 230, 235)
                                   : IM_COL32( 80, 220, 255, 210);

                char buf[48];
                snprintf(buf, sizeof(buf), "%s %.0f NM",
                         m.ident.c_str(), m.distM / 1852.f);
                // Seta apontando o aeroporto + rótulo centralizado acima
                dl->AddTriangleFilled({sx, sy}, {sx-6.f, sy-12.f}, {sx+6.f, sy-12.f}, col);
                ImVec2 ts = ImGui::CalcTextSize(buf);
                dl->AddRectFilled({sx - ts.x*0.5f - 3.f, sy - 30.f},
                                  {sx + ts.x*0.5f + 3.f, sy - 13.f},
                                  IM_COL32(0, 0, 0, 130), 3.f);
                dl->AddText({sx - ts.x*0.5f, sy - 29.f}, col, buf);
            }
        }

        // ── Painel AFCS Guidance (F1) ─────────────────────────────────────────
        if (guidancePanelOpen && fdmOk) {
            ImGui::SetNextWindowPos({(float)(fw/2 - 220), 10.f}, ImGuiCond_Always);
            ImGui::SetNextWindowSize({440.f, 0.f}, ImGuiCond_Always);
            ImGui::Begin("AFCS GUIDANCE", &guidancePanelOpen,
                ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoCollapse);

            auto modeBtn = [](const char* label, bool active) -> bool {
                if (active) ImGui::PushStyleColor(ImGuiCol_Button,{.1f,.7f,.2f,1.f});
                bool clicked = ImGui::Button(label);
                if (active) ImGui::PopStyleColor();
                return clicked;
            };

            FlyByWire::AircraftState acStNow = fdm.getStateForFBW();

            // ── Velocidade mínima (A/THR floor) ───────────────────────────────
            ImGui::Text("VMIN");
            ImGui::SameLine(70.f);
            ImGui::SetNextItemWidth(100.f);
            ImGui::InputFloat("kt##spd", &gm.targets.speedKt, 1.f, 10.f, "%.0f");
            ImGui::SameLine(200.f);
            bool athrOn = gm.mode.thr == GuidanceModule::ThrMode::SpeedHold;
            if (modeBtn(athrOn ? "A/THR ON##t" : "A/THR##t", athrOn)) {
                if (athrOn) gm.disengageThrottle();
                else        gm.engageSpeed(acStNow, axes.thr);
            }
            if (athrOn)
                ImGui::TextColored({.9f,.9f,.2f,1.f},
                    "  cmd %3.0f%%  lev %3.0f%%",
                    gmOut.throttle[0]*100.f, axes.thr*100.f);

            // ── Proa ───────────────────────────────────────────────────────────
            ImGui::Text("HDG");
            ImGui::SameLine(70.f);
            ImGui::SetNextItemWidth(100.f);
            ImGui::InputFloat("°##hdg", &gm.targets.headingDeg, 1.f, 10.f, "%03.0f");
            gm.targets.headingDeg = fmodf(gm.targets.headingDeg + 360.f, 360.f);
            ImGui::SameLine(200.f);
            bool hdgOn = gm.mode.lat == GuidanceModule::LatMode::HeadingHold;
            if (modeBtn(hdgOn ? "HDG SEL ON##h" : "HDG SEL##h", hdgOn)) {
                if (hdgOn) gm.disengageLat();
                else       gm.engageHeading(acStNow);
            }

            // ── LNAV: flight plan por ICAO ─────────────────────────────────────
            static char icaoBuf[8] = "";
            ImGui::Text("WPT");
            ImGui::SameLine(70.f);
            ImGui::SetNextItemWidth(64.f);
            bool icaoEnter = ImGui::InputText("##icao", icaoBuf, sizeof(icaoBuf),
                ImGuiInputTextFlags_CharsUppercase |
                ImGuiInputTextFlags_EnterReturnsTrue);
            ImGui::SameLine();
            if ((ImGui::SmallButton("ADD##wpt") || icaoEnter) && icaoBuf[0]) {
                double wlat = 0, wlon = 0;
                if (airports.findAirport(icaoBuf, wlat, wlon)) {
                    gm.fplan.push_back({wlat, wlon, icaoBuf});
                    icaoBuf[0] = 0;
                }
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("CLR##wpt")) {
                gm.fplan.clear();
                gm.activeWpt = 0;
                if (gm.mode.lat == GuidanceModule::LatMode::Nav)
                    gm.disengageLat();
            }
            ImGui::SameLine(200.f);
            bool navOn = gm.mode.lat == GuidanceModule::LatMode::Nav;
            if (modeBtn(navOn ? "LNAV ON##n" : "LNAV##n", navOn)) {
                if (navOn)                  gm.disengageLat();
                else if (!gm.fplan.empty()) gm.engageLNAV();
            }
            for (int i = 0; i < (int)gm.fplan.size(); i++) {
                bool act = navOn && i == gm.activeWpt;
                ImGui::TextColored(act ? ImVec4{.4f,.9f,1.f,1.f}
                                       : ImVec4{.55f,.55f,.55f,1.f},
                    act ? "   %d. %s  %.1f NM  <ATIVO>" : "   %d. %s",
                    i + 1, gm.fplan[i].name.c_str(), gm.navDistNm);
            }

            // ── Altitude ───────────────────────────────────────────────────────
            ImGui::Text("ALT");
            ImGui::SameLine(70.f);
            ImGui::SetNextItemWidth(100.f);
            ImGui::InputFloat("ft##alt", &gm.targets.altFt, 100.f, 1000.f, "%.0f");
            ImGui::SameLine(200.f);
            bool altOn = gm.mode.vert == GuidanceModule::VertMode::AltitudeHold;
            if (modeBtn(altOn ? "ALT SEL ON##a" : "ALT SEL##a", altOn)) {
                if (altOn) gm.disengageVert();
                else       gm.engageAltitude(acStNow, fbw);
            }

            // ── Vertical Speed (para Altitude Hold) ────────────────────────────
            ImGui::Text("V/S");
            ImGui::SameLine(70.f);
            ImGui::SetNextItemWidth(100.f);
            ImGui::InputFloat("fpm##vs", &gm.targets.vsFpm, 100.f, 500.f, "%+.0f");
            gm.targets.vsFpm = std::clamp(gm.targets.vsFpm, -3000.f, 3000.f);
            ImGui::SameLine(200.f);
            bool vsManOn = gm.targets.vsManual;
            if (modeBtn(vsManOn ? "V/S ON##vs" : "V/S##vs", vsManOn)) {
                if (vsManOn) {
                    gm.targets.vsManual = false;
                } else {
                    gm.targets.vsFpm = acStNow.vsFpm;
                    gm.targets.vsManual = true;
                    // garante que alt hold está ativo para o VS fazer efeito
                    if (gm.mode.vert != GuidanceModule::VertMode::AltitudeHold)
                        gm.engageAltitude(acStNow, fbw);
                    // climb power boost: se VS positivo e A/THR ativo,
                    // sobe a base do throttle para 80% para não partir de throttle de cruzeiro
                    if (gm.targets.vsFpm > 0.f && gm.athrEngaged())
                        gm.setBaseThrottle(std::max(gm.getBaseThrottle(), 0.80f));
                    else if (gm.targets.vsFpm < 0.f && gm.athrEngaged())
                        gm.setBaseThrottle(std::min(gm.getBaseThrottle(), 0.40f));
                }
            }

            // ── FLCH: muda de nível pela velocidade (ALT alvo @ SPD alvo) ──────
            ImGui::Text("FLCH");
            ImGui::SameLine(70.f);
            ImGui::TextDisabled("→ %5.0f ft @ %3.0f kt",
                                gm.targets.altFt, gm.targets.speedKt);
            ImGui::SameLine(200.f);
            bool flchOn = gm.mode.vert == GuidanceModule::VertMode::Flch;
            if (modeBtn(flchOn ? "FLCH ON##f" : "FLCH##f", flchOn)) {
                if (flchOn) gm.disengageVert();
                else        gm.engageFlch(acStNow, fbw);
            }

            ImGui::Separator();

            // ── Atitude (override FCU — não desengaja) ─────────────────────────
            ImGui::Text("PCH");
            ImGui::SameLine(70.f);
            ImGui::SetNextItemWidth(100.f);
            float pch = gm.targets.pitchDeg;
            if (ImGui::InputFloat("°##pch", &pch, 0.5f, 2.f, "%+.1f"))
                gm.overridePitch(std::clamp(pch, -15.f, 15.f));
            ImGui::SameLine(200.f);
            ImGui::Text("ROL");
            ImGui::SameLine(240.f);
            ImGui::SetNextItemWidth(100.f);
            float rol = gm.targets.bankDeg;
            if (ImGui::InputFloat("°##rol", &rol, 1.f, 5.f, "%+.1f"))
                gm.overrideBank(std::clamp(rol, -30.f, 30.f), fbw);

            ImGui::SetNextItemWidth(200.f);
            bool attOn = gm.mode.vert == GuidanceModule::VertMode::AttitudeHold;
            ImGui::SetCursorPosX((440.f - 200.f) * 0.5f);
            if (modeBtn(attOn ? "ATT HOLD ON##at" : "ATT HOLD##at", attOn)) {
                if (attOn) gm.disengageVert();
                else       gm.engageAttitude(acStNow, fbw);
            }

            ImGui::Separator();

            // ── AP master ──────────────────────────────────────────────────────
            ImGui::SetCursorPosX((440.f - 140.f) * 0.5f);
            bool apOn = gm.isEngaged();
            if (modeBtn(apOn ? "AP ON##ap" : "AP OFF##ap", apOn)) {
                if (apOn) {
                    gm.disengageVert();
                    gm.disengageLat();
                } else {
                    gm.engageAttitude(acStNow, fbw);
                }
            }

            ImGui::TextDisabled("F1 = fechar painel");
            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    sky.cleanup(); ultraFarTiles.cleanup(); farTiles.cleanup();
    closeTiles.cleanup(); nearTiles.cleanup();
    clouds.cleanup(); airports.cleanup(); osm.cleanup(); postfx.cleanup();
    e195.cleanup(); cockpit.cleanup(); minimap.cleanup();
    glDeleteVertexArrays(1,&ptVAO); glDeleteBuffers(1,&ptVBO);
    glDeleteProgram(prog); glDeleteProgram(modelProg); glDeleteProgram(animProg);
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(win); glfwTerminate();
    return 0;
}
