#pragma once
#include <glm/glm.hpp>

// HUD sintético (synthetic vision) para a câmera "Nose" — símbolos verdes
// clássicos de HUD, desenhados via ImGui draw list. Os símbolos de atitude
// (pitch ladder, waterline, FPV) são CONFORMAIS com a cena 3D: direções no
// MUNDO projetadas pela mesma view/proj da câmera — alinham com o horizonte
// renderizado e giram com o roll de graça. Os demais (fita de heading,
// caixas de velocidade/altitude, bloco de status) são fixos na tela, dentro
// da área do combiner.
namespace Hud {

struct Data {
    double     yawRad   = 0;      // heading (rad, convenção JSBSim)
    double     pitchRad = 0;      // pitch do avião (rad, + = nariz p/ cima)
    glm::vec3  velWorld{0.f};     // velocidade no frame render (X=E,Y=cima,Z=S)
    float      casKt    = 0.f;    // velocidade calibrada (kt)
    float      altFt    = 0.f;    // altitude baro MSL (ft)
    float      vsFpm    = 0.f;    // velocidade vertical (fpm, + = subindo)
    float      raltFt   = 0.f;    // rádio-altitude / AGL (ft)
    float      gearPos  = 1.f;    // 0=recolhido, 1=baixado (posição animada)
    bool       reverser = false;  // reversor DEPLOYADO (estado real pós-FBW)
    float      throttle = 0.f;    // manete comandada 0..1 (cmd efetivo FADEC)

    // ── ILS sintético ──
    // Cruzeta de desvio (approach mais alinhado no yaw / armado):
    //   locDevDeg + = voar pra direita   gsDevDeg + = voar pra cima
    //   todNm: distância até o ponto de início de descida 3° na altura atual
    //   (+ = ainda antes do ponto; ≤0 = já deveria estar descendo)
    struct Ils {
        bool  on = false;
        float locDevDeg = 0.f, gsDevDeg = 0.f;
        float distNm = 0.f, todNm = 0.f;
        char  label[24] = {0};
    };
    Ils ils;

    // Linhas de aproximação 3D (conformais): rampa de 3° subindo da cabeceira
    // contra a direção de pouso. Até 2 (ex.: pistas paralelas próximas).
    struct AppPath {
        glm::vec3 thrRel{0.f};  // cabeceira rel. ao avião (frame render, Y=ΔMSL)
        glm::vec3 dir{0.f};     // direção de POUSO (unit, horizontal)
        float     todM = 0.f;   // dist. da cabeceira do ponto de descida 3°
        char      label[24] = {0};
    };
    int     nPaths = 0;
    AppPath paths[2];
};

// Desenha o HUD completo:
//  - pitch ladder (5°/10°, tracejada abaixo do horizonte) — conformal
//  - waterline ─\_/─ na direção do NARIZ (yaw+pitch) — conformal
//  - flight path vector (círculo+asas+cauda) na direção da VELOCIDADE —
//    conformal; omitido abaixo de ~5 kt
//  - caixa de heading no topo (box digital %03d, mesma linguagem das demais)
//  - caixa de velocidade (CAS) à esquerda; altitude (MSL) e V/S à direita
//  - bloco inferior direito: RALT, GEAR UP/DOWN, REV (se ativo), THR %
// Deve ser chamado entre ImGui::NewFrame() e ImGui::Render().
void draw(const glm::mat4& view, const glm::mat4& proj, const Data& d,
          int fw, int fh);

}
