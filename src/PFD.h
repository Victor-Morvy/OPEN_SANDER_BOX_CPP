#pragma once
#include <imgui.h>

// Primary Flight Display — instrumentos desenhados via ImGui draw list (sem
// textura, sem geometria 3D).
namespace PFD {

// Painel completo: fita de velocidade | ADI (+ escala de bank e slip/skid) |
// fita de altitude | fita de V/S — layout clássico de EFIS, da esquerda p/
// direita. `size` é a área TOTAL do painel; a altura das fitas/ADI = size.y,
// as larguras internas são proporções fixas de size.x (ver PFD.cpp).
// Convenção de sinal (igual ao resto do projeto — JSBSim):
//   pitchDeg + = nariz para cima     rollDeg + = asa direita para baixo
//   betaDeg  + = mesma leitura do painel SUPERFICIES ("Beta")
// FPV (flight path vector) no ADI, preso à bola giroscópica:
//   fpaDeg   = ângulo de trajetória (γ, + = subindo) — posição na escada
//   driftDeg = deriva (track − heading, + = direita)  — offset lateral
//   showFpv  = false esconde (velocidade baixa demais p/ direção significar algo)
// ILS sintético (cruzeta no ADI): mostrado quando ilsOn.
//   locDevDeg + = voar pra direita (losango do LOC à direita), escala ±2.5°
//   gsDevDeg  + = voar pra cima (losango do GS acima),         escala ±0.7°
//   ilsLabel/ilsDistNm: identificação e distância à cabeceira.
void drawPanel(ImVec2 pos, ImVec2 size,
               float pitchDeg, float rollDeg, float betaDeg,
               float speedKt, float altFt, float vsFpm,
               float fpaDeg, float driftDeg, bool showFpv,
               bool ilsOn = false, float locDevDeg = 0.f, float gsDevDeg = 0.f,
               const char* ilsLabel = "", float ilsDistNm = 0.f);

// Só o ADI (horizonte + escada de pitch + escala/ponteiro de bank + slip/skid
// + FPV + cruzeta ILS), sem as fitas laterais — exposto à parte para reuso.
void drawAttitude(ImVec2 pos, ImVec2 size, float pitchDeg, float rollDeg, float betaDeg,
                  float fpaDeg, float driftDeg, bool showFpv,
                  bool ilsOn = false, float locDevDeg = 0.f, float gsDevDeg = 0.f,
                  const char* ilsLabel = "", float ilsDistNm = 0.f);

}
