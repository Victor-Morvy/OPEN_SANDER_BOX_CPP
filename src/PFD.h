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
void drawPanel(ImVec2 pos, ImVec2 size,
               float pitchDeg, float rollDeg, float betaDeg,
               float speedKt, float altFt, float vsFpm);

// Só o ADI (horizonte + escada de pitch + escala/ponteiro de bank + slip/skid),
// sem as fitas laterais — exposto à parte para reuso (ex.: textura de cockpit).
void drawAttitude(ImVec2 pos, ImVec2 size, float pitchDeg, float rollDeg, float betaDeg);

}
