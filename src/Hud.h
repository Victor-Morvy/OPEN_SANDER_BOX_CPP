#pragma once
#include <glm/glm.hpp>

// HUD sintético (synthetic vision) para a câmera "Nose" — símbolos verdes
// clássicos de HUD, desenhados via ImGui draw list, mas CONFORMAIS com a
// cena 3D: cada degrau é uma direção no MUNDO (heading do avião + pitch do
// degrau) projetada pela mesma view/proj da câmera — com roll ou head-look,
// a escada gira/desloca junto com o horizonte renderizado de verdade.
namespace Hud {

// Pitch ladder (degraus a cada 5°, rotulados a cada 10°, tracejados abaixo
// do horizonte) + linha d'água (waterline/boresight): símbolo fixo na
// direção em que o NARIZ aponta (yaw+pitch do avião), não no centro da tela.
// + Flight path vector (círculo com asas e cauda): direção do vetor
// VELOCIDADE — pra onde o avião está indo de fato (inclui alpha/beta/vento).
// view/proj: câmera atual (Nose). yaw/pitch em rad (convenção
// Telemetry/JSBSim: pitch + = nariz para cima). velWorld: velocidade no
// frame render (X=Leste, Y=cima, Z=Sul) — só a direção importa; abaixo de
// ~5 kt o FPV é omitido (direção sem significado no solo parado).
// Deve ser chamado entre ImGui::NewFrame() e ImGui::Render().
void draw(const glm::mat4& view, const glm::mat4& proj,
          double yawRad, double pitchRad, const glm::vec3& velWorld,
          int fw, int fh);

}
