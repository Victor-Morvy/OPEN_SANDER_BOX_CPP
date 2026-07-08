#include "Hud.h"
#include <imgui.h>
#include <cmath>
#include <cstdio>

namespace Hud {

static constexpr float D2R = 3.14159265358979323846f / 180.f;

// Verde clássico de HUD/combiner
static const ImU32 HUD_COL     = IM_COL32(30, 255, 100, 220);
static const ImU32 HUD_COL_DIM = IM_COL32(30, 255, 100, 150);

// Direção no mundo (frame render: X=Leste, Y=cima, Z=Sul; North=-Z) para um
// heading/pitch dados — mesma fórmula do aircraftForward() do main.cpp.
static glm::vec3 dirWorld(float yaw, float pitch) {
    return glm::vec3( sinf(yaw) * cosf(pitch),
                      sinf(pitch),
                     -cosf(yaw) * cosf(pitch));
}

// Projeta uma DIREÇÃO (não um ponto) para pixels de tela. w=0 elimina a
// translação da view — equivale a projetar o ponto de fuga da direção, que é
// exatamente onde um degrau "no infinito" deve aparecer (conformal com o
// horizonte, que também está efetivamente no infinito).
static bool projDir(const glm::vec3& dir, const glm::mat4& vp,
                    int fw, int fh, ImVec2& outPx) {
    glm::vec4 clip = vp * glm::vec4(dir, 0.f);
    if (clip.w <= 1e-6f) return false;          // atrás da câmera
    glm::vec3 ndc = glm::vec3(clip) / clip.w;
    outPx.x = (ndc.x * 0.5f + 0.5f) * (float)fw;
    outPx.y = (1.f - (ndc.y * 0.5f + 0.5f)) * (float)fh;
    return true;
}

// Direção de um ponto do degrau: centro no eixo da escada (heading yaw,
// pitch p) e extensão LATERAL por GRANDE CÍRCULO — gira o centro em torno do
// eixo vertical da escada, com largura angular constante:
//   e(az) = d(p)·cos(az) + right·sin(az),  right = (cos(yaw), 0, sin(yaw))
// NÃO usar pitch constante com offset de azimute (paralelo de latitude): esses
// círculos encolhem com cos(p) e convergem pro zênite — a escada afunilava
// rumo a um "ponto de fuga" no pitch 90° (defeito notado pelo Victor olhando
// degraus altos de baixo/de cima). Com grande círculo todo degrau tem a mesma
// largura angular e fica perpendicular ao eixo da escada, como num HUD real.
static glm::vec3 rungPoint(float yaw, float pitchDeg, float azDeg) {
    glm::vec3 d = dirWorld(yaw, pitchDeg * D2R);
    glm::vec3 right{cosf(yaw), 0.f, sinf(yaw)};
    float a = azDeg * D2R;
    return d * cosf(a) + right * sinf(a);
}

// Segmento de degrau entre dois offsets laterais (graus de arco).
static void rungSegment(ImDrawList* dl, const glm::mat4& vp, int fw, int fh,
                        float yaw, float pitchDeg, float az0, float az1,
                        ImU32 col, float thick) {
    ImVec2 a, b;
    if (!projDir(rungPoint(yaw, pitchDeg, az0), vp, fw, fh, a)) return;
    if (!projDir(rungPoint(yaw, pitchDeg, az1), vp, fw, fh, b)) return;
    dl->AddLine(a, b, col, thick);
}

// Tick vertical na ponta do degrau, apontando para o horizonte (para baixo
// nos degraus positivos, para cima nos negativos) — convenção clássica.
// O tick segue o eixo da escada: mesmo offset lateral, pitch deslocado.
static void rungTick(ImDrawList* dl, const glm::mat4& vp, int fw, int fh,
                     float yaw, float pitchDeg, float azDeg, ImU32 col) {
    float toHorizon = (pitchDeg > 0.f) ? -1.5f : 1.5f;
    ImVec2 a, b;
    if (!projDir(rungPoint(yaw, pitchDeg, azDeg), vp, fw, fh, a)) return;
    if (!projDir(rungPoint(yaw, pitchDeg + toHorizon, azDeg), vp, fw, fh, b)) return;
    dl->AddLine(a, b, col, 2.f);
}

void draw(const glm::mat4& view, const glm::mat4& proj,
          const Data& d, int fw, int fh) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const glm::mat4 vp = proj * view;
    const float yaw = (float)d.yawRad;

    // Área útil do combiner: caixa central — evita a escada invadir os
    // painéis ImGui das bordas e imita o campo limitado de um HUD real.
    ImVec2 clipMin{(float)fw * 0.24f, (float)fh * 0.10f};
    ImVec2 clipMax{(float)fw * 0.76f, (float)fh * 0.88f};
    dl->PushClipRect(clipMin, clipMax, true);

    // ── Pitch ladder ─────────────────────────────────────────────────────────
    // Degraus em pitch FIXO no mundo, no azimute do heading atual: projetados
    // pela câmera, alinham com o horizonte 3D e giram sozinhos com o roll.
    char lbl[8];
    for (int p = -90; p <= 90; p += 5) {
        bool major = (p % 10 == 0);
        float halfW = (p == 0) ? 30.f : (major ? 8.f : 5.f);   // azimute, graus
        float gap   = (p == 0) ? 4.5f : 3.f;
        ImU32 col   = (p == 0) ? HUD_COL : (major ? HUD_COL : HUD_COL_DIM);
        float thick = (p == 0) ? 3.f : (major ? 2.5f : 1.8f);

        if (p >= 0) {
            // Horizonte e degraus positivos: linha sólida
            rungSegment(dl, vp, fw, fh, yaw, (float)p, -halfW, -gap, col, thick);
            rungSegment(dl, vp, fw, fh, yaw, (float)p,  gap,  halfW, col, thick);
        } else {
            // Degraus negativos: tracejado (3 traços por lado)
            float span = halfW - gap;
            for (int d = 0; d < 5; d += 2) {
                float s0 = gap + span * (float)d / 5.f;
                float s1 = gap + span * (float)(d + 1) / 5.f;
                rungSegment(dl, vp, fw, fh, yaw, (float)p, -s1, -s0, col, thick);
                rungSegment(dl, vp, fw, fh, yaw, (float)p,  s0,  s1, col, thick);
            }
        }

        if (p != 0) {
            rungTick(dl, vp, fw, fh, yaw, (float)p, -halfW, col);
            rungTick(dl, vp, fw, fh, yaw, (float)p,  halfW, col);
        }

        // Rótulo nas duas pontas dos múltiplos de 10°
        if (major && p != 0) {
            snprintf(lbl, sizeof(lbl), "%d", p > 0 ? p : -p);
            ImVec2 ts = ImGui::CalcTextSize(lbl);
            ImVec2 e;
            if (projDir(rungPoint(yaw, (float)p, -(halfW + 1.5f)), vp, fw, fh, e))
                dl->AddText({e.x - ts.x, e.y - ts.y * 0.5f}, col, lbl);
            if (projDir(rungPoint(yaw, (float)p,  halfW + 1.5f), vp, fw, fh, e))
                dl->AddText({e.x, e.y - ts.y * 0.5f}, col, lbl);
        }
    }

    // ── Linha d'água (waterline / boresight) ────────────────────────────────
    // Fixa na direção do NARIZ (yaw+pitch do avião): com head-look ela sai do
    // centro da tela; olhando reto, fica no centro. Formato clássico ─\_/─
    // (W achatado), tamanho fixo em pixels.
    ImVec2 c;
    if (projDir(dirWorld(yaw, (float)d.pitchRad), vp, fw, fh, c)) {
        const float w = 26.f, v = 9.f;     // meia-envergadura e profundidade do V
        const ImU32 wl = HUD_COL;
        dl->AddLine({c.x - w,       c.y},     {c.x - v * 1.4f, c.y},     wl, 3.f);
        dl->AddLine({c.x - v * 1.4f, c.y},    {c.x - v * 0.7f, c.y + v}, wl, 3.f);
        dl->AddLine({c.x - v * 0.7f, c.y + v}, {c.x,           c.y},     wl, 3.f);
        dl->AddLine({c.x,           c.y},     {c.x + v * 0.7f, c.y + v}, wl, 3.f);
        dl->AddLine({c.x + v * 0.7f, c.y + v}, {c.x + v * 1.4f, c.y},    wl, 3.f);
        dl->AddLine({c.x + v * 1.4f, c.y},    {c.x + w,        c.y},     wl, 3.f);
    }

    // ── Flight path vector ───────────────────────────────────────────────────
    // Direção do vetor VELOCIDADE (inclui alpha/beta/vento) — pra onde o
    // avião realmente vai, não pra onde aponta. Voando coordenado ele senta
    // no degrau do ângulo de trajetória; a distância vertical até a waterline
    // é o alpha. Símbolo clássico: círculo + asinhas + cauda, px fixos.
    // ~8 fps ≈ 5 kt: parado/taxiando devagar a direção não significa nada.
    if (glm::dot(d.velWorld, d.velWorld) > 8.f * 8.f) {
        ImVec2 f;
        if (projDir(glm::normalize(d.velWorld), vp, fw, fh, f)) {
            const float r = 8.f, wing = 14.f, tail = 8.f;
            dl->AddCircle(f, r, HUD_COL, 24, 2.5f);
            dl->AddLine({f.x - r,        f.y}, {f.x - r - wing, f.y}, HUD_COL, 2.5f);
            dl->AddLine({f.x + r,        f.y}, {f.x + r + wing, f.y}, HUD_COL, 2.5f);
            dl->AddLine({f.x, f.y - r}, {f.x, f.y - r - tail},        HUD_COL, 2.5f);
        }
    }

    // ── Fita de heading (topo do combiner, fixa na tela) ────────────────────
    // Janela de ±25°, tick a cada 5°, rótulo %03d a cada 10°, caret central
    // apontando para baixo na proa atual.
    {
        float hdgDeg = fmodf((float)(d.yawRad / D2R) + 360.f, 360.f);
        float cx = (clipMin.x + clipMax.x) * 0.5f;
        float y0 = clipMin.y + 14.f;
        float pxPerDeg = (clipMax.x - clipMin.x) / 50.f;

        dl->AddLine({clipMin.x, y0}, {clipMax.x, y0}, HUD_COL_DIM, 1.5f);
        int h0 = (int)floorf((hdgDeg - 25.f) / 5.f) * 5;
        for (int h = h0; h <= h0 + 55; h += 5) {
            float diff = fmodf((float)h - hdgDeg + 540.f, 360.f) - 180.f;
            if (fabsf(diff) > 25.f) continue;
            float x = cx + diff * pxPerDeg;
            bool major = (h % 10 == 0);
            dl->AddLine({x, y0}, {x, y0 + (major ? 11.f : 6.f)}, HUD_COL, major ? 2.f : 1.5f);
            if (major) {
                snprintf(lbl, sizeof(lbl), "%03d", ((h % 360) + 360) % 360);
                ImVec2 ts = ImGui::CalcTextSize(lbl);
                dl->AddText({x - ts.x * 0.5f, y0 + 13.f}, HUD_COL, lbl);
            }
        }
        // Caret da proa atual (aponta pra baixo, tocando a linha da fita)
        dl->AddTriangleFilled({cx, y0}, {cx - 6.f, y0 - 9.f}, {cx + 6.f, y0 - 9.f},
                              HUD_COL);
    }

    // ── Caixas de velocidade (esquerda) e altitude (direita) ────────────────
    {
        float cy = (clipMin.y + clipMax.y) * 0.5f;
        char buf[16];

        // CAS à esquerda
        snprintf(buf, sizeof(buf), "%d", (int)lroundf(d.casKt));
        {
            ImVec2 a{clipMin.x + 4.f, cy - 14.f}, b{clipMin.x + 4.f + 64.f, cy + 14.f};
            dl->AddRect(a, b, HUD_COL, 2.f, 0, 2.f);
            ImVec2 ts = ImGui::CalcTextSize(buf);
            dl->AddText({b.x - ts.x - 6.f, cy - ts.y * 0.5f}, HUD_COL, buf);
            dl->AddText({a.x, a.y - 18.f}, HUD_COL_DIM, "CAS KT");
        }

        // Altitude à direita
        snprintf(buf, sizeof(buf), "%d", (int)lroundf(d.altFt));
        {
            ImVec2 a{clipMax.x - 4.f - 76.f, cy - 14.f}, b{clipMax.x - 4.f, cy + 14.f};
            dl->AddRect(a, b, HUD_COL, 2.f, 0, 2.f);
            ImVec2 ts = ImGui::CalcTextSize(buf);
            dl->AddText({b.x - ts.x - 6.f, cy - ts.y * 0.5f}, HUD_COL, buf);
            ImVec2 tl = ImGui::CalcTextSize("ALT FT");
            dl->AddText({b.x - tl.x, a.y - 18.f}, HUD_COL_DIM, "ALT FT");
        }
    }

    // ── Bloco de status (canto inferior direito do combiner) ────────────────
    // RALT | GEAR UP/DOWN | REV (se deployado) | THR %
    {
        char buf[24];
        float lineH = ImGui::GetTextLineHeight() + 3.f;
        float x1 = clipMax.x - 6.f;
        float y  = clipMax.y - 6.f - lineH * 4.f;
        auto textR = [&](ImU32 col, const char* s){
            ImVec2 ts = ImGui::CalcTextSize(s);
            dl->AddText({x1 - ts.x, y}, col, s);
            y += lineH;
        };

        snprintf(buf, sizeof(buf), "RALT %d", (int)lroundf(d.raltFt));
        textR(HUD_COL, buf);

        if (d.gearPos > 0.99f)      textR(HUD_COL, "GEAR DOWN");
        else if (d.gearPos < 0.01f) textR(HUD_COL_DIM, "GEAR UP");
        else                        textR(HUD_COL, "GEAR TRANS");

        if (d.reverser) textR(IM_COL32(255, 200, 40, 235), "REV");
        else            y += lineH;   // mantém o bloco estável sem o REV

        snprintf(buf, sizeof(buf), "THR %d%%", (int)lroundf(d.throttle * 100.f));
        textR(HUD_COL, buf);
    }

    dl->PopClipRect();
}

}
