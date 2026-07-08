#include "Hud.h"
#include <imgui.h>
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace Hud {

static constexpr float D2R = 3.14159265358979323846f / 180.f;

// Verde clássico de HUD/combiner + outline verde escuro (halo de contraste —
// sem ele os símbolos somem contra céu claro/nuvem)
static const ImU32 HUD_COL     = IM_COL32(30, 255, 100, 220);
static const ImU32 HUD_COL_DIM = IM_COL32(30, 255, 100, 150);
static const ImU32 HUD_OUT     = IM_COL32(0, 62, 22, 210);

// Primitivas com outline: o traço escuro mais grosso vai por baixo do claro
// (AddLine/AddCircle/AddRect centram a espessura → sobra ~1.5px de borda
// escura de cada lado).
static void oLine(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 col, float th) {
    dl->AddLine(a, b, HUD_OUT, th + 3.f);
    dl->AddLine(a, b, col, th);
}
static void oCircle(ImDrawList* dl, ImVec2 c, float r, ImU32 col, int seg, float th) {
    dl->AddCircle(c, r, HUD_OUT, seg, th + 3.f);
    dl->AddCircle(c, r, col, seg, th);
}
static void oRect(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 col, float round, float th) {
    dl->AddRect(a, b, HUD_OUT, round, 0, th + 3.f);
    dl->AddRect(a, b, col, round, 0, th);
}
// Fonte do HUD: default do ImGui ampliada (AddText com tamanho explícito) —
// os 13px padrão ficavam pequenos demais lendo o combiner em voo.
// Todo texto sai com halo escuro em 8 direções (1px).
static constexpr float FS = 20.f;
static ImVec2 hudTextSize(const char* s) {
    return ImGui::GetFont()->CalcTextSizeA(FS, FLT_MAX, 0.f, s);
}
static void hudText(ImDrawList* dl, ImVec2 pos, ImU32 col, const char* s) {
    ImFont* f = ImGui::GetFont();
    static const ImVec2 OFF[8] = {{-1,-1},{0,-1},{1,-1},{-1,0},{1,0},{-1,1},{0,1},{1,1}};
    for (const auto& o : OFF)
        dl->AddText(f, FS, {pos.x + o.x, pos.y + o.y}, HUD_OUT, s);
    dl->AddText(f, FS, pos, col, s);
}

// Caixa digital (CAS/ALT/HDG/VS): retângulo com valor alinhado à direita.
static void hudBox(ImDrawList* dl, ImVec2 a, ImVec2 b, const char* val) {
    oRect(dl, a, b, HUD_COL, 2.f, 2.f);
    ImVec2 ts = hudTextSize(val);
    hudText(dl, {b.x - ts.x - 6.f, (a.y + b.y) * 0.5f - ts.y * 0.5f}, HUD_COL, val);
}

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

// Projeta um PONTO relativo ao avião (w=1) — usado pelas linhas de approach,
// que têm posição real no mundo (não são direções no infinito).
static bool projPoint(const glm::vec3& rel, const glm::mat4& vp,
                      int fw, int fh, ImVec2& outPx) {
    glm::vec4 clip = vp * glm::vec4(rel, 1.f);
    if (clip.w <= 1.f) return false;            // atrás/em cima da câmera
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
    oLine(dl, a, b, col, thick);
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
    oLine(dl, a, b, col, 2.f);
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
            ImVec2 ts = hudTextSize(lbl);
            ImVec2 e;
            if (projDir(rungPoint(yaw, (float)p, -(halfW + 1.5f)), vp, fw, fh, e))
                hudText(dl, {e.x - ts.x, e.y - ts.y * 0.5f}, col, lbl);
            if (projDir(rungPoint(yaw, (float)p,  halfW + 1.5f), vp, fw, fh, e))
                hudText(dl, {e.x, e.y - ts.y * 0.5f}, col, lbl);
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
        oLine(dl, {c.x - w,       c.y},     {c.x - v * 1.4f, c.y},     wl, 3.f);
        oLine(dl, {c.x - v * 1.4f, c.y},    {c.x - v * 0.7f, c.y + v}, wl, 3.f);
        oLine(dl, {c.x - v * 0.7f, c.y + v}, {c.x,           c.y},     wl, 3.f);
        oLine(dl, {c.x,           c.y},     {c.x + v * 0.7f, c.y + v}, wl, 3.f);
        oLine(dl, {c.x + v * 0.7f, c.y + v}, {c.x + v * 1.4f, c.y},    wl, 3.f);
        oLine(dl, {c.x + v * 1.4f, c.y},    {c.x + w,        c.y},     wl, 3.f);
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
            oCircle(dl, f, r, HUD_COL, 24, 2.5f);
            oLine(dl, {f.x - r,        f.y}, {f.x - r - wing, f.y}, HUD_COL, 2.5f);
            oLine(dl, {f.x + r,        f.y}, {f.x + r + wing, f.y}, HUD_COL, 2.5f);
            oLine(dl, {f.x, f.y - r}, {f.x, f.y - r - tail},        HUD_COL, 2.5f);
        }
    }

    // ── ILS: linhas de aproximação 3D (rampa de 3° a partir da cabeceira) ───
    // Conformais: pontos no mundo relativos ao avião, projetados com w=1 e
    // curvatura da Terra por amostra (mesma correção do terreno). O marcador
    // TOD fica no ponto da rampa que cruza a ALTURA ATUAL do avião — é ali
    // que se inicia a descida de 3°.
    for (int pi = 0; pi < d.nPaths && pi < 2; ++pi) {
        const auto& ap = d.paths[pi];
        constexpr float TAN3   = 0.052408f;   // tan(3°)
        constexpr float STEP_M = 926.f;       // meia milha náutica
        float maxS = std::min(std::max(ap.todM * 1.3f, 9260.f), 46000.f);

        ImVec2 prev; bool prevOk = false;
        for (float s = 0.f; s <= maxS; s += STEP_M) {
            glm::vec3 p = ap.thrRel - ap.dir * s + glm::vec3(0.f, s * TAN3, 0.f);
            float dh = sqrtf(p.x * p.x + p.z * p.z);
            p.y -= dh * dh / (2.f * 6371000.f);       // curvatura da Terra
            ImVec2 px;
            bool ok = projPoint(p, vp, fw, fh, px);
            if (ok && prevOk) oLine(dl, prev, px, HUD_COL_DIM, 2.f);
            prev = px; prevOk = ok;
        }
        // Cabeceira: quadradinho
        ImVec2 tp;
        if (projPoint(ap.thrRel, vp, fw, fh, tp))
            oRect(dl, {tp.x - 5.f, tp.y - 5.f}, {tp.x + 5.f, tp.y + 5.f},
                  HUD_COL, 0.f, 2.f);
        // TOD: círculo + rótulo no ponto de início de descida
        if (ap.todM > 500.f && ap.todM < maxS) {
            glm::vec3 p = ap.thrRel - ap.dir * ap.todM
                        + glm::vec3(0.f, ap.todM * TAN3, 0.f);
            float dh = sqrtf(p.x * p.x + p.z * p.z);
            p.y -= dh * dh / (2.f * 6371000.f);
            ImVec2 px;
            if (projPoint(p, vp, fw, fh, px)) {
                oCircle(dl, px, 8.f, HUD_COL, 20, 2.5f);
                hudText(dl, {px.x + 12.f, px.y - 10.f}, HUD_COL, "TOD");
            }
        }
        // Identificação perto da cabeceira
        if (projPoint(ap.thrRel, vp, fw, fh, tp))
            hudText(dl, {tp.x + 8.f, tp.y + 6.f}, HUD_COL_DIM, ap.label);
    }

    // ── ILS: cruzeta de desvio (LOC vertical, GS horizontal) ─────────────────
    // Agulhas fly-to no centro do combiner: centralizar as duas = na rampa e
    // no eixo. Escala: LOC ±2.5° (40 px/°), GS ±0.7° (143 px/°).
    if (d.ils.on) {
        float cx = (clipMin.x + clipMax.x) * 0.5f;
        float cy = (clipMin.y + clipMax.y) * 0.5f;
        const float SPAN = 60.f;

        // Pontos da escala (2 por lado, a cada 50 px) nos dois eixos
        for (int i : {-2, -1, 1, 2}) {
            oCircle(dl, {cx + (float)i * 50.f, cy}, 2.5f, HUD_COL_DIM, 8, 1.5f);
            oCircle(dl, {cx, cy + (float)i * 50.f}, 2.5f, HUD_COL_DIM, 8, 1.5f);
        }
        // Agulha do LOC (vertical, desloca em X)
        float lx = cx + std::clamp(d.ils.locDevDeg, -2.5f, 2.5f) * 40.f;
        oLine(dl, {lx, cy - SPAN}, {lx, cy + SPAN}, HUD_COL, 2.5f);
        // Agulha do GS (horizontal, desloca em Y; + = voar pra cima = agulha acima)
        float gy = cy - std::clamp(d.ils.gsDevDeg, -0.7f, 0.7f) * 143.f;
        oLine(dl, {cx - SPAN, gy}, {cx + SPAN, gy}, HUD_COL, 2.5f);

        // Identificação + distância + TOD (base do combiner, centralizado)
        char il[64];
        if (d.ils.todNm > 0.1f)
            snprintf(il, sizeof(il), "ILS %s  %.1fNM  TOD %.1fNM",
                     d.ils.label, d.ils.distNm, d.ils.todNm);
        else
            snprintf(il, sizeof(il), "ILS %s  %.1fNM  DESCIDA",
                     d.ils.label, d.ils.distNm);
        ImVec2 ts = hudTextSize(il);
        hudText(dl, {cx - ts.x * 0.5f, clipMax.y - ts.y - 4.f}, HUD_COL, il);
    }

    // ── Caixa de heading (topo do combiner, centralizada) ───────────────────
    // Mesma linguagem visual das caixas de CAS/ALT: box digital com a proa
    // %03d e rótulo discreto.
    {
        char buf[8];
        int hdg = (int)lroundf(fmodf((float)(d.yawRad / D2R) + 360.f, 360.f)) % 360;
        snprintf(buf, sizeof(buf), "%03d", hdg);
        float cx = (clipMin.x + clipMax.x) * 0.5f;
        const float w = 72.f, h = 30.f;
        ImVec2 a{cx - w * 0.5f, clipMin.y + 24.f}, b{cx + w * 0.5f, clipMin.y + 24.f + h};
        hudBox(dl, a, b, buf);
        ImVec2 tl = hudTextSize("HDG");
        hudText(dl, {cx - tl.x * 0.5f, a.y - tl.y - 2.f}, HUD_COL_DIM, "HDG");
    }

    // ── Caixas de velocidade (esquerda), altitude e V/S (direita) ───────────
    {
        float cy = (clipMin.y + clipMax.y) * 0.5f;
        const float boxH = 30.f;
        char buf[16];

        // CAS à esquerda + Mach logo abaixo (mesma coluna)
        snprintf(buf, sizeof(buf), "%d", (int)lroundf(d.casKt));
        {
            ImVec2 a{clipMin.x + 4.f, cy - boxH * 0.5f};
            ImVec2 b{a.x + 80.f, cy + boxH * 0.5f};
            hudBox(dl, a, b, buf);
            hudText(dl, {a.x, a.y - 24.f}, HUD_COL_DIM, "CAS KT");

            snprintf(buf, sizeof(buf), ".%03d", (int)lroundf(d.mach * 1000.f));
            ImVec2 ma{a.x, b.y + 10.f}, mb{b.x, b.y + 10.f + boxH};
            hudBox(dl, ma, mb, buf);
            hudText(dl, {ma.x, mb.y + 2.f}, HUD_COL_DIM, "MACH");
        }

        // Altitude à direita
        snprintf(buf, sizeof(buf), "%d", (int)lroundf(d.altFt));
        {
            ImVec2 a{clipMax.x - 4.f - 96.f, cy - boxH * 0.5f};
            ImVec2 b{clipMax.x - 4.f, cy + boxH * 0.5f};
            hudBox(dl, a, b, buf);
            ImVec2 tl = hudTextSize("ALT FT");
            hudText(dl, {b.x - tl.x, a.y - 24.f}, HUD_COL_DIM, "ALT FT");

            // V/S logo abaixo da altitude (mesma coluna)
            snprintf(buf, sizeof(buf), "%+d", (int)lroundf(d.vsFpm));
            ImVec2 va{a.x, b.y + 10.f}, vb{b.x, b.y + 10.f + boxH};
            hudBox(dl, va, vb, buf);
            ImVec2 vl = hudTextSize("VS FPM");
            hudText(dl, {vb.x - vl.x, vb.y + 2.f}, HUD_COL_DIM, "VS FPM");
        }
    }

    // ── Bloco de status (canto inferior direito do combiner) ────────────────
    // RALT | GEAR UP/DOWN | REV (se deployado) | THR %
    {
        char buf[24];
        float lineH = FS + 4.f;
        float x1 = clipMax.x - 6.f;
        float y  = clipMax.y - 6.f - lineH * 4.f;
        auto textR = [&](ImU32 col, const char* s){
            ImVec2 ts = hudTextSize(s);
            hudText(dl, {x1 - ts.x, y}, col, s);
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
