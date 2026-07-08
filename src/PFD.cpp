#include "PFD.h"
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace PFD {

static constexpr float PFD_D2R = 3.14159265358979323846f / 180.f;

// Ponto "preso na bola" giroscópica: (x,y) já inclui o deslocamento de pitch
// (espaço local, X=direita+, Y=baixo+, origem no centro do instrumento).
// A bola fica estabilizada ao mundo — do ponto de vista do painel (fixo à
// aeronave), ela gira no sentido OPOSTO ao roll da aeronave. Para roll
// positivo (asa direita para baixo), um ponto à direita do centro (x>0,y=0)
// precisa subir na tela (y' menor) — verificado: com r=rollRad>0,
// yr = -x*sin(r) < 0 ⇒ sobe. Confere com a leitura real de um ADI.
static ImVec2 ballPoint(ImVec2 center, float x, float y, float cosR, float sinR) {
    float xr =  x*cosR + y*sinR;
    float yr = -x*sinR + y*cosR;
    return ImVec2(center.x + xr, center.y + yr);
}

// Caixa de valor central de uma fita (velocidade/altitude): retângulo escuro
// com borda clara e o número atual, na altura vertical central da fita.
static void drawValueBox(ImDrawList* dl, ImVec2 center, float w, float h,
                          const char* text) {
    ImVec2 a{center.x - w*0.5f, center.y - h*0.5f};
    ImVec2 b{center.x + w*0.5f, center.y + h*0.5f};
    dl->AddRectFilled(a, b, IM_COL32(10, 12, 16, 235), 3.f);
    dl->AddRect(a, b, IM_COL32(255, 255, 255, 255), 3.f, 0, 2.f);
    ImVec2 ts = ImGui::CalcTextSize(text);
    dl->AddText({center.x - ts.x*0.5f, center.y - ts.y*0.5f},
                IM_COL32(255, 255, 255, 255), text);
}

// ── Fita de velocidade (CAS, kt) — janela ±40 kt, marca a cada 10 ──────────
static void drawSpeedTape(ImDrawList* dl, ImVec2 pos, ImVec2 size, float speedKt) {
    ImVec2 pMax{pos.x + size.x, pos.y + size.y};
    ImVec2 center{pos.x + size.x*0.5f, pos.y + size.y*0.5f};
    dl->AddRectFilled(pos, pMax, IM_COL32(15, 16, 20, 235));
    dl->PushClipRect(pos, pMax, true);

    const float pxPerKt = size.y / 80.f;
    int v0 = ((int)(speedKt / 10.f)) * 10 - 40;
    char buf[8];
    for (int v = v0; v <= v0 + 90; v += 10) {
        if (v < 0) continue;
        float y = center.y - ((float)v - speedKt) * pxPerKt;
        if (y < pos.y - 4 || y > pMax.y + 4) continue;
        float tickW = (v % 20 == 0) ? size.x * 0.35f : size.x * 0.18f;
        dl->AddLine({pMax.x - tickW, y}, {pMax.x, y}, IM_COL32(230,230,230,255), 2.f);
        if (v % 20 == 0) {
            snprintf(buf, sizeof(buf), "%d", v);
            ImVec2 ts = ImGui::CalcTextSize(buf);
            dl->AddText({pMax.x - tickW - ts.x - 4.f, y - ts.y*0.5f},
                        IM_COL32(230,230,230,255), buf);
        }
    }
    dl->PopClipRect();
    dl->AddRect(pos, pMax, IM_COL32(80,80,90,255), 4.f, 0, 2.f);

    char cur[8]; snprintf(cur, sizeof(cur), "%d", (int)std::lround(speedKt));
    float boxW = size.x*0.62f;
    drawValueBox(dl, {pMax.x - 2.f - boxW*0.5f, center.y}, boxW, 22.f, cur);
}

// ── Fita de altitude (ft) — janela ±500 ft, marca a cada 100 ───────────────
static void drawAltTape(ImDrawList* dl, ImVec2 pos, ImVec2 size, float altFt) {
    ImVec2 pMax{pos.x + size.x, pos.y + size.y};
    ImVec2 center{pos.x + size.x*0.5f, pos.y + size.y*0.5f};
    dl->AddRectFilled(pos, pMax, IM_COL32(15, 16, 20, 235));
    dl->PushClipRect(pos, pMax, true);

    const float pxPerFt = size.y / 1000.f;   // ±500 ft de janela
    int v0 = ((int)(altFt / 100.f)) * 100 - 500;
    char buf[8];
    for (int v = v0; v <= v0 + 1100; v += 100) {
        float y = center.y - ((float)v - altFt) * pxPerFt;
        if (y < pos.y - 4 || y > pMax.y + 4) continue;
        float tickW = (v % 500 == 0) ? size.x * 0.35f : size.x * 0.18f;
        dl->AddLine({pos.x, y}, {pos.x + tickW, y}, IM_COL32(230,230,230,255), 2.f);
        if (v % 500 == 0) {
            snprintf(buf, sizeof(buf), "%d", v);
            dl->AddText({pos.x + tickW + 4.f, y - ImGui::CalcTextSize(buf).y*0.5f},
                        IM_COL32(230,230,230,255), buf);
        }
    }
    dl->PopClipRect();
    dl->AddRect(pos, pMax, IM_COL32(80,80,90,255), 4.f, 0, 2.f);

    char cur[8]; snprintf(cur, sizeof(cur), "%d", (int)std::lround(altFt));
    float boxW = size.x*0.72f;
    drawValueBox(dl, {pos.x + 2.f + boxW*0.5f, center.y}, boxW, 22.f, cur);
}

// ── Fita de V/S (fpm) — escala fixa ±2000, ponteiro linear ─────────────────
static void drawVsiStrip(ImDrawList* dl, ImVec2 pos, ImVec2 size, float vsFpm) {
    ImVec2 pMax{pos.x + size.x, pos.y + size.y};
    ImVec2 center{pos.x + size.x*0.5f, pos.y + size.y*0.5f};
    dl->AddRectFilled(pos, pMax, IM_COL32(15, 16, 20, 235));

    const float pxPerFpm = (size.y * 0.5f) / 2000.f;
    dl->AddLine({pos.x + 2.f, pos.y}, {pos.x + 2.f, pMax.y}, IM_COL32(120,120,130,255), 1.5f);
    for (int v = -2000; v <= 2000; v += 500) {
        float y = center.y - (float)v * pxPerFpm;
        float tickW = (v == 0) ? size.x * 0.85f : size.x * 0.45f;
        dl->AddLine({pos.x + 2.f, y}, {pos.x + 2.f + tickW, y},
                    IM_COL32(220,220,220,255), (v == 0) ? 2.f : 1.5f);
    }
    dl->AddRect(pos, pMax, IM_COL32(80,80,90,255), 4.f, 0, 2.f);

    // Ponteiro (triângulo saindo da borda esquerda, apontando p/ o valor)
    float vsClamped = std::clamp(vsFpm, -2000.f, 2000.f);
    float py = center.y - vsClamped * pxPerFpm;
    ImVec2 tip{pos.x + 2.f, py};
    dl->AddTriangleFilled(tip, {tip.x + 9.f, py - 6.f}, {tip.x + 9.f, py + 6.f},
                          IM_COL32(0, 230, 90, 255));
}

void drawAttitude(ImVec2 pos, ImVec2 size, float pitchDeg, float rollDeg, float betaDeg,
                  float fpaDeg, float driftDeg, bool showFpv) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 center(pos.x + size.x * 0.5f, pos.y + size.y * 0.5f);
    ImVec2 pMax(pos.x + size.x, pos.y + size.y);

    const float pxPerDeg = size.y / 40.f;      // ±20° de tela cheia
    const float rollRad  = rollDeg * PFD_D2R;
    const float cosR = cosf(rollRad), sinR = sinf(rollRad);
    const float pitchOffY = pitchDeg * pxPerDeg;   // deslocamento do horizonte
    const float BIG = 5000.f;                      // cobre qualquer rotação/pitch

    // Moldura escura de fundo (visível se o preenchimento não cobrir tudo)
    dl->AddRectFilled(pos, pMax, IM_COL32(15, 16, 20, 255));

    dl->PushClipRect(pos, pMax, true);

    // ── Céu / solo ────────────────────────────────────────────────────────────
    const ImU32 skyCol = IM_COL32(35, 88, 165, 255);
    const ImU32 gndCol = IM_COL32(115, 74, 38, 255);

    ImVec2 skyQuad[4] = {
        ballPoint(center, -BIG, -BIG,       cosR, sinR),
        ballPoint(center,  BIG, -BIG,       cosR, sinR),
        ballPoint(center,  BIG, pitchOffY,  cosR, sinR),
        ballPoint(center, -BIG, pitchOffY,  cosR, sinR),
    };
    dl->AddQuadFilled(skyQuad[0], skyQuad[1], skyQuad[2], skyQuad[3], skyCol);

    ImVec2 gndQuad[4] = {
        ballPoint(center, -BIG, pitchOffY, cosR, sinR),
        ballPoint(center,  BIG, pitchOffY, cosR, sinR),
        ballPoint(center,  BIG, BIG,       cosR, sinR),
        ballPoint(center, -BIG, BIG,       cosR, sinR),
    };
    dl->AddQuadFilled(gndQuad[0], gndQuad[1], gndQuad[2], gndQuad[3], gndCol);

    // ── Linha do horizonte ───────────────────────────────────────────────────
    ImVec2 hzL = ballPoint(center, -BIG, pitchOffY, cosR, sinR);
    ImVec2 hzR = ballPoint(center,  BIG, pitchOffY, cosR, sinR);
    dl->AddLine(hzL, hzR, IM_COL32(255, 255, 255, 255), 3.f);

    // ── Escada de pitch: linhas a cada 10° (rotuladas), 5° (curtas) e
    // 2.5° (marca central minúscula) — n em unidades de 2.5°, exceto 0=horizonte
    char lbl[8];
    for (int n = -34; n <= 34; ++n) {
        if (n == 0) continue;
        float p = (float)n * 2.5f;
        float y = pitchOffY - p * pxPerDeg;
        if (fabsf(y) > size.y * 1.3f) continue;   // fora da área útil

        ImU32 lineCol = IM_COL32(255, 255, 255, 235);
        if (n % 4 == 0) {
            // múltiplo de 10°: linha maior (30° = mais longa) + rótulo
            int p10 = (int)lroundf(p);
            bool major = (p10 % 30 == 0);
            float halfW = major ? size.x * 0.22f : size.x * 0.12f;
            ImVec2 l0 = ballPoint(center, -halfW, y, cosR, sinR);
            ImVec2 l1 = ballPoint(center,  halfW, y, cosR, sinR);
            dl->AddLine(l0, l1, lineCol, 2.f);

            snprintf(lbl, sizeof(lbl), "%d", p10 > 0 ? p10 : -p10);
            ImVec2 tR = ballPoint(center, halfW + 6.f, y, cosR, sinR);
            ImVec2 ts = ImGui::CalcTextSize(lbl);
            dl->AddText({tR.x, tR.y - ts.y * 0.5f}, lineCol, lbl);
        } else if (n % 2 == 0) {
            // múltiplo de 5° (não de 10°): linha curta, sem rótulo
            float halfW = size.x * 0.07f;
            ImVec2 l0 = ballPoint(center, -halfW, y, cosR, sinR);
            ImVec2 l1 = ballPoint(center,  halfW, y, cosR, sinR);
            dl->AddLine(l0, l1, lineCol, 1.5f);
        } else {
            // múltiplo de 2.5° (não de 5°): marca minúscula central
            float halfW = size.x * 0.03f;
            ImVec2 l0 = ballPoint(center, -halfW, y, cosR, sinR);
            ImVec2 l1 = ballPoint(center,  halfW, y, cosR, sinR);
            dl->AddLine(l0, l1, IM_COL32(255, 255, 255, 170), 1.f);
        }
    }

    // ── Escala de bank (arco fixo no topo + ponteiro preso à bola) ──────────
    const float bankR = (size.x < size.y ? size.x : size.y) * 0.44f;
    static const float BANK_TICKS[] = {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60};
    for (float a : BANK_TICKS) {
        float aRad = a * PFD_D2R;
        float sA = sinf(aRad), cA = -cosf(aRad);
        float r0 = bankR - ((a == 0.f) ? 14.f : 8.f);
        ImVec2 t0(center.x + r0     * sA, center.y + r0     * cA);
        ImVec2 t1(center.x + bankR  * sA, center.y + bankR  * cA);
        dl->AddLine(t0, t1, IM_COL32(230, 230, 230, 255), a == 0.f ? 3.f : 2.f);
    }
    // Ponteiro móvel: aponta para o LADO do bank (roll direita → ponteiro
    // direita — confirmado incorreto com sinR "preso à bola" e invertido
    // aqui por pedido; usa -rollRad, ou seja sinR trocado de sinal em relação
    // ao resto da bola (horizonte/escada/slip-skid, que continuam corretos).
    float ptrSinR = -sinR;
    ImVec2 ptrTip  = ballPoint(center, 0.f, -bankR,        cosR, ptrSinR);
    ImVec2 ptrBaseL= ballPoint(center, -7.f, -bankR + 14.f, cosR, ptrSinR);
    ImVec2 ptrBaseR= ballPoint(center,  7.f, -bankR + 14.f, cosR, ptrSinR);
    dl->AddTriangleFilled(ptrTip, ptrBaseL, ptrBaseR, IM_COL32(255, 210, 0, 255));

    // ── FPV (flight path vector): círculo + asinhas + cauda, preso à bola ────
    // Vertical = γ na mesma escala da escada de pitch; lateral = deriva
    // (track − heading). Asas paralelas ao horizonte (giram com a bola).
    if (showFpv) {
        const ImU32 fpvCol = IM_COL32(0, 230, 90, 255);
        float fx = driftDeg * pxPerDeg;
        float fy = pitchOffY - fpaDeg * pxPerDeg;
        const float r = 6.f, wing = 10.f, tail = 6.f;
        ImVec2 q = ballPoint(center, fx, fy, cosR, sinR);
        dl->AddCircle(q, r, fpvCol, 20, 2.f);
        ImVec2 wl0 = ballPoint(center, fx - r,        fy, cosR, sinR);
        ImVec2 wl1 = ballPoint(center, fx - r - wing, fy, cosR, sinR);
        ImVec2 wr0 = ballPoint(center, fx + r,        fy, cosR, sinR);
        ImVec2 wr1 = ballPoint(center, fx + r + wing, fy, cosR, sinR);
        ImVec2 t0  = ballPoint(center, fx, fy - r,        cosR, sinR);
        ImVec2 t1  = ballPoint(center, fx, fy - r - tail, cosR, sinR);
        dl->AddLine(wl0, wl1, fpvCol, 2.f);
        dl->AddLine(wr0, wr1, fpvCol, 2.f);
        dl->AddLine(t0,  t1,  fpvCol, 2.f);
    }

    // ── Slip/skid: trapézio numa trilha fixa logo abaixo do índice de bank ───
    // Não gira com o roll (fica preso ao case, como num ADI real) — desliza
    // lateralmente com beta. Mesma convenção de sinal do painel SUPERFICIES.
    {
        float trackY = center.y - (bankR - 14.f) + 20.f;
        float halfTrack = size.x * 0.09f;
        dl->AddLine({center.x - halfTrack - 10.f, trackY}, {center.x + halfTrack + 10.f, trackY},
                    IM_COL32(150,150,160,255), 1.5f);
        float bN = std::clamp(betaDeg / 10.f, -1.f, 1.f);
        float bx = center.x + bN * halfTrack;
        float half = 8.f, h = 9.f;
        ImVec2 tA{bx - half, trackY - h*0.5f}, tB{bx + half, trackY - h*0.5f};
        ImVec2 tC{bx + half*0.5f, trackY + h*0.5f}, tD{bx - half*0.5f, trackY + h*0.5f};
        dl->AddQuadFilled(tA, tB, tC, tD, IM_COL32(255,255,255,255));
    }

    dl->PopClipRect();

    // ── Símbolo fixo da aeronave (não gira nem desloca — referência do nariz) ──
    const float wingOuter = size.x * 0.30f, wingInner = size.x * 0.06f;
    const ImU32 acCol = IM_COL32(255, 200, 0, 255);
    dl->AddLine({center.x - wingOuter, center.y}, {center.x - wingInner, center.y}, acCol, 4.f);
    dl->AddLine({center.x + wingInner, center.y}, {center.x + wingOuter, center.y}, acCol, 4.f);
    dl->AddCircleFilled(center, 3.5f, acCol);

    // ── Moldura ──────────────────────────────────────────────────────────────
    dl->AddRect(pos, pMax, IM_COL32(80, 80, 90, 255), 4.f, 0, 2.f);
}

void drawPanel(ImVec2 pos, ImVec2 size,
               float pitchDeg, float rollDeg, float betaDeg,
               float speedKt, float altFt, float vsFpm,
               float fpaDeg, float driftDeg, bool showFpv) {
    ImDrawList* dl = ImGui::GetWindowDrawList();

    const float spdW = size.x * 0.155f;
    const float altW = size.x * 0.155f;
    const float vsW  = size.x * 0.075f;
    const float adiW = size.x - spdW - altW - vsW - 6.f;   // 6px de frestas (2px×3)

    ImVec2 spdPos = pos;
    ImVec2 adiPos{spdPos.x + spdW + 2.f, pos.y};
    ImVec2 altPos{adiPos.x + adiW + 2.f, pos.y};
    ImVec2 vsPos {altPos.x + altW + 2.f, pos.y};

    drawSpeedTape(dl, spdPos, {spdW, size.y}, speedKt);
    drawAttitude(adiPos, {adiW, size.y}, pitchDeg, rollDeg, betaDeg,
                 fpaDeg, driftDeg, showFpv);
    drawAltTape(dl, altPos, {altW, size.y}, altFt);
    drawVsiStrip(dl, vsPos, {vsW, size.y}, vsFpm);
}

}
