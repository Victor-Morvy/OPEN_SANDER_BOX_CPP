#include "MiniMap.h"
#include "TileManager.h"   // httpGet (curl compartilhado)
#include <glad/glad.h>
#include <stb_image.h>     // impl definida no TileManager.cpp
#include <cmath>
#include <cstdio>
#include <chrono>
#include <algorithm>

static constexpr double MM_PI  = 3.14159265358979323846;
static constexpr double MM_D2R = MM_PI / 180.0;
static constexpr int    MAX_TILES    = 220;  // cache LRU de texturas
static constexpr int    MAX_FETCHES  = 6;    // downloads simultâneos

// ── Web Mercator: lat/lon ↔ coordenada fracionária de tile ───────────────────
static void ll2tile(double lat, double lon, int z, double& tx, double& ty) {
    double n = (double)(1 << z);
    tx = (lon + 180.0) / 360.0 * n;
    double la = lat * MM_D2R;
    ty = (1.0 - std::asinh(std::tan(la)) / MM_PI) * 0.5 * n;
}
static void tile2ll(double tx, double ty, int z, double& lat, double& lon) {
    double n = (double)(1 << z);
    lon = tx / n * 360.0 - 180.0;
    lat = std::atan(std::sinh(MM_PI * (1.0 - 2.0 * ty / n))) / MM_D2R;
}
// metros por pixel de tela na latitude (tiles de 256 px)
static double metersPerPixel(double lat, int z) {
    return 40075016.686 * std::cos(lat * MM_D2R) / (256.0 * (double)(1 << z));
}

// ── Fetch / cache ─────────────────────────────────────────────────────────────

unsigned int MiniMap::lookupTile(int z, int x, int y) {
    auto it = _tiles.find(Key{z, x, y});
    if (it == _tiles.end()) return 0;
    it->second.lastUse = _frame;
    return it->second.tex;
}

unsigned int MiniMap::getTile(int z, int x, int y) {
    int n = 1 << z;
    if (x < 0 || y < 0 || x >= n || y >= n) return 0;
    Key k{z, x, y};
    auto it = _tiles.find(k);
    if (it != _tiles.end()) { it->second.lastUse = _frame; return it->second.tex; }
    if (_loading.count(k) || (int)_loading.size() >= MAX_FETCHES) return 0;

    _loading[k] = std::async(std::launch::async, [k]() {
        char url[128];
        snprintf(url, sizeof(url),
                 "https://tile.openstreetmap.org/%d/%d/%d.png", k.z, k.x, k.y);
        Img im; im.key = k;
        auto png = TileManager::httpGet(url);
        if (!png.empty()) {
            int w, h, ch;
            unsigned char* p = stbi_load_from_memory(png.data(), (int)png.size(),
                                                     &w, &h, &ch, 4);
            if (p) {
                im.ok = true; im.w = w; im.h = h;
                im.rgba.assign(p, p + (size_t)w * h * 4);
                stbi_image_free(p);
            }
        }
        return im;
    });
    return 0;
}

void MiniMap::processUploads() {
    _frame++;
    for (auto it = _loading.begin(); it != _loading.end();) {
        if (it->second.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            Img im = it->second.get();
            Tile t; t.lastUse = _frame;
            if (im.ok) {
                glGenTextures(1, &t.tex);
                glBindTexture(GL_TEXTURE_2D, t.tex);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, im.w, im.h, 0,
                             GL_RGBA, GL_UNSIGNED_BYTE, im.rgba.data());
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            }
            // tex=0 se o download falhou — fica registrado para não repetir o fetch
            _tiles[im.key] = t;
            it = _loading.erase(it);
        } else ++it;
    }
    while ((int)_tiles.size() > MAX_TILES) {
        auto oldest = _tiles.begin();
        for (auto it = _tiles.begin(); it != _tiles.end(); ++it)
            if (it->second.lastUse < oldest->second.lastUse) oldest = it;
        if (oldest->second.tex) glDeleteTextures(1, &oldest->second.tex);
        _tiles.erase(oldest);
    }
}

void MiniMap::cleanup() {
    for (auto& [k, f] : _loading) if (f.valid()) f.wait();
    _loading.clear();
    for (auto& [k, t] : _tiles) if (t.tex) glDeleteTextures(1, &t.tex);
    _tiles.clear();
}

// ── Desenho dos tiles (com fallback do ancestral enquanto carrega) ────────────

void MiniMap::drawTiles(ImDrawList* dl, ImVec2 center, double ctx, double cty,
                        int zoom, ImVec2 clipMin, ImVec2 clipMax, float rotRad) {
    float w = clipMax.x - clipMin.x, h = clipMax.y - clipMin.y;
    double rt = 0.5 * std::sqrt((double)w * w + (double)h * h) / 256.0 + 1.0;
    int n = 1 << zoom;
    float c = std::cos(rotRad), s = std::sin(rotRad);

    int x0 = (int)std::floor(ctx - rt), x1 = (int)std::floor(ctx + rt);
    int y0 = (int)std::floor(cty - rt), y1 = (int)std::floor(cty + rt);
    for (int ty = y0; ty <= y1; ++ty) {
        if (ty < 0 || ty >= n) continue;
        for (int tx = x0; tx <= x1; ++tx) {
            int wx = ((tx % n) + n) % n;   // wrap no antimeridiano
            unsigned int tex = getTile(zoom, wx, ty);
            float u0 = 0, v0 = 0, u1 = 1, v1 = 1;
            if (!tex) {
                for (int up = 1; up <= 4; ++up) {
                    int pz = zoom - up; if (pz < 3) break;
                    unsigned int pt = lookupTile(pz, wx >> up, ty >> up);
                    if (pt) {
                        float f = 1.f / (float)(1 << up);
                        u0 = (wx - ((wx >> up) << up)) * f;
                        v0 = (ty - ((ty >> up) << up)) * f;
                        u1 = u0 + f; v1 = v0 + f;
                        tex = pt; break;
                    }
                }
            }
            if (!tex) continue;
            float ox = (float)((tx - ctx) * 256.0), oy = (float)((ty - cty) * 256.0);
            const float qx[4] = { ox, ox + 256.f, ox + 256.f, ox };
            const float qy[4] = { oy, oy,         oy + 256.f, oy + 256.f };
            ImVec2 p[4];
            for (int i = 0; i < 4; ++i)
                p[i] = ImVec2(center.x + qx[i] * c - qy[i] * s,
                              center.y + qx[i] * s + qy[i] * c);
            dl->AddImageQuad((ImTextureID)(intptr_t)tex, p[0], p[1], p[2], p[3],
                             ImVec2(u0, v0), ImVec2(u1, v0),
                             ImVec2(u1, v1), ImVec2(u0, v1), IM_COL32_WHITE);
        }
    }
}

// ── Ícones ────────────────────────────────────────────────────────────────────

// seta de avião apontando na direção `ang` (rad, 0 = para cima na tela)
static void drawAircraftIcon(ImDrawList* dl, ImVec2 pos, float ang, float sz,
                             ImU32 fill) {
    float c = std::cos(ang), s = std::sin(ang);
    auto rot = [&](float x, float y) {
        return ImVec2(pos.x + x * c - y * s, pos.y + x * s + y * c);
    };
    ImVec2 pts[4] = {
        rot(0.f, -1.1f * sz),   // nariz
        rot(0.8f * sz,  0.9f * sz),
        rot(0.f,  0.45f * sz),  // entalhe da cauda
        rot(-0.8f * sz, 0.9f * sz),
    };
    // forma côncava (entalhe) → preenche com dois triângulos convexos
    dl->AddTriangleFilled(pts[0], pts[1], pts[2], fill);
    dl->AddTriangleFilled(pts[0], pts[2], pts[3], fill);
    dl->AddPolyline(pts, 4, IM_COL32(0, 0, 0, 220), ImDrawFlags_Closed, 1.5f);
}

// ── HSD (heading-up) ──────────────────────────────────────────────────────────

void MiniMap::drawHSD(ImVec2 size, double lat, double lon, float hdgDeg) {
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 p1 = ImVec2(p0.x + size.x, p0.y + size.y);
    ImGui::InvisibleButton("##hsd", size);
    if (ImGui::IsItemHovered()) {
        float wheel = ImGui::GetIO().MouseWheel;
        if (wheel > 0.f && hsdZoom < 16) hsdZoom++;
        if (wheel < 0.f && hsdZoom >  9) hsdZoom--;
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 center = ImVec2((p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f);
    float rot = -hdgDeg * (float)MM_D2R;

    dl->PushClipRect(p0, p1, true);
    dl->AddRectFilled(p0, p1, IM_COL32(12, 16, 20, 235), 6.f);

    double ctx, cty; ll2tile(lat, lon, hsdZoom, ctx, cty);
    drawTiles(dl, center, ctx, cty, hsdZoom, p0, p1, rot);

    // anel de alcance + marcador N girando com o mapa
    float rr = 0.5f * std::min(size.x, size.y) - 26.f;
    dl->AddCircle(center, rr, IM_COL32(255, 255, 255, 110), 64, 1.5f);
    {
        ImVec2 npos = ImVec2(center.x + rr * std::sin(rot),
                             center.y - rr * std::cos(rot));
        dl->AddCircleFilled(npos, 9.f, IM_COL32(15, 25, 35, 230));
        ImVec2 ts = ImGui::CalcTextSize("N");
        dl->AddText(ImVec2(npos.x - ts.x * 0.5f, npos.y - ts.y * 0.5f),
                    IM_COL32(255, 210, 60, 255), "N");
    }

    // alcance do anel (m/px na latitude atual)
    char buf[48];
    double rangeM = rr * metersPerPixel(lat, hsdZoom);
    if (rangeM >= 1000.0) snprintf(buf, sizeof(buf), "%.0f km", rangeM / 1000.0);
    else                  snprintf(buf, sizeof(buf), "%.0f m", rangeM);
    dl->AddText(ImVec2(p0.x + 6.f, p1.y - 20.f), IM_COL32(255, 255, 255, 200), buf);

    // proa no topo
    snprintf(buf, sizeof(buf), "%03d\xC2\xB0", ((int)std::lround(hdgDeg) % 360 + 360) % 360);
    {
        ImVec2 ts = ImGui::CalcTextSize(buf);
        ImVec2 tp = ImVec2(center.x - ts.x * 0.5f, p0.y + 4.f);
        dl->AddRectFilled(ImVec2(tp.x - 5.f, tp.y - 2.f),
                          ImVec2(tp.x + ts.x + 5.f, tp.y + ts.y + 2.f),
                          IM_COL32(10, 14, 18, 210), 3.f);
        dl->AddText(tp, IM_COL32(120, 255, 120, 255), buf);
        // linha de rumo do topo do anel até o avião
        dl->AddLine(ImVec2(center.x, center.y - rr), ImVec2(center.x, tp.y + ts.y + 3.f),
                    IM_COL32(120, 255, 120, 90), 1.f);
    }

    drawAircraftIcon(dl, center, 0.f, 10.f, IM_COL32(255, 255, 255, 255));

    dl->AddText(ImVec2(p1.x - 96.f, p1.y - 20.f),
                IM_COL32(255, 255, 255, 130), "\xC2\xA9 OpenStreetMap");
    dl->PopClipRect();
    dl->AddRect(p0, p1, IM_COL32(255, 255, 255, 70), 6.f, 0, 1.5f);
}

// ── Picker (north-up, menu de pausa) ──────────────────────────────────────────

void MiniMap::centerPickerOn(double lat, double lon) {
    _pLat = lat; _pLon = lon; _pInit = true;
}

bool MiniMap::drawPicker(ImVec2 size, double& selLat, double& selLon,
                         double acLat, double acLon, float acHdgDeg,
                         const std::vector<Poi>* pois) {
    if (!_pInit) centerPickerOn(acLat, acLon);

    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 p1 = ImVec2(p0.x + size.x, p0.y + size.y);
    ImVec2 center = ImVec2((p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f);
    ImGuiIO& io = ImGui::GetIO();

    ImGui::InvisibleButton("##picker", size);
    bool hovered = ImGui::IsItemHovered();
    bool clicked = false;

    double ctx, cty; ll2tile(_pLat, _pLon, _pZoom, ctx, cty);

    // pan (arrastar com botão esquerdo)
    if (ImGui::IsItemActive() && ImGui::IsMouseDragging(0, 2.f)) {
        ctx -= io.MouseDelta.x / 256.0;
        cty -= io.MouseDelta.y / 256.0;
        tile2ll(ctx, cty, _pZoom, _pLat, _pLon);
    }

    // zoom com scroll, ancorado no cursor
    if (hovered && io.MouseWheel != 0.f) {
        int nz = _pZoom + (io.MouseWheel > 0.f ? 1 : -1);
        nz = std::max(3, std::min(17, nz));
        if (nz != _pZoom) {
            double mx = (io.MousePos.x - center.x) / 256.0;
            double my = (io.MousePos.y - center.y) / 256.0;
            double scale = std::pow(2.0, nz - _pZoom);
            double nctx = (ctx + mx) * scale - mx;
            double ncty = (cty + my) * scale - my;
            _pZoom = nz;
            tile2ll(nctx, ncty, _pZoom, _pLat, _pLon);
            ll2tile(_pLat, _pLon, _pZoom, ctx, cty);
        }
    }

    // clique (soltar sem ter arrastado) → define o destino
    if (ImGui::IsItemDeactivated()) {
        float dx = io.MousePos.x - io.MouseClickedPos[0].x;
        float dy = io.MousePos.y - io.MouseClickedPos[0].y;
        if (dx * dx + dy * dy < 25.f && hovered) {
            double stx = ctx + (io.MousePos.x - center.x) / 256.0;
            double sty = cty + (io.MousePos.y - center.y) / 256.0;
            tile2ll(stx, sty, _pZoom, selLat, selLon);
            clicked = true;
        }
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(p0, p1, true);
    dl->AddRectFilled(p0, p1, IM_COL32(12, 16, 20, 255), 4.f);
    drawTiles(dl, center, ctx, cty, _pZoom, p0, p1, 0.f);

    // helper lat/lon → tela
    auto toScreen = [&](double la, double lo) {
        double tx, ty; ll2tile(la, lo, _pZoom, tx, ty);
        return ImVec2(center.x + (float)((tx - ctx) * 256.0),
                      center.y + (float)((ty - cty) * 256.0));
    };

    // aeroportos próximos
    if (pois && _pZoom >= 8) {
        for (const auto& poi : *pois) {
            ImVec2 sp = toScreen(poi.lat, poi.lon);
            if (sp.x < p0.x - 40 || sp.x > p1.x + 40 ||
                sp.y < p0.y - 20 || sp.y > p1.y + 20) continue;
            dl->AddCircleFilled(sp, 3.5f, IM_COL32(60, 190, 255, 255));
            dl->AddCircle(sp, 3.5f, IM_COL32(0, 0, 0, 200), 0, 1.f);
            dl->AddText(ImVec2(sp.x + 6.f, sp.y - 7.f),
                        IM_COL32(60, 190, 255, 255), poi.label.c_str());
        }
    }

    // avião (posição atual)
    {
        ImVec2 ap = toScreen(acLat, acLon);
        drawAircraftIcon(dl, ap, acHdgDeg * (float)MM_D2R, 9.f,
                         IM_COL32(255, 255, 255, 255));
    }

    // marcador do destino selecionado (pino laranja)
    {
        ImVec2 mp = toScreen(selLat, selLon);
        dl->AddLine(ImVec2(mp.x, mp.y - 14.f), mp, IM_COL32(255, 140, 20, 255), 2.5f);
        dl->AddCircleFilled(ImVec2(mp.x, mp.y - 17.f), 5.5f, IM_COL32(255, 140, 20, 255));
        dl->AddCircle(ImVec2(mp.x, mp.y - 17.f), 5.5f, IM_COL32(0, 0, 0, 200), 0, 1.5f);
        dl->AddCircleFilled(mp, 2.f, IM_COL32(0, 0, 0, 200));
    }

    // escala + atribuição
    {
        char buf[48];
        double barM = 100.0 * metersPerPixel(_pLat, _pZoom);
        if (barM >= 1000.0) snprintf(buf, sizeof(buf), "100 px = %.1f km", barM / 1000.0);
        else                snprintf(buf, sizeof(buf), "100 px = %.0f m", barM);
        dl->AddText(ImVec2(p0.x + 6.f, p1.y - 20.f), IM_COL32(255, 255, 255, 200), buf);
        dl->AddText(ImVec2(p1.x - 96.f, p1.y - 20.f),
                    IM_COL32(255, 255, 255, 140), "\xC2\xA9 OpenStreetMap");
    }

    dl->PopClipRect();
    dl->AddRect(p0, p1, IM_COL32(255, 255, 255, 60), 4.f);
    return clicked;
}
