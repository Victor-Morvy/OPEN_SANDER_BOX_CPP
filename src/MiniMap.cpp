#define IMGUI_DEFINE_MATH_OPERATORS
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
// proa verdadeira A→B [°]
static double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
    double dLon = (lon2 - lon1) * MM_D2R;
    double la1 = lat1 * MM_D2R, la2 = lat2 * MM_D2R;
    double y = std::sin(dLon) * std::cos(la2);
    double x = std::cos(la1) * std::sin(la2)
             - std::sin(la1) * std::cos(la2) * std::cos(dLon);
    return std::fmod(std::atan2(y, x) / MM_D2R + 360.0, 360.0);
}
static double geoDistM(double la1, double lo1, double la2, double lo2) {
    double x = (lo2 - lo1) * MM_D2R * std::cos(0.5 * (la1 + la2) * MM_D2R) * 6371000.0;
    double y = (la2 - la1) * MM_D2R * 6371000.0;
    return std::sqrt(x * x + y * y);
}

// transformação lat/lon → tela dos dois mapas (com rotação opcional do HSD)
namespace {
struct Xform {
    ImVec2 center; double ctx, cty; int zoom; float c = 1.f, s = 0.f;
    ImVec2 pt(double lat, double lon) const {
        double tx, ty; ll2tile(lat, lon, zoom, tx, ty);
        float ox = (float)((tx - ctx) * 256.0), oy = (float)((ty - cty) * 256.0);
        return ImVec2(center.x + ox * c - oy * s, center.y + ox * s + oy * c);
    }
};
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

// ── Overlays: espinha de peixe, pistas, POIs ─────────────────────────────────

// Espinha de peixe da aproximação: tracejado central estendido a partir da
// cabeceira, costelas a cada 1 NM com rótulo de distância e os marker
// beacons OM (~3.9 NM) / MM (~0.6 NM).
static void drawFishbone(ImDrawList* dl, ImVec2 origin, ImVec2 dir, double mpp) {
    float nmPx = (float)(1852.0 / mpp);
    if (nmPx < 3.f) return;                    // longe demais — vira ruído
    ImVec2 perp(-dir.y, dir.x);
    const int maxNM = 10;
    const ImU32 col = IM_COL32(255, 255, 255, 190);

    // espinha tracejada (0.25 NM traço / 0.25 NM vão)
    for (float d0 = 0.f; d0 < (float)maxNM; d0 += 0.5f)
        dl->AddLine(origin + dir * (d0 * nmPx),
                    origin + dir * ((d0 + 0.25f) * nmPx), col, 1.5f);

    // costelas + distância [NM]
    int labelStep = nmPx > 30.f ? 1 : nmPx > 15.f ? 2 : 5;
    char buf[8];
    for (int nm = 1; nm <= maxNM; ++nm) {
        ImVec2 c = origin + dir * ((float)nm * nmPx);
        float ribL = (nm % 5 == 0) ? 7.f : 4.5f;
        dl->AddLine(c - perp * ribL, c + perp * ribL, col, 1.5f);
        if (nm % labelStep == 0) {
            snprintf(buf, sizeof(buf), "%d", nm);
            dl->AddText(c + perp * (ribL + 3.f) - ImVec2(4.f, 7.f),
                        IM_COL32(255, 255, 255, 210), buf);
        }
    }

    // marker beacons na aproximação
    if (nmPx > 8.f) {
        auto beacon = [&](float nm, ImU32 bc, const char* tag) {
            ImVec2 c = origin + dir * (nm * nmPx);
            dl->AddCircleFilled(c, 3.5f, bc);
            dl->AddCircle(c, 3.5f, IM_COL32(0, 0, 0, 200), 0, 1.f);
            if (nmPx > 20.f) dl->AddText(c - perp * 16.f - ImVec2(6.f, 7.f), bc, tag);
        };
        beacon(3.9f, IM_COL32( 80, 200, 255, 255), "OM");
        beacon(0.6f, IM_COL32(255, 200,  60, 255), "MM");
    }
}

static void drawRunwaysOverlay(ImDrawList* dl, const Xform& xf,
                               const std::vector<MiniMap::Rwy>& rwys,
                               double mpp, ImVec2 p0, ImVec2 p1,
                               bool fishbone, bool idents) {
    float margin = fishbone ? (float)(10.0 * 1852.0 / mpp) : 40.f;
    for (const auto& r : rwys) {
        ImVec2 a = xf.pt(r.leLat, r.leLon);
        ImVec2 b = xf.pt(r.heLat, r.heLon);
        if (std::max(a.x, b.x) < p0.x - margin || std::min(a.x, b.x) > p1.x + margin ||
            std::max(a.y, b.y) < p0.y - margin || std::min(a.y, b.y) > p1.y + margin)
            continue;
        float dx = b.x - a.x, dy = b.y - a.y;
        float len = std::sqrt(dx * dx + dy * dy);
        float thick = std::max(3.f, (float)(r.widthM / mpp));
        dl->AddLine(a, b, IM_COL32(250, 250, 250, 235), thick + 2.f);
        dl->AddLine(a, b, IM_COL32(40, 42, 50, 255), thick);
        if (len < 1e-3f) continue;
        ImVec2 dir(dx / len, dy / len);

        // aproximações (só para pistas relevantes — evita clutter de aeroclube)
        if (fishbone && geoDistM(r.leLat, r.leLon, r.heLat, r.heLon) > 900.0) {
            drawFishbone(dl, a, ImVec2(-dir.x, -dir.y), mpp);
            drawFishbone(dl, b, dir, mpp);
        }

        if (idents && len > 30.f) {
            auto endLabel = [&](ImVec2 endPt, ImVec2 outDir, const std::string& id) {
                if (id.empty()) return;
                ImVec2 ts = ImGui::CalcTextSize(id.c_str());
                ImVec2 tp = endPt + outDir * 14.f - ts * 0.5f;
                dl->AddRectFilled(tp - ImVec2(2.f, 1.f), tp + ts + ImVec2(2.f, 1.f),
                                  IM_COL32(10, 14, 18, 190), 2.f);
                dl->AddText(tp, IM_COL32(255, 255, 255, 235), id.c_str());
            };
            endLabel(a, ImVec2(-dir.x, -dir.y), r.leIdent);
            endLabel(b, dir, r.heIdent);
        }
    }
}

// Polilinha com halo preto por baixo — legível sobre qualquer cor de tile
// (satélite claro, água escura, floresta verde etc.)
static void drawOutlinedPolyline(ImDrawList* dl, const Xform& xf,
                                 const std::vector<MiniMap::PathPt>& pts,
                                 ImU32 col, float thick) {
    if (pts.size() < 2) return;
    std::vector<ImVec2> sp(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) sp[i] = xf.pt(pts[i].lat, pts[i].lon);
    dl->AddPolyline(sp.data(), (int)sp.size(), IM_COL32(0, 0, 0, 210), 0, thick + 2.5f);
    dl->AddPolyline(sp.data(), (int)sp.size(), col, 0, thick);
}

static void drawPoisOverlay(ImDrawList* dl, const Xform& xf,
                            const std::vector<MiniMap::Poi>& pois,
                            ImVec2 p0, ImVec2 p1) {
    for (const auto& poi : pois) {
        ImVec2 sp = xf.pt(poi.lat, poi.lon);
        if (sp.x < p0.x - 40 || sp.x > p1.x + 40 ||
            sp.y < p0.y - 20 || sp.y > p1.y + 20) continue;
        ImU32 col;
        switch (poi.type) {
            case 1: {  // VOR — hexágono magenta
                col = IM_COL32(210, 70, 210, 255);
                ImVec2 hex[6];
                for (int i = 0; i < 6; ++i) {
                    float a = (float)(MM_PI / 3.0) * i + (float)(MM_PI / 6.0);
                    hex[i] = sp + ImVec2(std::cos(a), std::sin(a)) * 5.5f;
                }
                dl->AddPolyline(hex, 6, col, ImDrawFlags_Closed, 1.6f);
                dl->AddCircleFilled(sp, 1.6f, col);
                break;
            }
            case 2:    // NDB — círculo duplo laranja
                col = IM_COL32(225, 150, 40, 255);
                dl->AddCircle(sp, 5.5f, col, 0, 1.4f);
                dl->AddCircleFilled(sp, 2.f, col);
                break;
            case 3:    // DME — quadrado magenta
                col = IM_COL32(210, 70, 210, 255);
                dl->AddRect(sp - ImVec2(4.5f, 4.5f), sp + ImVec2(4.5f, 4.5f),
                            col, 0.f, 0, 1.6f);
                dl->AddCircleFilled(sp, 1.6f, col);
                break;
            default:   // aeroporto — círculo azul
                col = IM_COL32(60, 190, 255, 255);
                dl->AddCircleFilled(sp, 3.5f, col);
                dl->AddCircle(sp, 3.5f, IM_COL32(0, 0, 0, 200), 0, 1.f);
                break;
        }
        if (!poi.label.empty())
            dl->AddText(ImVec2(sp.x + 7.f, sp.y - 7.f), col, poi.label.c_str());
    }
}

// ── HSD (heading-up) ──────────────────────────────────────────────────────────

void MiniMap::drawHSD(ImVec2 size, double lat, double lon, float hdgDeg,
                      const std::vector<Poi>* pois,
                      const std::vector<Rwy>* rwys,
                      const std::vector<PathPt>* pred,
                      const std::vector<PathPt>* guidancePath,
                      const std::vector<PathPt>* route,
                      const std::vector<PathPt>* editWpts,
                      RouteEdit* editOut) {
    if (editOut) *editOut = RouteEdit{};
    else _hsdDragIdx = -1;   // mapa recolhido no meio de um arrasto → cancela

    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 p1 = ImVec2(p0.x + size.x, p0.y + size.y);
    ImGui::InvisibleButton("##hsd", size);
    ImGuiIO& io = ImGui::GetIO();
    bool hovered = ImGui::IsItemHovered();
    if (hovered) {
        float wheel = io.MouseWheel;
        if (wheel > 0.f && hsdZoom < 16) hsdZoom++;
        if (wheel < 0.f && hsdZoom >  5) hsdZoom--;
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 center = ImVec2((p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f);
    float rot = hsdNorthUp ? 0.f : -hdgDeg * (float)MM_D2R;

    // Pan (arrastar com botão esquerdo) — só no HSD expandido (editOut !=
    // nullptr): dá pra ver aeroportos longe do avião e preparar uma rota de
    // pouso sem perder a posição real — o avião continua desenhado na
    // lat/lon verdadeira, só a VISTA descola (ver drawAircraftIcon abaixo).
    if (editOut) {
        if (!_hsdPanned) { _hsdViewLat = lat; _hsdViewLon = lon; }  // segue o avião até o 1º arrasto
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 2.f)) {
            _hsdPanned = true;
            double vtx, vty; ll2tile(_hsdViewLat, _hsdViewLon, hsdZoom, vtx, vty);
            // delta de tela → delta de tile, desrotacionado pelo heading
            // (mesmo inverso usado no clique direito — ver screenToLatLon)
            double c = std::cos((double)rot), s = std::sin((double)rot);
            double dOx =  c * io.MouseDelta.x + s * io.MouseDelta.y;
            double dOy = -s * io.MouseDelta.x + c * io.MouseDelta.y;
            vtx -= dOx / 256.0;
            vty -= dOy / 256.0;
            tile2ll(vtx, vty, hsdZoom, _hsdViewLat, _hsdViewLon);
        }
    } else {
        _hsdPanned = false;   // mapa recolhido → volta a seguir o avião
    }
    double viewLat = _hsdPanned ? _hsdViewLat : lat;
    double viewLon = _hsdPanned ? _hsdViewLon : lon;

    dl->PushClipRect(p0, p1, true);
    dl->AddRectFilled(p0, p1, IM_COL32(12, 16, 20, 235), 6.f);

    double ctx, cty; ll2tile(viewLat, viewLon, hsdZoom, ctx, cty);
    // xf em escopo de função (não só do bloco de overlays) — precisa dele lá
    // embaixo pra achar a posição de tela do avião de verdade, que só
    // coincide com `center` quando a vista NÃO está deslocada (ver _hsdPanned).
    Xform xf{center, ctx, cty, hsdZoom, std::cos(rot), std::sin(rot)};
    if (hsdShowTiles)
        drawTiles(dl, center, ctx, cty, hsdZoom, p0, p1, rot);
    // hsdShowTiles=false: sem dado vetorial de land/water no pipeline (só
    // busca tiles raster), então o "fundo liso" acima já É o fallback —
    // deixa a espinha de peixe/pistas/waypoints (brancos) bem mais legíveis
    // sem a textura do mapa por baixo.

    // overlays de navegação (posições giram com o mapa; textos ficam retos)
    {
        double mpp = metersPerPixel(viewLat, hsdZoom);

        // ── Edição de rota por clique direito (só quando editOut != nullptr,
        // isto é, HSD expandido — ver drawHSD/hsdExpanded) ─────────────────
        if (editOut) {
            bool rClickStart = hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right);
            if (rClickStart) {
                // acha o waypoint mais próximo do clique (raio de captura 16 px)
                _hsdDragIdx = -1;
                if (editWpts) {
                    float bestD2 = 16.f * 16.f;
                    for (size_t i = 0; i < editWpts->size(); ++i) {
                        ImVec2 sp = xf.pt((*editWpts)[i].lat, (*editWpts)[i].lon);
                        float dx = io.MousePos.x - sp.x, dy = io.MousePos.y - sp.y;
                        float d2 = dx * dx + dy * dy;
                        if (d2 < bestD2) { bestD2 = d2; _hsdDragIdx = (int)i; }
                    }
                }
            }

            // Inverso de Xform::pt (tela → tile → lat/lon) — mesma rotação.
            auto screenToLatLon = [&](ImVec2 sp, double& oLat, double& oLon) {
                double dx = sp.x - center.x, dy = sp.y - center.y;
                double c = std::cos((double)rot), s = std::sin((double)rot);
                double ox =  c * dx + s * dy;
                double oy = -s * dx + c * dy;
                tile2ll(ctx + ox / 256.0, cty + oy / 256.0, hsdZoom, oLat, oLon);
            };

            if (_hsdDragIdx >= 0 && io.MouseDown[ImGuiMouseButton_Right]) {
                // arrastando um waypoint existente → atualiza a posição dele
                screenToLatLon(io.MousePos, editOut->dragLat, editOut->dragLon);
                editOut->dragIdx = _hsdDragIdx;
            } else if (rClickStart && _hsdDragIdx < 0) {
                // clique direito longe de qualquer waypoint → adiciona um novo
                screenToLatLon(io.MousePos, editOut->addLat, editOut->addLon);
                editOut->added = true;
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) _hsdDragIdx = -1;
        }

        // marcadores dos waypoints editáveis (só desenhados quando passados —
        // isto é, HSD expandido; o rosa da polilinha `route` já cobre o modo
        // pequeno, aqui é só o "alvo" clicável em destaque)
        if (editWpts) {
            for (size_t i = 0; i < editWpts->size(); ++i) {
                ImVec2 sp = xf.pt((*editWpts)[i].lat, (*editWpts)[i].lon);
                bool dragging = (int)i == _hsdDragIdx;
                float r = dragging ? 6.f : 4.5f;
                ImU32 col = dragging ? IM_COL32(255, 230, 60, 255) : IM_COL32(255, 90, 220, 255);
                dl->AddCircleFilled(sp, r, col);
                dl->AddCircle(sp, r, IM_COL32(0, 0, 0, 200), 0, 1.5f);
                const std::string& nm = (*editWpts)[i].name;
                if (!nm.empty()) {
                    ImVec2 tp(sp.x + r + 3.f, sp.y - 7.f);
                    dl->AddText(ImVec2(tp.x + 1, tp.y + 1), IM_COL32(0, 0, 0, 220), nm.c_str());
                    dl->AddText(tp, IM_COL32(255, 230, 60, 255), nm.c_str());
                }
            }
        }

        if (rwys) drawRunwaysOverlay(dl, xf, *rwys, mpp, p0, p1,
                                     /*fishbone=*/hsdZoom >= 9,
                                     /*idents=*/hsdZoom >= 12);
        if (pois) drawPoisOverlay(dl, xf, *pois, p0, p1);

        // Rota do plano de voo: liga os waypoints em rosa (referência
        // geométrica do plano, sem curvatura — desenhada ANTES das curvas
        // pra ficar por baixo delas).
        if (route && route->size() >= 2)
            drawOutlinedPolyline(dl, xf, *route, IM_COL32(255, 90, 220, 235), 2.f);

        // Trend vector: curva prevista (GS + turn rate atual), verde, com
        // ponto cheio na extremidade. Gira com o mapa como os demais overlays.
        if (pred && pred->size() >= 2) {
            const ImU32 col = IM_COL32(120, 255, 120, 230);
            drawOutlinedPolyline(dl, xf, *pred, col, 2.5f);
            dl->AddCircleFilled(xf.pt(pred->back().lat, pred->back().lon), 3.5f, col);
        }

        // Guidance path: curva que o AP vai voar até capturar HDG/NAV
        // (simulada pelo chamador com o mesmo ganho do GuidanceModule),
        // amarelo — distinta do trend vector (verde, é só física atual).
        if (guidancePath && guidancePath->size() >= 2) {
            const ImU32 col = IM_COL32(255, 210, 40, 235);
            drawOutlinedPolyline(dl, xf, *guidancePath, col, 2.5f);
            dl->AddCircleFilled(xf.pt(guidancePath->back().lat, guidancePath->back().lon), 3.5f, col);
        }
    }

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
    double rangeM = rr * metersPerPixel(viewLat, hsdZoom);
    if (rangeM >= 1000.0) snprintf(buf, sizeof(buf), "%.0f km", rangeM / 1000.0);
    else                  snprintf(buf, sizeof(buf), "%.0f m", rangeM);
    dl->AddText(ImVec2(p0.x + 6.f, p1.y - 20.f), IM_COL32(255, 255, 255, 200), buf);

    // Marcador de proa (lubber): desliza no anel na posição onde a proa
    // atual cai na carta — ângulo de tela = hdgDeg + rot, igual ao usado pro
    // "N" acima (bearing 0). Em HDG-UP, rot=-hdgDeg cancela e fica sempre no
    // topo (comportamento de sempre); em N-UP, rot=0 e ele gira em volta do
    // anel conforme a proa muda — igual um HSI real em modo N UP.
    {
        float hdgAng = hdgDeg * (float)MM_D2R + rot;
        ImVec2 hp = ImVec2(center.x + rr * std::sin(hdgAng), center.y - rr * std::cos(hdgAng));
        snprintf(buf, sizeof(buf), "%03d\xC2\xB0", ((int)std::lround(hdgDeg) % 360 + 360) % 360);
        ImVec2 ts = ImGui::CalcTextSize(buf);
        ImVec2 tp = ImVec2(hp.x - ts.x * 0.5f, hp.y - ts.y * 0.5f);
        dl->AddRectFilled(ImVec2(tp.x - 5.f, tp.y - 2.f),
                          ImVec2(tp.x + ts.x + 5.f, tp.y + ts.y + 2.f),
                          IM_COL32(10, 14, 18, 210), 3.f);
        dl->AddText(tp, IM_COL32(120, 255, 120, 255), buf);
    }

    // Posição real do avião — só coincide com `center` quando a vista não
    // está deslocada (_hsdPanned=false). Ângulo de tela = hdgDeg + rot: em
    // HDG-UP cancela e o ícone fica sempre apontando pra cima (é o MAPA que
    // gira); em N-UP o mapa fica fixo e é o ÍCONE que gira mostrando a proa
    // real, como no HSI.
    drawAircraftIcon(dl, xf.pt(lat, lon), hdgDeg * (float)MM_D2R + rot, 10.f, IM_COL32(255, 255, 255, 255));

    if (hsdShowTiles)
        dl->AddText(ImVec2(p1.x - 96.f, p1.y - 20.f),
                    IM_COL32(255, 255, 255, 130), "\xC2\xA9 OpenStreetMap");

    // Botão MAPA/LISO: esconde os tiles OSM, deixando só o fundo liso —
    // separa melhor o branco da espinha de peixe/pistas/waypoints do resto.
    {
        const char* lbl = hsdShowTiles ? "MAPA" : "LISO";
        ImVec2 bp0{p0.x + 4.f, p0.y + 4.f};
        ImVec2 bts = ImGui::CalcTextSize(lbl);
        ImVec2 bp1{bp0.x + bts.x + 10.f, bp0.y + bts.y + 6.f};
        bool hov = io.MousePos.x >= bp0.x && io.MousePos.x <= bp1.x &&
                   io.MousePos.y >= bp0.y && io.MousePos.y <= bp1.y;
        if (hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            hsdShowTiles = !hsdShowTiles;
        dl->AddRectFilled(bp0, bp1,
                          hov ? IM_COL32(60, 65, 75, 230) : IM_COL32(25, 28, 34, 210), 3.f);
        dl->AddRect(bp0, bp1, IM_COL32(255, 255, 255, 90), 3.f);
        dl->AddText({bp0.x + 5.f, bp0.y + 3.f}, IM_COL32(230, 230, 230, 255), lbl);

        // Botão N UP/HDG UP, empilhado logo abaixo — trava a orientação do
        // HSD (ver hsdNorthUp).
        const char* olbl = hsdNorthUp ? "N UP" : "HDG UP";
        ImVec2 obp0{bp0.x, bp1.y + 4.f};
        ImVec2 ots = ImGui::CalcTextSize(olbl);
        ImVec2 obp1{obp0.x + ots.x + 10.f, obp0.y + ots.y + 6.f};
        bool ohov = io.MousePos.x >= obp0.x && io.MousePos.x <= obp1.x &&
                    io.MousePos.y >= obp0.y && io.MousePos.y <= obp1.y;
        if (ohov && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            hsdNorthUp = !hsdNorthUp;
        dl->AddRectFilled(obp0, obp1,
                          ohov ? IM_COL32(60, 65, 75, 230) : IM_COL32(25, 28, 34, 210), 3.f);
        dl->AddRect(obp0, obp1, IM_COL32(255, 255, 255, 90), 3.f);
        dl->AddText({obp0.x + 5.f, obp0.y + 3.f}, IM_COL32(230, 230, 230, 255), olbl);
    }

    // Botão EXPANDIR/REDUZIR (topo-direita): quem chama é quem de fato muda
    // o `size` passado no próximo frame (ver hsdExpanded) — habilita também
    // a edição de rota por clique direito (main.cpp só passa editWpts/editOut
    // quando hsdExpanded). Dica de uso fica logo abaixo, só quando expandido.
    {
        const char* lbl = hsdExpanded ? "REDUZIR" : "EXPANDIR";
        ImVec2 bts = ImGui::CalcTextSize(lbl);
        ImVec2 bp1{p1.x - 4.f, p0.y + 4.f + bts.y + 6.f};
        ImVec2 bp0{bp1.x - bts.x - 10.f, p0.y + 4.f};
        bool hov = io.MousePos.x >= bp0.x && io.MousePos.x <= bp1.x &&
                   io.MousePos.y >= bp0.y && io.MousePos.y <= bp1.y;
        if (hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            hsdExpanded = !hsdExpanded;
        dl->AddRectFilled(bp0, bp1,
                          hov ? IM_COL32(60, 65, 75, 230) : IM_COL32(25, 28, 34, 210), 3.f);
        dl->AddRect(bp0, bp1, IM_COL32(255, 255, 255, 90), 3.f);
        dl->AddText({bp0.x + 5.f, bp0.y + 3.f}, IM_COL32(230, 230, 230, 255), lbl);

        float nextY = bp1.y;
        if (editOut) {
            const char* tip = "esquerdo: arrastar=deslocar    direito: clique=novo wpt, arrastar=mover";
            ImVec2 ts = ImGui::CalcTextSize(tip);
            dl->AddText({bp1.x - ts.x, nextY + 4.f}, IM_COL32(255, 255, 255, 150), tip);
            nextY += ts.y + 8.f;
        }

        // RECENTRAR: só aparece com a vista deslocada (_hsdPanned) — volta a
        // seguir o avião sem precisar reduzir e expandir de novo.
        if (editOut && _hsdPanned) {
            const char* rlbl = "RECENTRAR";
            ImVec2 rts = ImGui::CalcTextSize(rlbl);
            ImVec2 rp1{p1.x - 4.f, nextY + rts.y + 6.f};
            ImVec2 rp0{rp1.x - rts.x - 10.f, nextY};
            bool rhov = io.MousePos.x >= rp0.x && io.MousePos.x <= rp1.x &&
                        io.MousePos.y >= rp0.y && io.MousePos.y <= rp1.y;
            if (rhov && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                _hsdPanned = false;
            dl->AddRectFilled(rp0, rp1,
                              rhov ? IM_COL32(80, 60, 30, 230) : IM_COL32(45, 35, 15, 210), 3.f);
            dl->AddRect(rp0, rp1, IM_COL32(255, 200, 80, 140), 3.f);
            dl->AddText({rp0.x + 5.f, rp0.y + 3.f}, IM_COL32(255, 210, 120, 255), rlbl);
        }
    }

    dl->PopClipRect();
    dl->AddRect(p0, p1, IM_COL32(255, 255, 255, 70), 6.f, 0, 1.5f);
}

// ── Picker (north-up, menu de pausa) ──────────────────────────────────────────

void MiniMap::centerPickerOn(double lat, double lon) {
    _pLat = lat; _pLon = lon; _pInit = true;
}

bool MiniMap::drawPicker(ImVec2 size, double& selLat, double& selLon,
                         double acLat, double acLon, float acHdgDeg,
                         const std::vector<Poi>* pois,
                         const std::vector<Rwy>* rwys,
                         RwyPick* rwyPick) {
    if (rwyPick) *rwyPick = RwyPick{};
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

    Xform xf{center, ctx, cty, _pZoom, 1.f, 0.f};

    // Botão MAPA/LISO (mesma ideia do HSD) — calculado ANTES do clique de
    // destino pra esse clique não "vazar" pro mapa por baixo do botão.
    const char* tileBtnLbl = pickerShowTiles ? "MAPA" : "LISO";
    ImVec2 tbp0{p0.x + 4.f, p0.y + 4.f};
    ImVec2 tbts = ImGui::CalcTextSize(tileBtnLbl);
    ImVec2 tbp1{tbp0.x + tbts.x + 10.f, tbp0.y + tbts.y + 6.f};
    bool tileBtnHov = io.MousePos.x >= tbp0.x && io.MousePos.x <= tbp1.x &&
                      io.MousePos.y >= tbp0.y && io.MousePos.y <= tbp1.y;
    if (tileBtnHov && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        pickerShowTiles = !pickerShowTiles;

    // cabeceira de pista sob o cursor (raio de 14 px) → alvo de decolagem
    const Rwy* candRwy = nullptr;
    bool candLE = true;
    if (rwys && hovered) {
        float bestD2 = 14.f * 14.f;
        for (const auto& r : *rwys) {
            ImVec2 ends[2] = { xf.pt(r.leLat, r.leLon), xf.pt(r.heLat, r.heLon) };
            for (int e = 0; e < 2; ++e) {
                float dx = io.MousePos.x - ends[e].x;
                float dy = io.MousePos.y - ends[e].y;
                float d2 = dx * dx + dy * dy;
                if (d2 < bestD2) { bestD2 = d2; candRwy = &r; candLE = (e == 0); }
            }
        }
    }

    // clique (soltar sem ter arrastado) → define o destino (ignora clique
    // que começou em cima do botão MAPA/LISO)
    if (ImGui::IsItemDeactivated()) {
        float dx = io.MousePos.x - io.MouseClickedPos[0].x;
        float dy = io.MousePos.y - io.MouseClickedPos[0].y;
        bool startedOnTileBtn = io.MouseClickedPos[0].x >= tbp0.x && io.MouseClickedPos[0].x <= tbp1.x &&
                               io.MouseClickedPos[0].y >= tbp0.y && io.MouseClickedPos[0].y <= tbp1.y;
        if (dx * dx + dy * dy < 25.f && hovered && !startedOnTileBtn) {
            if (candRwy && rwyPick) {
                // gruda na cabeceira: posição + proa de decolagem
                const Rwy& r = *candRwy;
                rwyPick->valid  = true;
                rwyPick->lat    = candLE ? r.leLat : r.heLat;
                rwyPick->lon    = candLE ? r.leLon : r.heLon;
                rwyPick->hdgDeg = candLE
                    ? bearingDeg(r.leLat, r.leLon, r.heLat, r.heLon)
                    : bearingDeg(r.heLat, r.heLon, r.leLat, r.leLon);
                rwyPick->elevM   = candLE ? r.leElevM : r.heElevM;
                rwyPick->lengthM = (float)geoDistM(r.leLat, r.leLon, r.heLat, r.heLon);
                rwyPick->label   = r.apIdent + " " + (candLE ? r.leIdent : r.heIdent);
                selLat = rwyPick->lat;
                selLon = rwyPick->lon;
            } else {
                double stx = ctx + (io.MousePos.x - center.x) / 256.0;
                double sty = cty + (io.MousePos.y - center.y) / 256.0;
                tile2ll(stx, sty, _pZoom, selLat, selLon);
            }
            clicked = true;
        }
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(p0, p1, true);
    dl->AddRectFilled(p0, p1, IM_COL32(12, 16, 20, 255), 4.f);
    if (pickerShowTiles)
        drawTiles(dl, center, ctx, cty, _pZoom, p0, p1, 0.f);

    double mpp = metersPerPixel(_pLat, _pZoom);

    // pistas (+ espinha de peixe) e navaids/aeroportos
    if (rwys && _pZoom >= 9)
        drawRunwaysOverlay(dl, xf, *rwys, mpp, p0, p1,
                           /*fishbone=*/_pZoom >= 10, /*idents=*/_pZoom >= 12);
    if (pois && _pZoom >= 8)
        drawPoisOverlay(dl, xf, *pois, p0, p1);

    // destaque da cabeceira sob o cursor + dica de decolagem
    if (candRwy) {
        const Rwy& r = *candRwy;
        ImVec2 sp = candLE ? xf.pt(r.leLat, r.leLon) : xf.pt(r.heLat, r.heLon);
        dl->AddCircle(sp, 10.f, IM_COL32(255, 220, 40, 255), 0, 2.5f);
        double hdg = candLE
            ? bearingDeg(r.leLat, r.leLon, r.heLat, r.heLon)
            : bearingDeg(r.heLat, r.heLon, r.leLat, r.leLon);
        char tip[96];
        snprintf(tip, sizeof(tip), "DECOLAR %s %s  proa %03d\xC2\xB0",
                 r.apIdent.c_str(),
                 (candLE ? r.leIdent : r.heIdent).c_str(), (int)std::lround(hdg));
        ImVec2 tp = ImVec2(io.MousePos.x + 14.f, io.MousePos.y - 20.f);
        ImVec2 ts = ImGui::CalcTextSize(tip);
        dl->AddRectFilled(tp - ImVec2(4.f, 2.f), tp + ts + ImVec2(4.f, 2.f),
                          IM_COL32(10, 14, 18, 230), 3.f);
        dl->AddText(tp, IM_COL32(255, 220, 40, 255), tip);
    }

    // avião (posição atual)
    {
        ImVec2 ap = xf.pt(acLat, acLon);
        drawAircraftIcon(dl, ap, acHdgDeg * (float)MM_D2R, 9.f,
                         IM_COL32(255, 255, 255, 255));
    }

    // marcador do destino selecionado (pino laranja)
    {
        ImVec2 mp = xf.pt(selLat, selLon);
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
        if (pickerShowTiles)
            dl->AddText(ImVec2(p1.x - 96.f, p1.y - 20.f),
                        IM_COL32(255, 255, 255, 140), "\xC2\xA9 OpenStreetMap");
    }

    // Botão MAPA/LISO — desenhado por cima de tudo (hit-test já calculado
    // antes do clique de destino, ver tbp0/tbp1 acima)
    dl->AddRectFilled(tbp0, tbp1,
                      tileBtnHov ? IM_COL32(60, 65, 75, 230) : IM_COL32(25, 28, 34, 210), 3.f);
    dl->AddRect(tbp0, tbp1, IM_COL32(255, 255, 255, 90), 3.f);
    dl->AddText({tbp0.x + 5.f, tbp0.y + 3.f}, IM_COL32(230, 230, 230, 255), tileBtnLbl);

    dl->PopClipRect();
    dl->AddRect(p0, p1, IM_COL32(255, 255, 255, 60), 4.f);
    return clicked;
}
