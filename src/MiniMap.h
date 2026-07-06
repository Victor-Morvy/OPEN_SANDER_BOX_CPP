#pragma once
#include <imgui.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <future>
#include <mutex>

// Mapa raster OSM (slippy map) para UI — dois usos:
//   drawHSD    : minimapa heading-up em runtime (avião fixo no centro, mapa gira)
//   drawPicker : mapa north-up no menu de pausa — arrastar=pan, scroll=zoom,
//                clique=define lat/lon de teleporte
//
// Tiles https://tile.openstreetmap.org/{z}/{x}/{y}.png buscados em background
// (std::async, máx 6 simultâneos), decodificados com stb_image e subidos para
// a GPU em processUploads() (thread principal, obrigatório p/ OpenGL).
// Cache LRU de ~220 texturas 256×256 (~57 MB VRAM máx).
class MiniMap {
public:
    struct Poi { double lat, lon; std::string label; };

    // 1×/frame na thread principal, antes de desenhar a UI
    void processUploads();

    // Desenha o HSD ocupando `size` a partir do cursor da janela ImGui atual.
    // Scroll sobre o widget altera hsdZoom.
    void drawHSD(ImVec2 size, double lat, double lon, float hdgDeg);

    // Mapa de seleção. Retorna true no frame em que o usuário clicou
    // (selLat/selLon recebem a posição do clique). Marcador em selLat/selLon,
    // seta do avião em acLat/acLon.
    bool drawPicker(ImVec2 size, double& selLat, double& selLon,
                    double acLat, double acLon, float acHdgDeg,
                    const std::vector<Poi>* pois = nullptr);

    // Recentra a vista do picker (ex.: ao abrir o menu de pausa)
    void centerPickerOn(double lat, double lon);

    // Centro atual da vista do picker (para buscar POIs em volta dele)
    void pickerCenter(double& lat, double& lon) const { lat = _pLat; lon = _pLon; }

    void cleanup();

    int hsdZoom = 12;   // 9..16

private:
    struct Key {
        int z, x, y;
        bool operator==(const Key& o) const { return z==o.z && x==o.x && y==o.y; }
    };
    struct KeyHash {
        size_t operator()(const Key& k) const {
            return std::hash<long long>()(
                (long long)k.z << 40 ^ (long long)k.x << 20 ^ (long long)k.y);
        }
    };
    struct Img  { Key key; bool ok = false; int w = 0, h = 0; std::vector<uint8_t> rgba; };
    struct Tile { unsigned int tex = 0; int lastUse = 0; };

    // id GL do tile; 0 = ainda não disponível (agenda o fetch se houver vaga)
    unsigned int getTile(int z, int x, int y);
    // só consulta o cache (fallback de ancestral enquanto o tile certo carrega)
    unsigned int lookupTile(int z, int x, int y);

    // Desenha os tiles cobrindo o retângulo de clip. (ctx,cty) = coordenada
    // fracionária de tile do ponto `center` da tela; rotRad gira o mapa em
    // torno de center (0 = north-up).
    void drawTiles(ImDrawList* dl, ImVec2 center, double ctx, double cty,
                   int zoom, ImVec2 clipMin, ImVec2 clipMax, float rotRad);

    std::unordered_map<Key, Tile, KeyHash> _tiles;
    std::unordered_map<Key, std::future<Img>, KeyHash> _loading;
    int _frame = 0;

    // vista do picker
    bool   _pInit = false;
    double _pLat = 0.0, _pLon = 0.0;
    int    _pZoom = 12;
};
