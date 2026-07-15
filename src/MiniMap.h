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
    struct Poi {
        double lat, lon;
        std::string label;
        int type = 0;   // 0=aeroporto  1=VOR  2=NDB  3=DME
    };

    // Segmento de pista para desenhar no mapa (+ espinha de peixe nas
    // aproximações e clique-na-cabeceira no picker)
    struct Rwy {
        std::string apIdent, leIdent, heIdent;
        double leLat, leLon, heLat, heLon;
        float  widthM;
        float  leElevM, heElevM;   // MSL [m]
    };

    // Ponto de trajetória prevista (trend vector do HSD)
    struct PathPt { double lat, lon; };

    // Preenchido quando o clique do picker "gruda" numa cabeceira:
    // posição = cabeceira, proa = decolagem (cabeceira → outra ponta)
    struct RwyPick {
        bool   valid = false;
        double lat = 0, lon = 0;
        double hdgDeg = 0;
        float  elevM = 0;
        float  lengthM = 0;  // cabeceira→cabeceira — usado p/ mirar o pouso no ILS sintético
        std::string label;   // ex.: "SBGL 10"
    };

    // Resultado da edição de rota no HSD expandido (ver drawHSD/editWpts) —
    // populado só quando `editOut` é passado (não-nulo). Quem chama aplica o
    // resultado no plano de voo de verdade (gm.fplan) — o MiniMap não sabe
    // nada de GuidanceModule, só mexe em lat/lon.
    struct RouteEdit {
        bool   added   = false;   // clique direito longe de qualquer waypoint → novo no fim
        double addLat = 0.0, addLon = 0.0;
        int    dragIdx = -1;      // índice movido NESTE frame (-1 = nenhum)
        double dragLat = 0.0, dragLon = 0.0;
    };

    // 1×/frame na thread principal, antes de desenhar a UI
    void processUploads();

    // Desenha o HSD ocupando `size` a partir do cursor da janela ImGui atual.
    // Scroll sobre o widget altera hsdZoom. Todas as polilinhas (pred/guidance/
    // route) ganham um halo preto por baixo para ficarem legíveis sobre
    // qualquer cor de tile. Botão no canto alterna hsdExpanded (ver campo).
    //   pred         = trend vector físico (turn rate atual), verde
    //   guidancePath = curva que o AP vai voar até capturar o alvo de
    //                  heading/NAV (simulada pelo chamador com o mesmo ganho
    //                  do GuidanceModule), amarelo
    //   route        = plano de voo completo ligando os waypoints, rosa
    //   editWpts     = só os waypoints (sem o avião), usados para o hit-test
    //                  do arrasto — passar só quando hsdExpanded (editOut
    //                  não-nulo é o que de fato liga a edição por clique)
    //   editOut      = clique/arrasto direito no frame: botão direito longe
    //                  de qualquer waypoint (raio 16 px) adiciona um novo na
    //                  posição do clique; segurando o botão direito sobre um
    //                  waypoint e arrastando move ele (lat/lon atualizados
    //                  todo frame do arrasto, ver RouteEdit)
    void drawHSD(ImVec2 size, double lat, double lon, float hdgDeg,
                 const std::vector<Poi>* pois = nullptr,
                 const std::vector<Rwy>* rwys = nullptr,
                 const std::vector<PathPt>* pred = nullptr,
                 const std::vector<PathPt>* guidancePath = nullptr,
                 const std::vector<PathPt>* route = nullptr,
                 const std::vector<PathPt>* editWpts = nullptr,
                 RouteEdit* editOut = nullptr);

    // Mapa de seleção. Retorna true no frame em que o usuário clicou
    // (selLat/selLon recebem a posição do clique). Marcador em selLat/selLon,
    // seta do avião em acLat/acLon. Clique perto de uma cabeceira de pista
    // gruda nela e preenche rwyPick (proa de decolagem).
    bool drawPicker(ImVec2 size, double& selLat, double& selLon,
                    double acLat, double acLon, float acHdgDeg,
                    const std::vector<Poi>* pois = nullptr,
                    const std::vector<Rwy>* rwys = nullptr,
                    RwyPick* rwyPick = nullptr);

    // Recentra a vista do picker (ex.: ao abrir o menu de pausa)
    void centerPickerOn(double lat, double lon);

    // Centro atual da vista do picker (para buscar POIs em volta dele)
    void pickerCenter(double& lat, double& lon) const { lat = _pLat; lon = _pLon; }

    // Vista do HSD expandido está deslocada do avião (arrastada com o
    // esquerdo, ver drawHSD)? Quem chama PRECISA checar isto — enquanto
    // deslocada, os POIs/pistas/navaids buscados em volta do AVIÃO não
    // cobrem a área realmente visível no widget; buscar em volta de
    // hsdViewCenter() em vez disso.
    bool hsdPanned() const { return _hsdPanned; }
    void hsdViewCenter(double& lat, double& lon) const { lat = _hsdViewLat; lon = _hsdViewLon; }

    void cleanup();

    int  hsdZoom      = 12;    // 5..16 (scroll)
    // Botão no canto do HSD alterna: tiles OSM completos ↔ fundo liso (sem
    // land/water real — o pipeline só busca tiles raster, sem dado vetorial
    // de terra/água — mas separa bem o branco da espinha de peixe/pistas/
    // waypoints do resto do mapa, que era o pedido quando não dá pra ter
    // land/water de verdade).
    bool hsdShowTiles    = true;
    bool pickerShowTiles = true;   // mesmo botão, independente, no mapa do menu de pausa

    // Botão no canto do HSD (topo-direita) alterna widget pequeno ↔ grande —
    // expandido é o que libera a edição de rota por clique direito (ver
    // drawHSD/editOut). Começa pequeno (false); quem chama controla o
    // tamanho de verdade passado em `size`, isto aqui é só a intenção.
    bool hsdExpanded = false;

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

    // arrasto de waypoint no HSD expandido (persiste entre frames enquanto o
    // botão direito estiver segurado — ver drawHSD)
    int _hsdDragIdx = -1;

    // vista deslocada do HSD expandido (arrastar com botão esquerdo) — só
    // usada enquanto hsdExpanded; volta a seguir o avião ao reduzir ou
    // clicar RECENTRAR (ver drawHSD)
    bool   _hsdPanned  = false;
    double _hsdViewLat = 0.0, _hsdViewLon = 0.0;
};
