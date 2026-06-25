#pragma once
#include <glm/glm.hpp>
#include <unordered_map>
#include <future>
#include <mutex>
#include <vector>
#include <string>

// Fase 3 — Terrain tiles reais com LOD de 2 níveis
// Elevação: AWS Terrarium (PNG RGB → metro MSL)
// Textura:  ESRI Satellite
//
// Uso típico em main.cpp:
//   TileManager farTiles,  closeTiles;
//   farTiles .init(lat, lon, 13, 3, 33);  // zoom13, 7×7, 32-quad grid → 34 km
//   closeTiles.init(lat, lon, 15, 2, 65); // zoom15, 5×5, 64-quad grid →  6 km
//
// Render farTiles primeiro (com glPolygonOffset), depois closeTiles por cima.

class TileManager {
public:
    struct TileKey {
        int z, x, y;
        bool operator==(const TileKey& o) const { return z==o.z&&x==o.x&&y==o.y; }
    };
    struct TileKeyHash {
        size_t operator()(const TileKey& k) const {
            return std::hash<long long>()((long long)k.z<<40|(long long)k.x<<20|k.y);
        }
    };

    // zoom  : nível do mapa (13=4.9km, 14=2.5km, 15=1.2km)
    // grid  : carrega de -grid a +grid → (2*grid+1)² tiles
    // vgrid : vértices por lado (17=16 quads, 33=32, 65=64)
    // yBias: deslocamento Y em metros (ex: -20 para anel externo evitar z-fighting)
    bool init(double originLat, double originLon,
              int zoom=13, int grid=2, int vgrid=17, float yBias=0.f);

    void update(glm::vec3 acWorld, double originLat, double originLon);

    void render(const glm::mat4& VP,
                glm::vec3 acWorld, float acMslM,
                const glm::vec3& sunDir, float day);

    // Elevação em metros MSL na posição do avião. Retorna 0 até tiles carregarem.
    float getElevAt(glm::vec3 acWorld) const;

    // Registra área plana sob pista; achata os vértices do mesh ao carregar tiles.
    // heading = atan2(wx2-wx1, wz2-wz1), leElev/heElev em metros MSL.
    void registerFlatArea(float cx, float cz, float halfWid, float halfLen,
                          float heading, float leElev, float heElev, float blendR = 200.f);

    void cleanup();

private:
    struct TileData {
        TileKey key;
        bool ok = false;
        std::vector<float>         verts; // x,y,z,u,v por vértice
        std::vector<unsigned short> idx;
        std::vector<uint8_t>       tex;
        int texW=0, texH=0;
        std::vector<float> elev;          // vgrid×vgrid elevações MSL
        float worldX=0, worldZ=0;
        float tileSzX=0, tileSzZ=0;
    };

    struct GpuTile {
        unsigned int vao=0, vbo=0, ebo=0, tex=0;
        int idxCount=0;
        float worldX=0, worldZ=0;
        float tileSzX=0, tileSzZ=0;
        std::vector<float> elev;
        int elevGridN=17;
    };

    TileData loadTile(TileKey key, double originLat, double originLon, int vgrid);
    void     uploadTile(TileData& d);
    void     processUploads();

    static std::pair<int,int>       latLonToTile(double lat, double lon, int zoom);
    static std::pair<double,double> tileNWLatLon(int tx, int ty, int zoom);
    static std::vector<uint8_t>     httpGet(const std::string& url);

    // Área plana para aplainar o mesh sob pistas de pouso
    struct FlatArea {
        float cx, cz;           // centro da pista (world)
        float halfWid, halfLen; // metade da largura/comprimento + shoulder
        float ux, uz;           // versor ao longo da pista (LE→HE)
        float px, pz;           // versor perpendicular (direita)
        float leElev, heElev;   // elevação MSL em cada cabeceira
        float blendR;           // raio de blend suave (metros)
    };

    std::unordered_map<TileKey, GpuTile,  TileKeyHash> _gpu;
    std::unordered_map<TileKey, std::future<TileData>, TileKeyHash> _loading;
    std::vector<TileData> _uploadQueue;
    mutable std::mutex    _uploadMtx;

    std::vector<FlatArea> _flatAreas;
    std::vector<FlatArea> _pendingEvict;
    mutable std::mutex    _flatMtx;

    void evictPendingAreas();  // chamado no início de update()

    unsigned int _prog = 0;
    int   _zoom=13, _grid=2, _vgrid=17;
    float _yBias=0.f;
    int _centerTx=-9999, _centerTy=-9999;
};
