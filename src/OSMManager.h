#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <climits>

class TileManager;

// Renders OSM features: buildings, roads, water bodies.
// Async Overpass fetch (libcurl) with file-based 7-day cache.
// All geometry is GPU-uploaded on the main thread after the bg thread finishes.
class OSMManager {
public:
    bool init(double originLat, double originLon,
              const std::string& cacheDir);

    // Call each frame. Triggers async fetch when cell changes.
    void update(double acLat, double acLon,
                TileManager& close, TileManager& far_);

    // Call in render loop after terrain.
    void render(const glm::mat4& VP, const glm::vec3& camPos,
                float dayT, float timeS);

    void cleanup();

private:
    // ── GPU mesh ──────────────────────────────────────────────────────────────
    enum MeshType : uint8_t { BLDWALL=0, BLDROOF=1, ROAD=2, WATER=3 };

    struct GpuMesh {
        GLuint   vao=0, vbo=0, ibo=0;
        uint32_t indexCount=0;
        MeshType type;
        glm::vec3 flatColor{1,1,1};
        int      facadeVariant=0;
        float    baseY=0.f;      // terrain elevation at centroid (added in shader)
    };

    // ── CPU raw mesh (built on bg thread, no GL calls) ────────────────────────
    struct RawMesh {
        std::vector<float>    verts;
        std::vector<uint32_t> idx;
        MeshType  type;
        glm::vec3 flatColor{1,1,1};
        int       facadeVariant=0;
        glm::vec2 centroidXZ{0,0};  // world XZ centroid, for elevation sampling
    };
    struct RawBatch { std::vector<RawMesh> meshes; };

    // ── State ─────────────────────────────────────────────────────────────────
    std::vector<GpuMesh> _meshes;
    std::thread          _thread;
    std::mutex           _mutex;
    std::atomic<int>     _gen{0};
    bool                 _hasPending = false;
    RawBatch             _pending;

    GLuint _wallProg  = 0;
    GLuint _flatProg  = 0;
    GLuint _waterProg = 0;
    GLuint _facadeTex[5] = {};

    double      _originLat  = 0;
    double      _originLon  = 0;
    double      _mPerDegLon = 1.0;
    std::string _cacheDir;
    int         _lastCellX  = INT_MIN;
    int         _lastCellZ  = INT_MIN;

    // ── Helpers ───────────────────────────────────────────────────────────────
    glm::vec2 toWorld(double lat, double lon) const;
    void      clearMeshes();
    void      uploadPending(TileManager& close, TileManager& far_);
    void      uploadMesh(GpuMesh& g, const RawMesh& r);

    void  fetchAsync(double lat, double lon, int gen);
    void  buildGeometry(const std::string& json, int gen, RawBatch& out);
    void  addBuilding(const std::vector<std::pair<double,double>>& pts,
                      float height, int bldIdx, RawBatch& out);
    void  addRoad(const std::vector<std::pair<double,double>>& pts,
                  float halfW, glm::vec3 col, RawBatch& out);
    void  addWater(const std::vector<std::pair<double,double>>& pts,
                   RawBatch& out);

    void   genFacadeTextures();
    GLuint compileShader(const char* vert, const char* frag);
};
