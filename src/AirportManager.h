#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <unordered_map>

class TileManager;

class AirportManager {
public:
    bool load(const std::string& apCsv, const std::string& rwCsv,
              double originLat, double originLon);

    void update(const glm::vec3& acWorld, float acMslM,
                TileManager& close, TileManager& far_);

    void render(const glm::mat4& VP,
                const glm::vec3& acWorld, float acMslM, float day);

    void cleanup();

private:
    static constexpr float SHOW_RADIUS_M = 60000.f;

    struct Runway {
        float leLat, leLon, leElevM;
        float heLat, heLon, heElevM;
        float widthM;
        bool  lighted;
        std::string leIdent, heIdent, surface;
    };

    struct Airport {
        std::string ident;
        float lat, lon, elevM;
    };

    struct GpuRwy {
        GLuint vao=0, vbo=0;
        float wx1, wz1, wx2, wz2;
        float elevLE, elevHE;
        float widthM, lengthM;
        bool  lighted;
        glm::vec3 surfColor{0.16f, 0.16f, 0.16f};
        std::string leIdent, heIdent, surface;
    };

    struct LightPt { float x, y, z, r, g, b; };

    double _originLat=0, _originLon=0;
    float  _mPerDegLon=1.f;

    std::vector<Airport>                                  _airports;
    std::unordered_map<std::string, std::vector<Runway>>  _runways;
    std::unordered_map<int64_t, std::vector<size_t>>      _grid;

    std::vector<GpuRwy> _activeRwys;

    // Lights VBO (rebuilt when active set changes, updated every frame for PAPI)
    GLuint  _ltVAO=0, _ltVBO=0;
    int     _ltCap=0, _ltCount=0;

    GLuint _rwProg=0, _ltProg=0;

    // track last cell to avoid per-frame rebuilds
    int _lastCellX=-99999, _lastCellZ=-99999;

    glm::vec2 toWorld(float lat, float lon) const;
    void      clearActive();
    void      addAirportGpu(const Airport& ap,
                             TileManager& close, TileManager& far_);
    void      buildGpuRwy(GpuRwy& g);
    void      rebuildLightVBO(const glm::vec3& acWorld, float acMslM, float day);
};
