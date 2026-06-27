#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <unordered_map>

// Animation state passed to draw() each frame
struct AcAnimState {
    float aileronL  = 0.f;   // [-1..1]
    float aileronR  = 0.f;
    float elevL     = 0.f;   // [-1..1]
    float elevR     = 0.f;
    float rudder    = 0.f;   // [-1..1]
    float flaps     = 0.f;   // [0..1]
    float spoilerL  = 0.f;   // [0..1]
    float spoilerR  = 0.f;
    float gearPos   = 1.f;   // 1=extended, 0=retracted
    float fanAngle  = 0.f;   // cumulative radians
};

struct AcMesh {
    GLuint vao=0, vbo=0, ibo=0;
    int    count=0;
    GLuint texId=0;
    int    matIdx=0;
};

struct AcPart {
    std::string        name;
    std::vector<AcMesh> meshes;
    glm::vec3          pivot{0,0,0};
    glm::vec3          axis{1,0,0};
    float              maxRad = 0.611f;  // max rotation (radians), from JSON max_deg
    float              sign   = 1.f;     // direction sign, from JSON
};

class AcModel {
public:
    bool load(const std::string& objPath);
    void draw(GLuint staticProg, GLuint animProg,
              const glm::mat4& MVP, const AcAnimState& anim) const;
    void cleanup();
    bool ok() const { return _loaded; }

private:
    std::vector<AcMesh> _static;
    std::vector<AcPart> _parts;
    std::vector<GLuint> _textures;
    bool   _loaded = false;
    std::string _dir;
    std::unordered_map<std::string, GLuint> _texCache;

    static const char* TEX_MAP[4];

    bool   loadMesh(const std::string& path, std::vector<AcMesh>& out);
    GLuint loadTex(const std::string& file);
    float  partAngle(const AcPart& part, const AcAnimState& s) const;
    void   drawMeshes(GLuint prog, const std::vector<AcMesh>& meshes) const;
};
