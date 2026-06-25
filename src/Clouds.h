#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <random>

class Clouds {
public:
    static constexpr int N = 500;

    bool init();
    void update(const glm::vec3& acWorld);
    void render(const glm::mat4& view, const glm::mat4& proj,
                const glm::vec3& acWorld, float acMslM,
                const glm::vec3& sunDir, float day);
    void cleanup();

private:
    struct Inst { float wx, wy, wz, sz, op; };

    Inst   _data[N];
    GLuint _vao=0, _vboQ=0, _vboI=0, _prog=0, _tex=0;
    std::mt19937 _rng{1337};

    float rng01() { return (_rng() & 0xFFFF) / 65535.f; }
    void  respawn(int i, float cx, float cz);
};
