#pragma once
#include <glm/glm.hpp>

class Sky {
public:
    bool init();
    void render(const glm::mat4& view, const glm::mat4& proj,
                const glm::vec3& sunDir, float dayFactor);
    void cleanup();

    // Calcula direção do sol a partir de hora local simples (0–24 h)
    static glm::vec3 sunDirection(float localHour);
    // day factor: 0 = noite, 1 = dia pleno (suavizado na transição)
    static float dayFactor(const glm::vec3& sunDir);

private:
    unsigned int _prog = 0;
    unsigned int _vao  = 0, _vbo = 0, _ebo = 0;
    int          _idxCount = 0;
};
