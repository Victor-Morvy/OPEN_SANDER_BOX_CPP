#pragma once
#include <glm/glm.hpp>

class Terrain {
public:
    // gridN: vértices por lado (par+1 recomendado, ex: 201)
    // tileM: tamanho de cada quadrado em metros (ex: 250)
    bool init(int gridN = 201, float tileM = 250.f);
    // acWorldPos: posição do avião no espaço mundial (aircraft = render origin)
    void render(const glm::mat4& view, const glm::mat4& proj,
                const glm::vec3& acWorldPos, float dayFactor);
    void cleanup();
private:
    unsigned int _prog = 0, _vao = 0, _vbo = 0, _ebo = 0;
    int  _idxCount = 0;
    int  _gridN    = 0;
    float _tileM   = 0;
};
