#include "Terrain.h"
#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <cstdio>
#include <cmath>

// ── Shaders ──────────────────────────────────────────────────────────────────
// Convenção render: X=direita, Y=cima, Z=cauda(aft/Sul)
// Terreno em y=0 no espaço mundial; avião em y=altitude acima do terreno.
// O vertex calcula a posição em render space subtraindo uAcPos.

static const char* TRN_VERT = R"glsl(
#version 330 core
layout(location=0) in vec2 aXZ;   // grid local (sem altitude)
out vec2 vWorldXZ;                 // para fog e cor
out vec3 vColor;
uniform mat4  uVP;
uniform vec3  uAcPos;  // posição do avião no mundo (aircraft = render origin)
uniform float uTileM;  // tamanho de cada tile em metros

void main() {
    // Snap do grid ao tile mais próximo do avião (sem deslizamento visual)
    vec2 snap = floor(uAcPos.xz / uTileM) * uTileM;
    // Posição mundial deste vértice (terreno plano, y=0)
    vec2 wXZ   = aXZ + snap;
    vec3 world = vec3(wXZ.x, 0.0, wXZ.y);
    // Espaço de render: subtrair posição do avião (avião é a origem)
    vec3 renderPos = world - uAcPos;
    gl_Position = uVP * vec4(renderPos, 1.0);
    vWorldXZ = wXZ;

    // Xadrez em espaço mundial — padrão visível pelo movimento
    vec2 cell = floor(wXZ / (uTileM * 4.0));
    float even = mod(cell.x + cell.y, 2.0);
    vColor = mix(vec3(0.28, 0.40, 0.21), vec3(0.40, 0.33, 0.19), even);
}
)glsl";

static const char* TRN_FRAG = R"glsl(
#version 330 core
in vec2 vWorldXZ;
in vec3 vColor;
out vec4 FragColor;
uniform vec3  uAcPos;
uniform float uDay;

void main() {
    vec3 col = vColor;
    // Fog exponencial baseado na distância XZ ao avião
    float dist = length(vWorldXZ - uAcPos.xz);
    float fog  = exp(-dist * 0.00003);
    vec3 fogDay   = vec3(0.68, 0.75, 0.88);
    vec3 fogNight = vec3(0.04, 0.05, 0.08);
    vec3 fogCol   = mix(fogNight, fogDay, uDay);
    col = mix(fogCol, col, clamp(fog, 0.0, 1.0));
    col *= mix(0.05, 1.0, uDay);   // escurece à noite
    FragColor = vec4(col, 1.0);
}
)glsl";

// ── Helpers ───────────────────────────────────────────────────────────────────

static GLuint buildShader(GLenum t, const char* s) {
    GLuint sh = glCreateShader(t);
    glShaderSource(sh,1,&s,nullptr); glCompileShader(sh);
    GLint ok=0; glGetShaderiv(sh,GL_COMPILE_STATUS,&ok);
    if(!ok){char b[512];glGetShaderInfoLog(sh,512,nullptr,b);
            fprintf(stderr,"[Terrain shader] %s\n",b);}
    return sh;
}

// ── API ───────────────────────────────────────────────────────────────────────

bool Terrain::init(int gridN, float tileM) {
    _gridN = gridN; _tileM = tileM;

    GLuint v = buildShader(GL_VERTEX_SHADER, TRN_VERT);
    GLuint f = buildShader(GL_FRAGMENT_SHADER, TRN_FRAG);
    _prog = glCreateProgram();
    glAttachShader(_prog,v); glAttachShader(_prog,f); glLinkProgram(_prog);
    glDeleteShader(v); glDeleteShader(f);

    // Grid centrado na origem local (−half a +half em X e Z)
    int N = gridN;
    float half = (N-1) * tileM * 0.5f;
    std::vector<float> verts;
    verts.reserve(N * N * 2);
    for(int iz=0;iz<N;iz++)
        for(int ix=0;ix<N;ix++){
            verts.push_back(ix * tileM - half);
            verts.push_back(iz * tileM - half);
        }

    std::vector<unsigned int> idxs;
    idxs.reserve((N-1)*(N-1)*6);
    for(int iz=0;iz<N-1;iz++)
        for(int ix=0;ix<N-1;ix++){
            unsigned int a=iz*N+ix, b=a+1, c=a+N, d=c+1;
            idxs.push_back(a); idxs.push_back(c); idxs.push_back(b);
            idxs.push_back(b); idxs.push_back(c); idxs.push_back(d);
        }
    _idxCount = (int)idxs.size();

    glGenVertexArrays(1,&_vao);
    glGenBuffers(1,&_vbo); glGenBuffers(1,&_ebo);
    glBindVertexArray(_vao);
      glBindBuffer(GL_ARRAY_BUFFER,_vbo);
      glBufferData(GL_ARRAY_BUFFER,verts.size()*sizeof(float),verts.data(),GL_STATIC_DRAW);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,_ebo);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER,idxs.size()*sizeof(unsigned int),idxs.data(),GL_STATIC_DRAW);
      glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,2*sizeof(float),(void*)0);
      glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    printf("[Terrain] grid=%dx%d tile=%.0fm → %.1fkm × %.1fkm  tri=%d\n",
           N,N,tileM,(N-1)*tileM/1000.f,(N-1)*tileM/1000.f,(N-1)*(N-1)*2);
    return true;
}

void Terrain::render(const glm::mat4& view, const glm::mat4& proj,
                     const glm::vec3& acWorldPos, float day)
{
    glUseProgram(_prog);
    glm::mat4 VP = proj * view;
    glUniformMatrix4fv(glGetUniformLocation(_prog,"uVP"),     1,GL_FALSE,glm::value_ptr(VP));
    glUniform3fv      (glGetUniformLocation(_prog,"uAcPos"),  1,glm::value_ptr(acWorldPos));
    glUniform1f       (glGetUniformLocation(_prog,"uTileM"),  _tileM);
    glUniform1f       (glGetUniformLocation(_prog,"uDay"),    day);

    glBindVertexArray(_vao);
    glDrawElements(GL_TRIANGLES,_idxCount,GL_UNSIGNED_INT,nullptr);
    glBindVertexArray(0);
}

void Terrain::cleanup() {
    glDeleteVertexArrays(1,&_vao);
    glDeleteBuffers(1,&_vbo); glDeleteBuffers(1,&_ebo);
    glDeleteProgram(_prog);
}
