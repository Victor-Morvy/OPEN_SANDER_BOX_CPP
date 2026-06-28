#include "Sky.h"
#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>
#include <cstdio>
#include <vector>

// ── Shaders ──────────────────────────────────────────────────────────────────

static const char* SKY_VERT = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
out vec3 vDir;
uniform mat4 uVP;
uniform vec3 uCamPos;
void main() {
    vec3 world = aPos * 90000.0 + uCamPos;   // esfera de 90 km centrada na câmera
    gl_Position = uVP * vec4(world, 1.0);
    gl_Position.z = gl_Position.w * 0.9999;  // força para o fundo (quase 1.0 NDC)
    vDir = normalize(aPos);
}
)glsl";

// Modelo atmosférico simplificado: scattering Rayleigh + disco solar + dawn glow
static const char* SKY_FRAG = R"glsl(
#version 330 core
in vec3 vDir;
out vec4 FragColor;

uniform vec3  uSunDir;   // direção normalizada para o sol
uniform float uDay;      // 0 = noite, 1 = dia pleno

void main() {
    vec3 dir = normalize(vDir);
    float up = dir.y;

    // ── Cor do céu diurno (aproximação Rayleigh) ──────────────────────────────
    vec3 zenith  = vec3(0.08, 0.20, 0.52);
    vec3 horizon = vec3(0.50, 0.68, 0.88);
    float t = pow(max(up, 0.0), 0.35);
    vec3 dayColor = mix(horizon, zenith, t);

    // ── Brilho de amanhecer/entardecer ───────────────────────────────────────
    float sunElev  = uSunDir.y;
    float sunFwd   = max(0.0, dot(dir, vec3(uSunDir.x, 0.0, uSunDir.z)));
    float dawnGlow = pow(sunFwd, 5.0) * exp(-abs(up) * 5.0)
                   * smoothstep(-0.1, 0.3, sunElev)
                   * (1.0 - smoothstep(0.3, 0.7, sunElev));
    dayColor += vec3(1.0, 0.45, 0.12) * dawnGlow * 1.4;

    // ── Neblina no horizonte ──────────────────────────────────────────────────
    float hazeAmt = exp(-max(up, 0.0) * 8.0) * uDay;
    dayColor = mix(dayColor, vec3(0.80, 0.87, 0.96), hazeAmt * 0.35);

    // ── Céu noturno ───────────────────────────────────────────────────────────
    vec3 nightColor = vec3(0.008, 0.012, 0.030);

    // ── Mistura dia/noite ─────────────────────────────────────────────────────
    vec3 sky = mix(nightColor, dayColor, uDay);

    // ── Disco solar ──────────────────────────────────────────────────────────
    float cosS  = dot(dir, uSunDir);
    float disc  = smoothstep(0.9997, 0.9999, cosS);
    vec3 sunCol = vec3(1.6, 1.45, 1.1) * disc;
    // coroa difusa
    float corona = pow(max(0.0, cosS), 64.0) * uDay * 0.25;
    sunCol += vec3(1.0, 0.85, 0.6) * corona;
    sky += sunCol;

    // Abaixo do horizonte: continua com a cor do horizonte (evita "faixa" marrom visível)
    float groundT = smoothstep(0.0, -0.60, dir.y);
    vec3 earthHaze = mix(vec3(0.008, 0.010, 0.018), vec3(0.50, 0.68, 0.88), uDay);
    sky = mix(sky, earthHaze, groundT * 0.85);

    FragColor = vec4(sky, 1.0);
}
)glsl";

// ── Helpers ──────────────────────────────────────────────────────────────────

static GLuint compileShader(GLenum t, const char* src) {
    GLuint sh = glCreateShader(t);
    glShaderSource(sh, 1, &src, nullptr);
    glCompileShader(sh);
    GLint ok = 0; glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) { char buf[512]; glGetShaderInfoLog(sh, 512, nullptr, buf);
               fprintf(stderr, "[Sky shader] %s\n", buf); }
    return sh;
}
static GLuint linkProgram(const char* vs, const char* fs) {
    GLuint v = compileShader(GL_VERTEX_SHADER, vs);
    GLuint f = compileShader(GL_FRAGMENT_SHADER, fs);
    GLuint p = glCreateProgram();
    glAttachShader(p, v); glAttachShader(p, f); glLinkProgram(p);
    glDeleteShader(v); glDeleteShader(f);
    return p;
}

// ── Esfera UV (raio 1, vira para dentro com índices invertidos) ───────────────

static void buildSphere(int stacks, int slices,
                        std::vector<float>& verts,
                        std::vector<unsigned int>& idxs)
{
    constexpr float PI = 3.14159265358979f;
    for (int i = 0; i <= stacks; ++i) {
        float phi = PI * 0.5f - PI * i / stacks; // -π/2 a π/2
        float y   = sinf(phi);
        float r   = cosf(phi);
        for (int j = 0; j <= slices; ++j) {
            float theta = 2.f * PI * j / slices;
            verts.push_back(r * cosf(theta));
            verts.push_back(y);
            verts.push_back(r * sinf(theta));
        }
    }
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            unsigned int a = i*(slices+1)+j, b=a+1, c=a+slices+1, d=c+1;
            // Winding invertido → faces visíveis por dentro
            idxs.push_back(a); idxs.push_back(c); idxs.push_back(b);
            idxs.push_back(b); idxs.push_back(c); idxs.push_back(d);
        }
    }
}

// ── API pública ───────────────────────────────────────────────────────────────

bool Sky::init() {
    _prog = linkProgram(SKY_VERT, SKY_FRAG);

    std::vector<float>        verts;
    std::vector<unsigned int> idxs;
    buildSphere(24, 48, verts, idxs);
    _idxCount = (int)idxs.size();

    glGenVertexArrays(1, &_vao);
    glGenBuffers(1, &_vbo);
    glGenBuffers(1, &_ebo);
    glBindVertexArray(_vao);
      glBindBuffer(GL_ARRAY_BUFFER, _vbo);
      glBufferData(GL_ARRAY_BUFFER, verts.size()*sizeof(float), verts.data(), GL_STATIC_DRAW);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, idxs.size()*sizeof(unsigned int), idxs.data(), GL_STATIC_DRAW);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
      glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    return true;
}

void Sky::render(const glm::mat4& view, const glm::mat4& proj,
                 const glm::vec3& sunDir, float day)
{
    glDepthMask(GL_FALSE);          // não escreve no depth buffer
    glDisable(GL_DEPTH_TEST);

    glUseProgram(_prog);

    // Câmera: extrair posição da view matrix (inverse row 3)
    glm::mat4 invV = glm::inverse(view);
    glm::vec3 camPos = glm::vec3(invV[3]);

    glm::mat4 VP = proj * view;
    glUniformMatrix4fv(glGetUniformLocation(_prog,"uVP"),   1, GL_FALSE, glm::value_ptr(VP));
    glUniform3fv      (glGetUniformLocation(_prog,"uCamPos"), 1, glm::value_ptr(camPos));
    glUniform3fv      (glGetUniformLocation(_prog,"uSunDir"), 1, glm::value_ptr(sunDir));
    glUniform1f       (glGetUniformLocation(_prog,"uDay"),    day);

    glBindVertexArray(_vao);
    glDrawElements(GL_TRIANGLES, _idxCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
}

void Sky::cleanup() {
    glDeleteVertexArrays(1, &_vao);
    glDeleteBuffers(1, &_vbo);
    glDeleteBuffers(1, &_ebo);
    glDeleteProgram(_prog);
}

// ── Funções utilitárias de hora solar (simples) ───────────────────────────────

glm::vec3 Sky::sunDirection(float localHour) {
    // Eleva o sol de E (leste) para O (oeste) ao longo do dia
    // Mapeamento: 06h → nasce a Leste, 12h → zênite ao Sul, 18h → poente a Oeste
    constexpr float PI = 3.14159265358979f;
    float angle = (localHour - 6.f) / 12.f * PI; // 0h → -90°, 12h → 90°(topo), 24h → 270°
    float elev  = sinf(angle);                    // +1 ao meio-dia
    float azFrac = (localHour - 6.f) / 12.f;     // 0 a 1 de Leste a Oeste
    float azAngle = (azFrac - 0.5f) * PI;         // -90° a +90°
    return glm::normalize(glm::vec3(sinf(azAngle), elev, -cosf(azAngle)));
}

float Sky::dayFactor(const glm::vec3& sunDir) {
    // Transição suave: sol abaixo de -5° → noite, acima de 10° → dia pleno
    return glm::clamp((sunDir.y + 0.08f) / 0.18f, 0.f, 1.f);
}
