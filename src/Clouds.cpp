#include "Clouds.h"
#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>
#include <cstdio>

// ── Shaders ───────────────────────────────────────────────────────────────────

static const char* CL_VERT = R"glsl(
#version 330 core
layout(location=0) in vec2 aCorner;       // quad: −1..+1
layout(location=1) in vec4 iXYZS;         // instance: world(x,y,z), size(w)
layout(location=2) in float iOp;          // instance: opacity

out vec2  vUV;
out float vOp;
out float vDist;

uniform mat4 uView;
uniform mat4 uProj;
uniform vec3 uAcFull;   // (east, mslAlt, south)

void main(){
    // Cylindrical billboard: right from view, always stand upright
    vec3 right = normalize(vec3(uView[0][0], uView[1][0], uView[2][0]));
    vec3 up    = vec3(0.0, 1.0, 0.0);
    vec3 w = iXYZS.xyz + right * aCorner.x * iXYZS.w
                       + up    * aCorner.y * iXYZS.w;
    gl_Position = uProj * uView * vec4(w - uAcFull, 1.0);
    vUV   = aCorner * 0.5 + 0.5;
    vOp   = iOp;
    vDist = length(iXYZS.xz - uAcFull.xz);
}
)glsl";

static const char* CL_FRAG = R"glsl(
#version 330 core
in vec2  vUV;
in float vOp;
in float vDist;
out vec4 FragColor;

uniform sampler2D uTex;
uniform vec3      uSunDir;
uniform float     uDay;

void main(){
    float a = texture(uTex, vUV).a * vOp;
    // Fade com distância (20−45 km)
    a *= 1.0 - smoothstep(20000.0, 45000.0, vDist);
    // Desaparece à noite
    a *= smoothstep(0.0, 0.15, uDay);

    // Iluminação topo/base: topo mais brilhante, base cinza
    float lit = clamp(uSunDir.y * 0.4 + 0.75, 0.4, 1.0);
    // Fundo da nuvem (v baixo) mais escuro
    float shade = mix(0.70, 1.0, vUV.y);
    vec3 col = vec3(lit * shade);

    // Borda alaranjada perto do pôr/nascer do sol
    float sunGlow = max(0.0, 1.0 - abs(uSunDir.y) * 4.0);
    col = mix(col, col * vec3(1.4, 0.85, 0.55), sunGlow * 0.5);

    FragColor = vec4(col, a);
}
)glsl";

// ── Helpers GL ────────────────────────────────────────────────────────────────

static GLuint makeShader(GLenum t, const char* s){
    GLuint sh=glCreateShader(t);
    glShaderSource(sh,1,&s,nullptr); glCompileShader(sh);
    GLint ok=0; glGetShaderiv(sh,GL_COMPILE_STATUS,&ok);
    if(!ok){char b[512];glGetShaderInfoLog(sh,512,nullptr,b);fprintf(stderr,"[Clouds] %s\n",b);}
    return sh;
}

// ── Classe ────────────────────────────────────────────────────────────────────

void Clouds::respawn(int i, float cx, float cz){
    float angle = rng01() * 6.28318f;
    float dist  = sqrtf(rng01()) * 44000.f;   // distribuição uniforme no disco
    _data[i].wx = cx + cosf(angle) * dist;
    _data[i].wz = cz + sinf(angle) * dist;
    _data[i].wy = 1200.f + rng01() * 2200.f;  // 1200–3400 m MSL (cumulus)
    _data[i].sz = 350.f  + rng01() * 1300.f;  // 350–1650 m de tamanho
    _data[i].op = 0.30f  + rng01() * 0.50f;
}

bool Clouds::init(){
    // Programa de shader
    GLuint vs=makeShader(GL_VERTEX_SHADER,CL_VERT);
    GLuint fs=makeShader(GL_FRAGMENT_SHADER,CL_FRAG);
    _prog=glCreateProgram();
    glAttachShader(_prog,vs); glAttachShader(_prog,fs); glLinkProgram(_prog);
    glDeleteShader(vs); glDeleteShader(fs);

    // Textura soft-blob 128×128 RGBA procedural
    {
        static uint8_t tex[128*128*4];
        for(int y=0;y<128;y++) for(int x=0;x<128;x++){
            float u=(x-63.5f)/63.5f, v=(y-63.5f)/63.5f;
            float r=sqrtf(u*u+v*v);
            // Bordo suave com dois lobes para parecer menos redondo
            float a = r<1.f ? powf(1.f - r*r, 2.8f) : 0.f;
            // Pequeno ruído sintético para variar a silhueta
            a *= 0.82f + 0.18f * sinf(u*9.1f + 1.3f) * cosf(v*7.3f + 0.7f);
            a = std::max(0.f, a);
            int idx=(y*128+x)*4;
            tex[idx]=tex[idx+1]=tex[idx+2]=255;
            tex[idx+3]=(uint8_t)(std::min(a, 1.f)*255);
        }
        glGenTextures(1,&_tex);
        glBindTexture(GL_TEXTURE_2D,_tex);
        glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,128,128,0,GL_RGBA,GL_UNSIGNED_BYTE,tex);
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    }

    // Quad de referência (2 triângulos = 6 vértices de −1..+1)
    static const float quad[]={-1,-1,1,-1,1,1,-1,-1,1,1,-1,1};

    glGenVertexArrays(1,&_vao);
    glGenBuffers(1,&_vboQ);
    glGenBuffers(1,&_vboI);
    glBindVertexArray(_vao);

    // VBO 0: corners (por vértice)
    glBindBuffer(GL_ARRAY_BUFFER,_vboQ);
    glBufferData(GL_ARRAY_BUFFER,sizeof(quad),quad,GL_STATIC_DRAW);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,0,nullptr);
    glEnableVertexAttribArray(0);
    // divisor 0 = per-vertex (padrão)

    // VBO 1: instâncias (por instância)
    glBindBuffer(GL_ARRAY_BUFFER,_vboI);
    glBufferData(GL_ARRAY_BUFFER,N*5*4,nullptr,GL_DYNAMIC_DRAW);
    // location 1: iXYZS (4 floats: x,y,z,size)
    glVertexAttribPointer(1,4,GL_FLOAT,GL_FALSE,20,(void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1,1);
    // location 2: iOp (1 float: opacity) — stride=20, offset=16
    glVertexAttribPointer(2,1,GL_FLOAT,GL_FALSE,20,(void*)16);
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2,1);

    glBindVertexArray(0);

    // Posicionar nuvens inicialmente ao redor da origem
    for(int i=0;i<N;i++) respawn(i, 0.f, 0.f);

    printf("[Clouds] %d instâncias, altitude 1200–3400 m MSL\n", N);
    return true;
}

void Clouds::update(const glm::vec3& acWorld){
    constexpr float MAX2 = 50000.f * 50000.f;
    for(int i=0;i<N;i++){
        float dx=_data[i].wx-acWorld.x, dz=_data[i].wz-acWorld.z;
        if(dx*dx+dz*dz > MAX2) respawn(i, acWorld.x, acWorld.z);
    }
}

void Clouds::render(const glm::mat4& view, const glm::mat4& proj,
                     const glm::vec3& acWorld, float acMslM,
                     const glm::vec3& sunDir, float day)
{
    if(day < 0.01f) return;  // noite completa: não renderizar

    // Upload instâncias (são modificadas por update)
    glBindBuffer(GL_ARRAY_BUFFER,_vboI);
    glBufferSubData(GL_ARRAY_BUFFER,0,N*5*4,_data);

    glUseProgram(_prog);
    glm::vec3 acFull(acWorld.x, acMslM, acWorld.z);
    glUniformMatrix4fv(glGetUniformLocation(_prog,"uView"), 1,GL_FALSE,glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(_prog,"uProj"), 1,GL_FALSE,glm::value_ptr(proj));
    glUniform3fv(glGetUniformLocation(_prog,"uAcFull"),  1,glm::value_ptr(acFull));
    glUniform3fv(glGetUniformLocation(_prog,"uSunDir"),  1,glm::value_ptr(sunDir));
    glUniform1f (glGetUniformLocation(_prog,"uDay"),  day);
    glUniform1i (glGetUniformLocation(_prog,"uTex"),  0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D,_tex);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);   // transparente não escreve profundidade
    glDisable(GL_CULL_FACE);

    glBindVertexArray(_vao);
    glDrawArraysInstanced(GL_TRIANGLES, 0, 6, N);
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
}

void Clouds::cleanup(){
    if(_vao) glDeleteVertexArrays(1,&_vao);
    if(_vboQ)glDeleteBuffers(1,&_vboQ);
    if(_vboI)glDeleteBuffers(1,&_vboI);
    if(_tex) glDeleteTextures(1,&_tex);
    if(_prog)glDeleteProgram(_prog);
}
