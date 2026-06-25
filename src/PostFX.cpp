#include "PostFX.h"
#include <cstdio>
#include <algorithm>

// ── Shaders ───────────────────────────────────────────────────────────────────

static const char* Q_VERT = R"glsl(
#version 330 core
layout(location=0) in vec2 aPos;
out vec2 vUV;
void main(){
    gl_Position = vec4(aPos, 0.0, 1.0);
    vUV = aPos * 0.5 + 0.5;
}
)glsl";

// Extrai pixels acima do threshold (saída para FBO de bloom, meia res)
static const char* THRESH_FRAG = R"glsl(
#version 330 core
in vec2 vUV;
out vec4 FragColor;
uniform sampler2D uScene;
uniform float uThreshold;
void main(){
    vec3 col = texture(uScene, vUV).rgb;
    float lum = dot(col, vec3(0.2126, 0.7152, 0.0722));
    float factor = max(0.0, lum - uThreshold) / max(lum, 0.001);
    FragColor = vec4(col * factor, 1.0);
}
)glsl";

// Gaussian blur 9-tap com linear-sampling trick
static const char* BLUR_FRAG = R"glsl(
#version 330 core
in vec2 vUV;
out vec4 FragColor;
uniform sampler2D uTex;
uniform vec2      uDir;  // (1,0) horizontal | (0,1) vertical

// Pesos para kernel Gaussiano sigma≈2
const float W[5] = float[](0.22703, 0.19459, 0.12162, 0.05405, 0.01621);

void main(){
    vec2 d = uDir / textureSize(uTex, 0);
    vec3 r = texture(uTex, vUV).rgb * W[0];
    for(int i=1;i<5;i++){
        r += texture(uTex, vUV + d*float(i)).rgb * W[i];
        r += texture(uTex, vUV - d*float(i)).rgb * W[i];
    }
    FragColor = vec4(r, 1.0);
}
)glsl";

// Composição final: cena HDR + bloom, ACES tone-map, gamma
static const char* COMP_FRAG = R"glsl(
#version 330 core
in vec2 vUV;
out vec4 FragColor;
uniform sampler2D uScene;
uniform sampler2D uBloom;
uniform float     uBloomStr;
uniform float     uExposure;

// ACES filmic aproximação (Narkowicz 2015)
vec3 aces(vec3 x){
    x *= uExposure;
    return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14), 0.0, 1.0);
}

void main(){
    vec3 hdr   = texture(uScene, vUV).rgb;
    vec3 bloom = texture(uBloom, vUV).rgb;
    vec3 col   = aces(hdr + bloom * uBloomStr);
    // Correção gamma sRGB
    col = pow(col, vec3(1.0/2.2));
    FragColor  = vec4(col, 1.0);
}
)glsl";

// ── Helpers ───────────────────────────────────────────────────────────────────

static GLuint mkShader(GLenum t, const char* s){
    GLuint sh = glCreateShader(t);
    glShaderSource(sh, 1, &s, nullptr);
    glCompileShader(sh);
    GLint ok = 0; glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok){ char b[512]; glGetShaderInfoLog(sh,512,nullptr,b); fprintf(stderr,"[PostFX] %s\n",b); }
    return sh;
}
static GLuint mkProg(const char* vs, const char* fs){
    GLuint v=mkShader(GL_VERTEX_SHADER,vs), f=mkShader(GL_FRAGMENT_SHADER,fs);
    GLuint p=glCreateProgram();
    glAttachShader(p,v); glAttachShader(p,f); glLinkProgram(p);
    glDeleteShader(v); glDeleteShader(f);
    return p;
}

// ── create / destroy ──────────────────────────────────────────────────────────

void PostFX::create(int w, int h){
    _w = w; _h = h;
    int bw = std::max(1, w/2), bh = std::max(1, h/2);

    // FBO principal HDR (RGB16F + depth renderbuffer)
    glGenFramebuffers(1, &_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, _fbo);

    glGenTextures(1, &_col);
    glBindTexture(GL_TEXTURE_2D, _col);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _col, 0);

    glGenRenderbuffers(1, &_dep);
    glBindRenderbuffer(GL_RENDERBUFFER, _dep);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, w, h);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, _dep);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        fprintf(stderr, "[PostFX] FBO principal incompleto!\n");

    // FBOs de bloom (ping-pong, meia resolução)
    for (int i = 0; i < 2; ++i){
        glGenFramebuffers(1, &_bFBO[i]);
        glBindFramebuffer(GL_FRAMEBUFFER, _bFBO[i]);
        glGenTextures(1, &_bTex[i]);
        glBindTexture(GL_TEXTURE_2D, _bTex[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, bw, bh, 0, GL_RGB, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _bTex[i], 0);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void PostFX::destroy(){
    if (_fbo) { glDeleteFramebuffers(1,&_fbo); _fbo=0; }
    if (_col) { glDeleteTextures(1,&_col);      _col=0; }
    if (_dep) { glDeleteRenderbuffers(1,&_dep); _dep=0; }
    for (int i=0;i<2;i++){
        if (_bFBO[i]) { glDeleteFramebuffers(1,&_bFBO[i]); _bFBO[i]=0; }
        if (_bTex[i]) { glDeleteTextures(1,&_bTex[i]);     _bTex[i]=0; }
    }
}

// ── API pública ───────────────────────────────────────────────────────────────

bool PostFX::init(){
    // Quad de tela cheia (−1..+1)
    static const float verts[] = {-1,-1, 1,-1, 1,1, -1,-1, 1,1, -1,1};
    glGenVertexArrays(1, &_qVAO); glGenBuffers(1, &_qVBO);
    glBindVertexArray(_qVAO);
    glBindBuffer(GL_ARRAY_BUFFER, _qVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    _pThresh = mkProg(Q_VERT, THRESH_FRAG);
    _pBlur   = mkProg(Q_VERT, BLUR_FRAG);
    _pComp   = mkProg(Q_VERT, COMP_FRAG);

    _ok = true;
    return true;
}

void PostFX::bind(int w, int h){
    if (!_ok) return;
    if (w != _w || h != _h){ destroy(); create(w, h); }
    else if (!_fbo)          create(w, h);
    glBindFramebuffer(GL_FRAMEBUFFER, _fbo);
    glViewport(0, 0, _w, _h);
}

void PostFX::quad(){
    glBindVertexArray(_qVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
}

void PostFX::resolve(float bloomStr, float exposure, float threshold){
    if (!_ok || !_fbo) return;

    int bw = std::max(1, _w/2), bh = std::max(1, _h/2);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    // ── 1. Threshold → bFBO[0] ───────────────────────────────────────────────
    glBindFramebuffer(GL_FRAMEBUFFER, _bFBO[0]);
    glViewport(0, 0, bw, bh);
    glUseProgram(_pThresh);
    glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, _col);
    glUniform1i(glGetUniformLocation(_pThresh,"uScene"), 0);
    glUniform1f(glGetUniformLocation(_pThresh,"uThreshold"), threshold);
    quad();

    // ── 2. Blur ping-pong (4 passes H+V = 2 iterações) ───────────────────────
    bool horiz = true;
    for (int i = 0; i < 4; ++i){
        glBindFramebuffer(GL_FRAMEBUFFER, _bFBO[!horiz]);
        glUseProgram(_pBlur);
        glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, _bTex[horiz]);
        glUniform1i(glGetUniformLocation(_pBlur,"uTex"), 0);
        glUniform2f(glGetUniformLocation(_pBlur,"uDir"),
                    horiz ? 1.f : 0.f, horiz ? 0.f : 1.f);
        quad();
        horiz = !horiz;
    }

    // ── 3. Composite para tela ────────────────────────────────────────────────
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, _w, _h);
    glUseProgram(_pComp);
    glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, _col);
    glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, _bTex[0]);
    glUniform1i(glGetUniformLocation(_pComp,"uScene"),    0);
    glUniform1i(glGetUniformLocation(_pComp,"uBloom"),    1);
    glUniform1f(glGetUniformLocation(_pComp,"uBloomStr"), bloomStr);
    glUniform1f(glGetUniformLocation(_pComp,"uExposure"), exposure);
    quad();

    // Restaura estado
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glActiveTexture(GL_TEXTURE0);
}

void PostFX::cleanup(){
    destroy();
    if (_qVAO) glDeleteVertexArrays(1, &_qVAO);
    if (_qVBO) glDeleteBuffers(1, &_qVBO);
    if (_pThresh) glDeleteProgram(_pThresh);
    if (_pBlur)   glDeleteProgram(_pBlur);
    if (_pComp)   glDeleteProgram(_pComp);
}
