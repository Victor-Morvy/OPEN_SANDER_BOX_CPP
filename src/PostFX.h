#pragma once
#include <glad/glad.h>

// Bloom + ACES tone-mapping em dois FBOs ping-pong (metade da resolução).
// Uso:
//   postfx.bind(fw, fh);           // antes de qualquer render 3D
//   glClear(...);  sky/terrain/... // render normal
//   postfx.resolve(bloomStr, exp); // antes do ImGui
class PostFX {
public:
    bool init();
    void bind(int w, int h);    // redireciona para FBO HDR; auto-resize
    void resolve(float bloomStr, float exposure, float threshold = 0.82f);
    void cleanup();

private:
    bool   _ok  = false;
    GLuint _fbo = 0, _col = 0, _dep = 0;   // cena HDR principal
    GLuint _bFBO[2] = {}, _bTex[2] = {};   // ping-pong blur (meia resolução)
    GLuint _pThresh = 0, _pBlur = 0, _pComp = 0;
    GLuint _qVAO = 0, _qVBO = 0;
    int    _w = 0, _h = 0;

    void create(int w, int h);
    void destroy();
    void quad();
};
