#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include "TileManager.h"
#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <curl/curl.h>

#include <cmath>
#include <cstdio>
#include <algorithm>
#include <atomic>

// ── Constantes ────────────────────────────────────────────────────────────────

static constexpr double M_PI_       = 3.14159265358979323846;
static constexpr double DEG2RAD     = M_PI_ / 180.0;
static constexpr double M_PER_DEG_LAT = 111320.0;
static const char* URL_ELEV = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png";
static const char* URL_TEX  = "https://services.arcgisonline.com/arcgis/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";

// ── Curl: init/cleanup global com ref-count (suporta múltiplas instâncias) ───

static std::atomic<int> s_curlRefs{0};
static void curlGlobalInit()   { if(s_curlRefs++ == 0) curl_global_init(CURL_GLOBAL_DEFAULT); }
static void curlGlobalCleanup(){ if(--s_curlRefs == 0) curl_global_cleanup(); }

// ── Shaders ───────────────────────────────────────────────────────────────────

static const char* TM_VERT = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec2 aUV;
out vec2  vUV;
out vec3  vWorld;
uniform mat4  uVP;
uniform vec3  uTileOrig;
uniform vec3  uAcWorld;
uniform float uYBias;   // deslocamento Y para evitar z-fighting entre LODs
void main() {
    vec3 world = vec3(aPos.x + uTileOrig.x, aPos.y + uYBias, aPos.z + uTileOrig.z);
    gl_Position = uVP * vec4(world - uAcWorld, 1.0);
    vUV    = aUV;
    vWorld = world;
}
)glsl";

static const char* TM_FRAG = R"glsl(
#version 330 core
in vec2 vUV;
in vec3 vWorld;
out vec4 FragColor;
uniform sampler2D uTex;
uniform bool      uHasTex;
uniform vec3      uSunDir;
uniform float     uDay;
uniform vec3      uAcWorld;
void main() {
    vec3 col;
    if(uHasTex) {
        col = pow(texture(uTex, vUV).rgb, vec3(2.2));
    } else {
        vec2 cell = floor(vWorld.xz / 1000.0);
        float e = mod(cell.x + cell.y, 2.0);
        col = mix(vec3(.28,.40,.21), vec3(.40,.33,.19), e);
    }
    float diff = clamp(uSunDir.y * 0.6 + 0.4, 0.0, 1.0);
    col *= mix(0.08, diff, uDay);
    // Fog baseado em distância do avião (reduzido para visibilidade maior)
    float d   = length(vWorld.xz - uAcWorld.xz);
    float fog = exp(-d * 0.000006);
    vec3  fc  = mix(vec3(.05,.06,.08), vec3(.58,.66,.80), uDay);
    col = mix(fc, col, clamp(fog, 0.0, 1.0));
    FragColor = vec4(pow(col, vec3(1.0/2.2)), 1.0);
}
)glsl";

// ── Helpers ───────────────────────────────────────────────────────────────────

static GLuint buildShader(GLenum t, const char* s) {
    GLuint sh = glCreateShader(t);
    glShaderSource(sh,1,&s,nullptr); glCompileShader(sh);
    GLint ok=0; glGetShaderiv(sh,GL_COMPILE_STATUS,&ok);
    if(!ok){char b[512];glGetShaderInfoLog(sh,512,nullptr,b);fprintf(stderr,"[TileShader] %s\n",b);}
    return sh;
}

static std::string buildUrl(const char* tmpl, int z, int x, int y) {
    std::string s(tmpl);
    auto rep=[&](const std::string& from, int v){
        size_t p=s.find(from);
        if(p!=std::string::npos) s.replace(p,from.size(),std::to_string(v));
    };
    rep("{z}",z); rep("{x}",x); rep("{y}",y);
    return s;
}

static size_t curlWrite(char* p, size_t s, size_t n, void* d){
    auto* v=(std::vector<uint8_t>*)d; v->insert(v->end(),p,p+s*n); return s*n;
}
std::vector<uint8_t> TileManager::httpGet(const std::string& url){
    std::vector<uint8_t> data;
    CURL* c=curl_easy_init(); if(!c) return data;
    curl_easy_setopt(c,CURLOPT_URL,url.c_str());
    curl_easy_setopt(c,CURLOPT_WRITEFUNCTION,curlWrite);
    curl_easy_setopt(c,CURLOPT_WRITEDATA,&data);
    curl_easy_setopt(c,CURLOPT_FOLLOWLOCATION,1L);
    curl_easy_setopt(c,CURLOPT_SSL_VERIFYPEER,0L);
    curl_easy_setopt(c,CURLOPT_TIMEOUT,15L);
    curl_easy_setopt(c,CURLOPT_USERAGENT,"webflight-cpp/1.0");
    CURLcode rc=curl_easy_perform(c);
    if(rc!=CURLE_OK) fprintf(stderr,"[curl] %s\n",curl_easy_strerror(rc));
    curl_easy_cleanup(c);
    return data;
}

// ── Mercator ──────────────────────────────────────────────────────────────────

std::pair<int,int> TileManager::latLonToTile(double lat, double lon, int zoom){
    double n=(double)(1<<zoom);
    int tx=(int)std::floor((lon+180.0)/360.0*n);
    double lr=lat*DEG2RAD;
    int ty=(int)std::floor((1.0-std::log(std::tan(lr)+1.0/std::cos(lr))/M_PI_)/2.0*n);
    int nmax=(int)n-1;
    return {std::max(0,std::min(nmax,tx)), std::max(0,std::min(nmax,ty))};
}

std::pair<double,double> TileManager::tileNWLatLon(int tx, int ty, int zoom){
    double n=(double)(1<<zoom);
    double lon=tx/n*360.0-180.0;
    double lat=std::atan(std::sinh(M_PI_*(1.0-2.0*ty/n)))/DEG2RAD;
    return {lat,lon};
}

// ── Tile load (worker thread) ─────────────────────────────────────────────────

TileManager::TileData TileManager::loadTile(TileKey key,
                                              double originLat, double originLon,
                                              int vgrid)
{
    TileData d; d.key=key;
    int z=key.z, tx=key.x, ty=key.y;

    auto elevPng=httpGet(buildUrl(URL_ELEV,z,tx,ty));
    auto texPng =httpGet(buildUrl(URL_TEX, z,tx,ty));

    auto [nwLat,nwLon]=tileNWLatLon(tx,  ty,  z);
    auto [seLat,seLon]=tileNWLatLon(tx+1,ty+1,z);

    double avgLat    = (nwLat+seLat)*0.5;
    double mPerDegLon= M_PER_DEG_LAT*std::cos(avgLat*DEG2RAD);
    float  tileSzX   =(float)((seLon-nwLon)*mPerDegLon);
    float  tileSzZ   =(float)((nwLat-seLat)*M_PER_DEG_LAT);
    float  worldX    =(float)((nwLon-originLon)*mPerDegLon);
    float  worldZ    =-(float)((nwLat-originLat)*M_PER_DEG_LAT);

    d.worldX=worldX; d.worldZ=worldZ;
    d.tileSzX=tileSzX; d.tileSzZ=tileSzZ;

    // Elevação VGRID×VGRID amostrada do PNG Terrarium
    int N=vgrid;
    d.elev.resize(N*N, 0.f);
    if(!elevPng.empty()){
        int w,h,c;
        uint8_t* pix=stbi_load_from_memory(elevPng.data(),(int)elevPng.size(),&w,&h,&c,3);
        if(pix){
            for(int iy=0;iy<N;iy++) for(int ix=0;ix<N;ix++){
                float u=(float)ix/(N-1)*(w-1);
                float v=(float)iy/(N-1)*(h-1);
                int px=std::min((int)u,w-1), py=std::min((int)v,h-1);
                int idx=(py*w+px)*3;
                float r=pix[idx],g=pix[idx+1],b=pix[idx+2];
                d.elev[iy*N+ix]=r*256.f+g+b/256.f-32768.f;
            }
            stbi_image_free(pix);
        }
    }

    // Aplica áreas planas (pistas) ao grid de elevação antes de construir vértices
    {
        std::vector<FlatArea> fas;
        { std::lock_guard<std::mutex> lk(_flatMtx); fas = _flatAreas; }
        if (!fas.empty()) {
            auto sm = [](float lo, float hi, float x) -> float {
                x = std::max(0.f, std::min(1.f, (x-lo)/(hi-lo)));
                return x*x*(3.f-2.f*x);
            };
            for (int iy=0; iy<N; iy++) for (int ix=0; ix<N; ix++) {
                float vx = (float)worldX + (float)ix/(N-1)*tileSzX;
                float vz = (float)worldZ + (float)iy/(N-1)*tileSzZ;
                float vy = d.elev[iy*N+ix];
                for (const auto& fa : fas) {
                    float da = (vx-fa.cx)*fa.ux + (vz-fa.cz)*fa.uz;
                    float dp = (vx-fa.cx)*fa.px + (vz-fa.cz)*fa.pz;
                    float absA = std::abs(da), absP = std::abs(dp);
                    if (absA > fa.halfLen+fa.blendR || absP > fa.halfWid+fa.blendR)
                        continue;
                    float t   = std::max(0.f, std::min(1.f, (da+fa.halfLen)/(2.f*fa.halfLen)));
                    float tgt = fa.leElev + (fa.heElev - fa.leElev)*t;
                    float bA  = absA < fa.halfLen ? 1.f : sm(fa.halfLen+fa.blendR, fa.halfLen, absA);
                    float bP  = absP < fa.halfWid ? 1.f : sm(fa.halfWid+fa.blendR, fa.halfWid, absP);
                    vy = vy + (tgt - vy) * bA * bP;
                }
                d.elev[iy*N+ix] = vy;
            }
        }
    }

    // Vertices: (x,y,z, u,v) — x=east, y=elev MSL, z=south (metros desde NW)
    d.verts.reserve(N*N*5);
    for(int iy=0;iy<N;iy++) for(int ix=0;ix<N;ix++){
        float u=(float)ix/(N-1), v=(float)iy/(N-1);
        d.verts.push_back(u*tileSzX);
        d.verts.push_back(d.elev[iy*N+ix]);
        d.verts.push_back(v*tileSzZ);
        d.verts.push_back(u);
        d.verts.push_back(v);
    }

    // Índices (N-1)×(N-1) quads
    int Q=N-1;
    d.idx.reserve(Q*Q*6 + 4*(N-1)*6);
    for(int iy=0;iy<Q;iy++) for(int ix=0;ix<Q;ix++){
        unsigned short a=iy*N+ix, b=a+1, c2=a+N, dd=c2+1;
        d.idx.push_back(a);  d.idx.push_back(c2); d.idx.push_back(b);
        d.idx.push_back(b);  d.idx.push_back(c2); d.idx.push_back(dd);
    }

    // Saias (skirts) para esconder valas nas bordas entre tiles.
    // Para cada borda, adiciona N vértices rebaixados 30 m e conecta com a borda.
    constexpr float SKIRT = 30.f;
    int sBase = N*N;  // índice base dos vértices de saia

    // Borda norte (iy=0, z=0)
    for(int ix=0;ix<N;ix++){
        float u=(float)ix/(N-1);
        d.verts.push_back(u*tileSzX); d.verts.push_back(d.elev[ix]-SKIRT);
        d.verts.push_back(0.f);       d.verts.push_back(u); d.verts.push_back(0.f);
    }
    // Borda sul (iy=N-1, z=tileSzZ)
    for(int ix=0;ix<N;ix++){
        float u=(float)ix/(N-1);
        d.verts.push_back(u*tileSzX); d.verts.push_back(d.elev[(N-1)*N+ix]-SKIRT);
        d.verts.push_back(tileSzZ);   d.verts.push_back(u); d.verts.push_back(1.f);
    }
    // Borda oeste (ix=0, x=0)
    for(int iy=0;iy<N;iy++){
        float v=(float)iy/(N-1);
        d.verts.push_back(0.f);       d.verts.push_back(d.elev[iy*N]-SKIRT);
        d.verts.push_back(v*tileSzZ); d.verts.push_back(0.f); d.verts.push_back(v);
    }
    // Borda leste (ix=N-1, x=tileSzX)
    for(int iy=0;iy<N;iy++){
        float v=(float)iy/(N-1);
        d.verts.push_back(tileSzX);   d.verts.push_back(d.elev[iy*N+(N-1)]-SKIRT);
        d.verts.push_back(v*tileSzZ); d.verts.push_back(1.f); d.verts.push_back(v);
    }

    // Quads de saia (borda ↔ saia)
    auto sk = [&](int eA, int eB, int sA, int sB){
        d.idx.push_back((unsigned short)eA); d.idx.push_back((unsigned short)sA); d.idx.push_back((unsigned short)eB);
        d.idx.push_back((unsigned short)eB); d.idx.push_back((unsigned short)sA); d.idx.push_back((unsigned short)sB);
    };
    for(int i=0;i<N-1;i++){
        sk(    i,     i+1, sBase+i,   sBase+i+1);   // norte
        sk((N-1)*N+i,(N-1)*N+i+1,sBase+N+i,sBase+N+i+1);  // sul
        sk(i*N,   (i+1)*N, sBase+2*N+i, sBase+2*N+i+1);    // oeste
        sk(i*N+(N-1),(i+1)*N+(N-1),sBase+3*N+i,sBase+3*N+i+1); // leste
    }

    // Textura satélite
    if(!texPng.empty()){
        int w,h,c;
        uint8_t* pix=stbi_load_from_memory(texPng.data(),(int)texPng.size(),&w,&h,&c,3);
        if(pix){ d.tex.assign(pix,pix+w*h*3); d.texW=w; d.texH=h; stbi_image_free(pix); }
    }

    d.ok=true;
    return d;
}

// ── GPU upload (main thread) ──────────────────────────────────────────────────

void TileManager::uploadTile(TileData& d){
    GpuTile g;
    g.worldX=d.worldX; g.worldZ=d.worldZ;
    g.tileSzX=d.tileSzX; g.tileSzZ=d.tileSzZ;
    g.elev=d.elev; g.elevGridN=(int)std::sqrt((double)d.elev.size());
    g.idxCount=(int)d.idx.size();

    glGenVertexArrays(1,&g.vao); glGenBuffers(1,&g.vbo); glGenBuffers(1,&g.ebo);
    glBindVertexArray(g.vao);
      glBindBuffer(GL_ARRAY_BUFFER,g.vbo);
      glBufferData(GL_ARRAY_BUFFER,d.verts.size()*4,d.verts.data(),GL_STATIC_DRAW);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,g.ebo);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER,d.idx.size()*2,d.idx.data(),GL_STATIC_DRAW);
      glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,20,(void*)0);   glEnableVertexAttribArray(0);
      glVertexAttribPointer(1,2,GL_FLOAT,GL_FALSE,20,(void*)12);  glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    glGenTextures(1,&g.tex);
    glBindTexture(GL_TEXTURE_2D,g.tex);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
    if(!d.tex.empty()){
        glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,d.texW,d.texH,0,GL_RGB,GL_UNSIGNED_BYTE,d.tex.data());
        glGenerateMipmap(GL_TEXTURE_2D);
    } else {
        uint8_t w[3]={255,255,255};
        glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,1,1,0,GL_RGB,GL_UNSIGNED_BYTE,w);
    }

    _gpu[d.key]=std::move(g);
}

void TileManager::processUploads(){
    std::vector<TileData> batch;
    { std::lock_guard<std::mutex> lk(_uploadMtx); batch.swap(_uploadQueue); }
    for(auto& d:batch) if(d.ok) uploadTile(d);

    for(auto it=_loading.begin(); it!=_loading.end();){
        if(it->second.wait_for(std::chrono::seconds(0))==std::future_status::ready){
            auto d=it->second.get();
            if(d.ok) uploadTile(d);
            it=_loading.erase(it);
        } else ++it;
    }
}

// ── API pública ───────────────────────────────────────────────────────────────

bool TileManager::init(double originLat, double originLon,
                        int zoom, int grid, int vgrid, float yBias)
{
    _zoom=zoom; _grid=grid; _vgrid=vgrid; _yBias=yBias;
    curlGlobalInit();

    GLuint vs=buildShader(GL_VERTEX_SHADER,TM_VERT);
    GLuint fs=buildShader(GL_FRAGMENT_SHADER,TM_FRAG);
    _prog=glCreateProgram();
    glAttachShader(_prog,vs); glAttachShader(_prog,fs); glLinkProgram(_prog);
    glDeleteShader(vs); glDeleteShader(fs);

    printf("[TileManager] zoom=%d  grid=%dx%d  vgrid=%d×%d  (%d tiles)\n",
           zoom,(2*grid+1),(2*grid+1),vgrid,vgrid,(2*grid+1)*(2*grid+1));
    (void)originLat; (void)originLon;
    return true;
}

void TileManager::registerFlatArea(float cx, float cz, float halfWid, float halfLen,
                                    float heading, float leElev, float heElev, float blendR)
{
    FlatArea fa;
    fa.cx = cx; fa.cz = cz;
    fa.halfWid = halfWid; fa.halfLen = halfLen;
    fa.ux = std::sin(heading); fa.uz = std::cos(heading);
    fa.px = fa.uz;            fa.pz = -fa.ux;
    fa.leElev = leElev; fa.heElev = heElev;
    fa.blendR = blendR;
    std::lock_guard<std::mutex> lk(_flatMtx);
    _flatAreas.push_back(fa);
    _pendingEvict.push_back(fa);
}

void TileManager::evictPendingAreas() {
    std::vector<FlatArea> toEvict;
    {
        std::lock_guard<std::mutex> lk(_flatMtx);
        if (_pendingEvict.empty()) return;
        toEvict.swap(_pendingEvict);
    }
    // Remove tiles carregados que se sobrepõem a qualquer área pendente
    // (força recarga com o flat area aplicado)
    for (auto it = _gpu.begin(); it != _gpu.end(); ) {
        const GpuTile& g = it->second;
        bool hit = false;
        for (const auto& fa : toEvict) {
            float r = fa.halfLen + fa.blendR;
            if (g.worldX + g.tileSzX > fa.cx - r && g.worldX < fa.cx + r &&
                g.worldZ + g.tileSzZ > fa.cz - r && g.worldZ < fa.cz + r) {
                hit = true; break;
            }
        }
        if (hit) {
            glDeleteVertexArrays(1,&it->second.vao);
            glDeleteBuffers(1,&it->second.vbo);
            glDeleteBuffers(1,&it->second.ebo);
            glDeleteTextures(1,&it->second.tex);
            it = _gpu.erase(it);
        } else ++it;
    }
}

void TileManager::update(glm::vec3 acWorld, double originLat, double originLon){
    evictPendingAreas();
    double mPerDegLon=M_PER_DEG_LAT*std::cos(originLat*DEG2RAD);
    double lat=originLat+(-acWorld.z)/M_PER_DEG_LAT;
    double lon=originLon+  acWorld.x /mPerDegLon;

    auto [ctx,cty]=latLonToTile(lat,lon,_zoom);
    bool changed=(ctx!=_centerTx||cty!=_centerTy);
    _centerTx=ctx; _centerTy=cty;

    if(changed){
        // Tiles necessários
        std::unordered_map<TileKey,bool,TileKeyHash> needed;
        int nmax=(1<<_zoom)-1;
        for(int dy=-_grid;dy<=_grid;dy++) for(int dx=-_grid;dx<=_grid;dx++){
            TileKey k{_zoom,
                      std::max(0,std::min(nmax,ctx+dx)),
                      std::max(0,std::min(nmax,cty+dy))};
            needed[k]=true;
            if(!_gpu.count(k)&&!_loading.count(k)){
                int vg=_vgrid;
                _loading[k]=std::async(std::launch::async,
                    [this,k,originLat,originLon,vg]{
                        return loadTile(k,originLat,originLon,vg);
                    });
            }
        }
        // Remove tiles fora de alcance
        for(auto it=_gpu.begin(); it!=_gpu.end();){
            if(!needed.count(it->first)){
                glDeleteVertexArrays(1,&it->second.vao);
                glDeleteBuffers(1,&it->second.vbo);
                glDeleteBuffers(1,&it->second.ebo);
                glDeleteTextures(1,&it->second.tex);
                it=_gpu.erase(it);
            } else ++it;
        }
    }
    processUploads();
}

void TileManager::render(const glm::mat4& VP,
                          glm::vec3 acWorld, float acMslM,
                          const glm::vec3& sunDir, float day)
{
    processUploads();
    glUseProgram(_prog);
    glm::vec3 acFull(acWorld.x, acMslM, acWorld.z);
    int uVP      =glGetUniformLocation(_prog,"uVP");
    int uTileOrig=glGetUniformLocation(_prog,"uTileOrig");
    int uAcWorld =glGetUniformLocation(_prog,"uAcWorld");
    int uSunDir  =glGetUniformLocation(_prog,"uSunDir");
    int uDay     =glGetUniformLocation(_prog,"uDay");
    int uTex     =glGetUniformLocation(_prog,"uTex");
    int uHasTex  =glGetUniformLocation(_prog,"uHasTex");
    int uYBias   =glGetUniformLocation(_prog,"uYBias");

    glUniformMatrix4fv(uVP,1,GL_FALSE,glm::value_ptr(VP));
    glUniform3fv(uAcWorld,1,glm::value_ptr(acFull));
    glUniform3fv(uSunDir, 1,glm::value_ptr(sunDir));
    glUniform1f (uDay,day);
    glUniform1i (uTex,0);
    glUniform1f (uYBias,_yBias);

    for(auto& [key,g]:_gpu){
        glm::vec3 orig(g.worldX,0.f,g.worldZ);
        glUniform3fv(uTileOrig,1,glm::value_ptr(orig));
        glUniform1i(uHasTex, g.tex!=0 ? 1 : 0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D,g.tex);
        glBindVertexArray(g.vao);
        glDrawElements(GL_TRIANGLES,g.idxCount,GL_UNSIGNED_SHORT,nullptr);
        glBindVertexArray(0);
    }
}

float TileManager::getElevAt(glm::vec3 acWorld) const{
    for(auto& [key,g]:_gpu){
        float dx=acWorld.x-g.worldX;
        float dz=acWorld.z-g.worldZ;
        if(dx>=0&&dx<=g.tileSzX&&dz>=0&&dz<=g.tileSzZ){
            int N=g.elevGridN;
            float u=dx/g.tileSzX*(N-1), v=dz/g.tileSzZ*(N-1);
            int ix=std::max(0,std::min(N-2,(int)u));
            int iy=std::max(0,std::min(N-2,(int)v));
            float fu=u-ix, fv=v-iy;
            float e00=g.elev[iy*N+ix],   e10=g.elev[iy*N+ix+1];
            float e01=g.elev[(iy+1)*N+ix],e11=g.elev[(iy+1)*N+ix+1];
            return e00*(1-fu)*(1-fv)+e10*fu*(1-fv)+e01*(1-fu)*fv+e11*fu*fv;
        }
    }
    return 0.f;
}

void TileManager::cleanup(){
    for(auto& [k,g]:_gpu){
        glDeleteVertexArrays(1,&g.vao); glDeleteBuffers(1,&g.vbo);
        glDeleteBuffers(1,&g.ebo); glDeleteTextures(1,&g.tex);
    }
    _gpu.clear();
    if(_prog){ glDeleteProgram(_prog); _prog=0; }
    curlGlobalCleanup();
}
