#include "OSMManager.h"
#include "TileManager.h"

#include <nlohmann/json.hpp>
#include <mapbox/earcut.hpp>
#include <curl/curl.h>
#include <glm/gtc/type_ptr.hpp>

#include <cstdio>
#include <cmath>
#include <cstring>
#include <ctime>
#include <sys/stat.h>
#include <algorithm>
#include <filesystem>

#ifdef _WIN32
#  define stat _stat
#endif

// ── Shaders ───────────────────────────────────────────────────────────────────

static const char* WALL_VERT = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aUV;
uniform mat4 uVP;
uniform float uBaseY;
out vec3 vNormal;
out vec2 vUV;
void main(){
    gl_Position = uVP * vec4(aPos.x, aPos.y + uBaseY, aPos.z, 1.0);
    vNormal = aNormal;
    vUV = aUV;
}
)glsl";

static const char* WALL_FRAG = R"glsl(
#version 330 core
in vec3 vNormal; in vec2 vUV;
out vec4 FragColor;
uniform sampler2D uTex;
uniform float uDay;
void main(){
    vec3 sun  = normalize(vec3(0.6, 0.9, 0.4));
    float diff = max(dot(normalize(vNormal), sun), 0.08);
    float amb  = mix(0.25, 1.0, uDay);
    vec3 col   = texture(uTex, vUV).rgb * (diff * amb + 0.05);
    FragColor  = vec4(col, 1.0);
}
)glsl";

static const char* FLAT_VERT = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uVP;
uniform float uBaseY;
void main(){
    gl_Position = uVP * vec4(aPos.x, aPos.y + uBaseY, aPos.z, 1.0);
}
)glsl";

static const char* FLAT_FRAG = R"glsl(
#version 330 core
out vec4 FragColor;
uniform vec3  uColor;
uniform float uDay;
void main(){
    FragColor = vec4(uColor * mix(0.2, 1.0, uDay), 1.0);
}
)glsl";

static const char* WATER_VERT = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4  uVP;
uniform float uBaseY;
out vec3 vWorld;
void main(){
    vWorld      = vec3(aPos.x, aPos.y + uBaseY, aPos.z);
    gl_Position = uVP * vec4(vWorld, 1.0);
}
)glsl";

static const char* WATER_FRAG = R"glsl(
#version 330 core
in vec3 vWorld;
out vec4 FragColor;
uniform float uTime;
uniform float uDay;
uniform vec3  uCamPos;

float hash(vec2 p){ return fract(sin(dot(p,vec2(127.1,311.7)))*43758.5453); }
float noise(vec2 p){
    vec2 i=floor(p), f=fract(p);
    f=f*f*(3.0-2.0*f);
    return mix(mix(hash(i),hash(i+vec2(1,0)),f.x),
               mix(hash(i+vec2(0,1)),hash(i+vec2(1,1)),f.x),f.y);
}
float fbm(vec2 p){
    float v=0.,amp=0.5;
    for(int i=0;i<4;i++){ v+=amp*noise(p); p=p*2.1+vec2(1.7,9.2); amp*=0.5; }
    return v;
}
void main(){
    vec2 uv = vWorld.xz * 0.02;
    float wave = fbm(uv + vec2(uTime*0.05, uTime*0.03)) * 0.6
               + fbm(uv*2.0 + 1.7 + vec2(uTime*0.07, -uTime*0.04)) * 0.3
               + fbm(uv*4.0) * 0.1;
    wave = clamp(wave, 0.0, 1.0);

    vec3 deep    = vec3(0.05, 0.15, 0.35);
    vec3 shallow = vec3(0.15, 0.45, 0.65);
    vec3 foam    = vec3(0.75, 0.90, 1.0);
    vec3 col = mix(deep, shallow, wave);
    col = mix(col, foam, pow(wave, 8.0) * 0.35);
    col *= mix(0.3, 1.0, uDay);

    vec3 toC = normalize(uCamPos - vWorld);
    vec3 R   = reflect(normalize(vec3(-0.6, 0.9, -0.4)), vec3(0,1,0));
    float spec = pow(max(dot(R, toC), 0.0), 48.0) * 0.5 * uDay;
    col += spec;

    FragColor = vec4(col, 0.85);
}
)glsl";

// ── Helpers ───────────────────────────────────────────────────────────────────

static GLuint buildShader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetShaderInfoLog(s, 512, nullptr, log);
        fprintf(stderr, "[OSM] shader error: %s\n", log);
    }
    return s;
}

GLuint OSMManager::compileShader(const char* vert, const char* frag) {
    GLuint v = buildShader(GL_VERTEX_SHADER, vert);
    GLuint f = buildShader(GL_FRAGMENT_SHADER, frag);
    GLuint p = glCreateProgram();
    glAttachShader(p, v); glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v); glDeleteShader(f);
    return p;
}

static size_t curlWrite(char* ptr, size_t, size_t n, void* ud) {
    ((std::string*)ud)->append(ptr, n);
    return n;
}

static std::string urlEncode(const std::string& s) {
    CURL* c = curl_easy_init();
    if (!c) return s;
    char* enc = curl_easy_escape(c, s.c_str(), (int)s.size());
    std::string r = enc ? enc : s;
    if (enc) curl_free(enc);
    curl_easy_cleanup(c);
    return r;
}

// ── Coordinate conversion ─────────────────────────────────────────────────────

glm::vec2 OSMManager::toWorld(double lat, double lon) const {
    return {
        (float)((lon - _originLon) * _mPerDegLon),
        (float)(-(lat - _originLat) * 111320.0)
    };
}

// ── init ──────────────────────────────────────────────────────────────────────

bool OSMManager::init(double originLat, double originLon,
                      const std::string& cacheDir) {
    _originLat  = originLat;
    _originLon  = originLon;
    _mPerDegLon = 111320.0 * std::cos(originLat * 3.14159265358979 / 180.0);
    _cacheDir   = cacheDir;

    std::error_code ec;
    std::filesystem::create_directories(cacheDir, ec);

    _wallProg  = compileShader(WALL_VERT,  WALL_FRAG);
    _flatProg  = compileShader(FLAT_VERT,  FLAT_FRAG);
    _waterProg = compileShader(WATER_VERT, WATER_FRAG);

    genFacadeTextures();
    printf("[OSM] init ok (origin %.4f, %.4f)\n", originLat, originLon);
    return true;
}

// ── Facade textures ───────────────────────────────────────────────────────────

void OSMManager::genFacadeTextures() {
    const int W = 128, H = 128;
    // 5 wall colors: warm gray, approximate hsl(35, 8%, L%)
    const uint8_t WR[5] = {191, 196, 181, 203, 173};
    const uint8_t WG[5] = {186, 191, 176, 198, 168};
    const uint8_t WB[5] = {178, 183, 168, 190, 161};

    const int COLS=4, ROWS=6;
    const int CW = W/COLS, CH = H/ROWS;  // 32, 21

    std::vector<uint8_t> pix(W * H * 3);

    for (int v = 0; v < 5; v++) {
        // Wall fill
        for (int i = 0; i < W*H; i++) {
            pix[i*3]=WR[v]; pix[i*3+1]=WG[v]; pix[i*3+2]=WB[v];
        }
        // Windows
        for (int col = 0; col < COLS; col++) {
            for (int row = 0; row < ROWS; row++) {
                int wx = col*CW + 6, wy = row*CH + 3;
                int ww = CW - 12,    wh = CH - 9;
                bool lit = ((v*7 + col*3 + row*5) % 5) != 0;
                uint8_t wr = lit ? 204 : 26;
                uint8_t wg = lit ? 232 : 37;
                uint8_t wb = lit ? 244 : 53;
                for (int py = wy; py < wy+wh && py < H; py++) {
                    for (int px = wx; px < wx+ww && px < W; px++) {
                        pix[(py*W+px)*3]   = wr;
                        pix[(py*W+px)*3+1] = wg;
                        pix[(py*W+px)*3+2] = wb;
                    }
                }
                // Subtle frame (1px, slightly lighter than wall)
                auto setF = [&](int px, int py) {
                    if (px<0||px>=W||py<0||py>=H) return;
                    pix[(py*W+px)*3]   = 220;
                    pix[(py*W+px)*3+1] = 220;
                    pix[(py*W+px)*3+2] = 220;
                };
                for (int px = wx-1; px <= wx+ww; px++) { setF(px,wy-1); setF(px,wy+wh); }
                for (int py = wy;   py <  wy+wh; py++) { setF(wx-1,py); setF(wx+ww,py); }
            }
        }
        glGenTextures(1, &_facadeTex[v]);
        glBindTexture(GL_TEXTURE_2D, _facadeTex[v]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, W, H, 0, GL_RGB, GL_UNSIGNED_BYTE, pix.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
}

// ── update ────────────────────────────────────────────────────────────────────

static constexpr double CELL_DEG = 0.05;  // ~5.5 km

void OSMManager::update(double acLat, double acLon,
                        TileManager& close, TileManager& far_) {
    // Upload any pending batch (main thread — GL allowed)
    uploadPending(close, far_);

    int cx = (int)std::floor(acLat / CELL_DEG);
    int cz = (int)std::floor(acLon / CELL_DEG);
    if (cx == _lastCellX && cz == _lastCellZ) return;
    _lastCellX = cx;
    _lastCellZ = cz;

    // Increment generation to cancel in-flight fetch
    ++_gen;
    if (_thread.joinable()) _thread.detach();
    clearMeshes();

    int g = _gen.load();
    _thread = std::thread([this, acLat, acLon, g]{ fetchAsync(acLat, acLon, g); });
}

// ── clearMeshes ───────────────────────────────────────────────────────────────

void OSMManager::clearMeshes() {
    for (auto& m : _meshes) {
        if (m.vao) glDeleteVertexArrays(1, &m.vao);
        if (m.vbo) glDeleteBuffers(1, &m.vbo);
        if (m.ibo) glDeleteBuffers(1, &m.ibo);
    }
    _meshes.clear();
}

// ── uploadPending (main thread) ───────────────────────────────────────────────

void OSMManager::uploadPending(TileManager& close, TileManager& far_) {
    std::lock_guard<std::mutex> lk(_mutex);
    if (!_hasPending) return;
    _hasPending = false;

    for (auto& r : _pending.meshes) {
        GpuMesh g;
        g.type          = r.type;
        g.flatColor     = r.flatColor;
        g.facadeVariant = r.facadeVariant;

        // Sample terrain elevation at centroid (main thread, GL safe)
        glm::vec3 cPos{r.centroidXZ.x, 0.f, r.centroidXZ.y};
        g.baseY = close.getElevAt(cPos);
        if (g.baseY == 0.f) g.baseY = far_.getElevAt(cPos);

        uploadMesh(g, r);
        _meshes.push_back(g);
    }
    _pending.meshes.clear();
    printf("[OSM] uploaded %zu meshes\n", _meshes.size());
}

void OSMManager::uploadMesh(GpuMesh& g, const RawMesh& r) {
    if (r.verts.empty() || r.idx.empty()) return;

    glGenVertexArrays(1, &g.vao);
    glBindVertexArray(g.vao);

    glGenBuffers(1, &g.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(r.verts.size() * sizeof(float)),
                 r.verts.data(), GL_STATIC_DRAW);

    if (r.type == BLDWALL) {
        // stride = 8 floats: pos(xyz) + normal(xyz) + uv(xy)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(0));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(3*sizeof(float)));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(6*sizeof(float)));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
    } else {
        // stride = 3 floats: pos(xyz) only
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    }

    glGenBuffers(1, &g.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 (GLsizeiptr)(r.idx.size() * sizeof(uint32_t)),
                 r.idx.data(), GL_STATIC_DRAW);

    g.indexCount = (uint32_t)r.idx.size();
    glBindVertexArray(0);
}

// ── render ────────────────────────────────────────────────────────────────────

void OSMManager::render(const glm::mat4& VP, const glm::vec3& camPos,
                        float dayT, float timeS) {
    if (_meshes.empty()) return;
    glDisable(GL_CULL_FACE);

    // ── Building walls ────────────────────────────────────────────────────────
    glUseProgram(_wallProg);
    glUniformMatrix4fv(glGetUniformLocation(_wallProg, "uVP"), 1, GL_FALSE,
                       glm::value_ptr(VP));
    glUniform1f(glGetUniformLocation(_wallProg, "uDay"), dayT);
    GLint wallBaseY = glGetUniformLocation(_wallProg, "uBaseY");
    GLint wallTex   = glGetUniformLocation(_wallProg, "uTex");
    glUniform1i(wallTex, 0);

    for (auto& m : _meshes) {
        if (m.type != BLDWALL || m.indexCount == 0) continue;
        glUniform1f(wallBaseY, m.baseY);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, _facadeTex[m.facadeVariant]);
        glBindVertexArray(m.vao);
        glDrawElements(GL_TRIANGLES, (GLsizei)m.indexCount, GL_UNSIGNED_INT, nullptr);
    }

    // ── Roofs & roads (flat) ──────────────────────────────────────────────────
    glUseProgram(_flatProg);
    glUniformMatrix4fv(glGetUniformLocation(_flatProg, "uVP"), 1, GL_FALSE,
                       glm::value_ptr(VP));
    glUniform1f(glGetUniformLocation(_flatProg, "uDay"), dayT);
    GLint flatBaseY = glGetUniformLocation(_flatProg, "uBaseY");
    GLint flatColor = glGetUniformLocation(_flatProg, "uColor");

    for (auto& m : _meshes) {
        if ((m.type != BLDROOF && m.type != ROAD) || m.indexCount == 0) continue;
        glUniform1f(flatBaseY, m.baseY);
        glUniform3fv(flatColor, 1, glm::value_ptr(m.flatColor));
        glBindVertexArray(m.vao);
        glDrawElements(GL_TRIANGLES, (GLsizei)m.indexCount, GL_UNSIGNED_INT, nullptr);
    }

    // ── Water ─────────────────────────────────────────────────────────────────
    bool hasWater = false;
    for (auto& m : _meshes) if (m.type == WATER) { hasWater = true; break; }

    if (hasWater) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glUseProgram(_waterProg);
        glUniformMatrix4fv(glGetUniformLocation(_waterProg, "uVP"), 1, GL_FALSE,
                           glm::value_ptr(VP));
        glUniform1f(glGetUniformLocation(_waterProg, "uDay"),  dayT);
        glUniform1f(glGetUniformLocation(_waterProg, "uTime"), timeS);
        glUniform3fv(glGetUniformLocation(_waterProg, "uCamPos"), 1,
                     glm::value_ptr(camPos));
        GLint waterBaseY = glGetUniformLocation(_waterProg, "uBaseY");

        for (auto& m : _meshes) {
            if (m.type != WATER || m.indexCount == 0) continue;
            glUniform1f(waterBaseY, m.baseY);
            glBindVertexArray(m.vao);
            glDrawElements(GL_TRIANGLES, (GLsizei)m.indexCount, GL_UNSIGNED_INT, nullptr);
        }
        glDisable(GL_BLEND);
    }

    glEnable(GL_CULL_FACE);
    glBindVertexArray(0);
    glUseProgram(0);
}

// ── cleanup ───────────────────────────────────────────────────────────────────

void OSMManager::cleanup() {
    ++_gen;
    if (_thread.joinable()) _thread.detach();

    clearMeshes();

    if (_wallProg)  glDeleteProgram(_wallProg);
    if (_flatProg)  glDeleteProgram(_flatProg);
    if (_waterProg) glDeleteProgram(_waterProg);
    for (auto& t : _facadeTex) if (t) glDeleteTextures(1, &t);
}

// ── fetchAsync (background thread) ───────────────────────────────────────────

void OSMManager::fetchAsync(double lat, double lon, int gen) {
    const double R = CELL_DEG;
    double s = lat - R, n = lat + R, w = lon - R, e = lon + R;

    // Cache filename: grid-aligned so nearby positions share cache
    double cLat = std::floor(lat / (R*2)) * (R*2);
    double cLon = std::floor(lon / (R*2)) * (R*2);
    char cacheFile[512];
    snprintf(cacheFile, sizeof(cacheFile), "%s/osm_%.3f_%.3f.json",
             _cacheDir.c_str(), cLat, cLon);

    std::string json;

    // Check cache
    struct stat st{};
    if (::stat(cacheFile, &st) == 0) {
        long long age = (long long)(time(nullptr) - st.st_mtime);
        if (age < 7LL * 24 * 3600) {
            FILE* f = fopen(cacheFile, "rb");
            if (f) {
                fseek(f, 0, SEEK_END);
                long sz = ftell(f); rewind(f);
                if (sz > 0) {
                    json.resize((size_t)sz);
                    fread(json.data(), 1, (size_t)sz, f);
                }
                fclose(f);
            }
        }
    }

    if (json.empty()) {
        char bbox[128];
        snprintf(bbox, sizeof(bbox), "%.6f,%.6f,%.6f,%.6f", s, w, n, e);

        std::string q;
        q += "[out:json][timeout:60];(\n";
        q += "  way[\"building\"]("; q += bbox; q += ");\n";
        q += "  way[\"highway\"~\"motorway|trunk|primary|secondary|tertiary|residential|service\"]("; q += bbox; q += ");\n";
        q += "  way[\"natural\"=\"water\"]("; q += bbox; q += ");\n";
        q += "  way[\"waterway\"~\"river|canal\"]("; q += bbox; q += ");\n";
        q += ");out body geom;";

        std::string postBody = "data=" + urlEncode(q);
        std::string resp;

        CURL* c = curl_easy_init();
        if (!c) return;
        curl_easy_setopt(c, CURLOPT_URL,           "https://overpass-api.de/api/interpreter");
        curl_easy_setopt(c, CURLOPT_POSTFIELDS,    postBody.c_str());
        curl_easy_setopt(c, CURLOPT_TIMEOUT,       75L);
        curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, curlWrite);
        curl_easy_setopt(c, CURLOPT_WRITEDATA,     &resp);
        curl_easy_setopt(c, CURLOPT_USERAGENT,     "WebFlightSim/1.0");
        curl_easy_setopt(c, CURLOPT_FOLLOWLOCATION,1L);

        CURLcode res = curl_easy_perform(c);
        long httpCode = 0;
        curl_easy_getinfo(c, CURLINFO_RESPONSE_CODE, &httpCode);
        curl_easy_cleanup(c);

        if (res == CURLE_OK && httpCode == 200) {
            json = std::move(resp);
            FILE* f = fopen(cacheFile, "wb");
            if (f) { fwrite(json.data(), 1, json.size(), f); fclose(f); }
            printf("[OSM] fetched %.1f KB for bbox %s\n",
                   json.size() / 1024.0, bbox);
        } else {
            fprintf(stderr, "[OSM] Overpass error curl=%d http=%ld\n", res, httpCode);
            return;
        }
    } else {
        printf("[OSM] cache hit (%zu bytes)\n", json.size());
    }

    if (_gen.load() != gen) return;

    RawBatch batch;
    try {
        buildGeometry(json, gen, batch);
    } catch (const std::exception& ex) {
        fprintf(stderr, "[OSM] buildGeometry failed: %s\n", ex.what());
        return;
    }

    if (_gen.load() != gen) return;

    std::lock_guard<std::mutex> lk(_mutex);
    _pending    = std::move(batch);
    _hasPending = true;
}

// ── buildGeometry ─────────────────────────────────────────────────────────────

void OSMManager::buildGeometry(const std::string& json, int gen, RawBatch& out) {
    using J = nlohmann::json;
    auto j = J::parse(json, nullptr, /*exceptions=*/false);
    if (j.is_discarded() || !j.contains("elements") || !j["elements"].is_array()) {
        fprintf(stderr, "[OSM] JSON parse/structure error\n");
        return;
    }

    // Road style table: halfWidth_m + color
    struct RS { float hw; glm::vec3 col; };
    static const std::pair<const char*, RS> ROAD_TABLE[] = {
        {"motorway",    {7.f,  {0.80f, 0.60f, 0.20f}}},
        {"trunk",       {6.f,  {0.85f, 0.65f, 0.25f}}},
        {"primary",     {5.f,  {0.75f, 0.55f, 0.20f}}},
        {"secondary",   {4.f,  {0.50f, 0.50f, 0.55f}}},
        {"tertiary",    {3.f,  {0.40f, 0.40f, 0.42f}}},
        {"residential", {2.5f, {0.35f, 0.35f, 0.35f}}},
        {"service",     {1.5f, {0.28f, 0.28f, 0.28f}}},
    };

    auto findRoad = [&](const std::string& hw) -> const RS* {
        for (auto& e : ROAD_TABLE)
            if (hw == e.first) return &e.second;
        return nullptr;
    };

    int bldIdx = 0;
    for (auto& el : j["elements"]) {
        if (_gen.load() != gen) return;
        if (!el.contains("geometry") || !el["geometry"].is_array()) continue;

        // Parse geometry
        std::vector<std::pair<double,double>> pts;
        pts.reserve(el["geometry"].size());
        bool geomOk = true;
        for (auto& g : el["geometry"]) {
            if (!g.contains("lat") || !g.contains("lon")) { geomOk=false; break; }
            pts.emplace_back(g["lat"].get<double>(), g["lon"].get<double>());
        }
        if (!geomOk || pts.size() < 2) continue;

        J empty = J::object();
        const J& tags = el.contains("tags") ? el["tags"] : empty;

        if (tags.contains("building")) {
            if (pts.size() >= 3) {
                float h = 0.f;
                if (tags.contains("height"))
                    try { h = std::stof(tags["height"].get<std::string>()); } catch(...) {}
                if (h <= 0.f && tags.contains("building:levels"))
                    try { h = (float)std::stoi(tags["building:levels"].get<std::string>()) * 3.5f; } catch(...) {}
                if (h <= 0.f) h = 3.5f;
                addBuilding(pts, h, bldIdx++, out);
            }
        } else if (tags.contains("highway")) {
            const RS* rs = findRoad(tags["highway"].get<std::string>());
            if (rs) addRoad(pts, rs->hw, rs->col, out);
        } else if (tags.contains("natural") && tags["natural"] == "water") {
            if (pts.size() >= 3) addWater(pts, out);
        } else if (tags.contains("waterway")) {
            std::string ww = tags["waterway"].get<std::string>();
            if ((ww == "river" || ww == "canal") && pts.size() >= 3)
                addWater(pts, out);
        }
    }

    printf("[OSM] geometry: %d buildings, %zu meshes total\n",
           bldIdx, out.meshes.size());
}

// ── addBuilding ───────────────────────────────────────────────────────────────

void OSMManager::addBuilding(const std::vector<std::pair<double,double>>& pts,
                              float height, int bldIdx, RawBatch& out) {
    // Centroid
    double sumLat=0, sumLon=0;
    for (auto& p : pts) { sumLat+=p.first; sumLon+=p.second; }
    glm::vec2 centXZ = toWorld(sumLat/pts.size(), sumLon/pts.size());

    // Polygon in local coords (relative to centroid)
    std::vector<glm::vec2> poly;
    poly.reserve(pts.size());
    for (auto& p : pts)
        poly.push_back(toWorld(p.first, p.second) - centXZ);

    // Remove closing duplicate
    if (poly.size() > 1 &&
        glm::length(poly.front() - poly.back()) < 0.05f)
        poly.pop_back();
    if (poly.size() < 3) return;

    // ── Walls ────────────────────────────────────────────────────────────────
    RawMesh walls;
    walls.type = BLDWALL;
    walls.facadeVariant = bldIdx % 5;
    walls.centroidXZ = centXZ;

    const float UV_U = 1.f / 7.f;    // one window column per 7 m
    const float UV_V = 1.f / 3.5f;   // one window row per 3.5 m

    for (size_t i = 0; i < poly.size(); i++) {
        glm::vec2 a = poly[i];
        glm::vec2 b = poly[(i+1) % poly.size()];
        float dx = b.x-a.x, dz = b.y-a.y;
        float len = std::sqrtf(dx*dx + dz*dz);
        if (len < 0.05f) continue;
        float nx = -dz/len, nz = dx/len;

        uint32_t base = (uint32_t)(walls.verts.size() / 8);
        float uLen = len * UV_U;
        float vHgt = height * UV_V;
        // pos(xyz) + normal(xyz) + uv(xy) per vertex
        float quad[] = {
            a.x, 0.f,    a.y,  nx,0.f,nz,  0.f,  0.f,
            b.x, 0.f,    b.y,  nx,0.f,nz,  uLen, 0.f,
            b.x, height, b.y,  nx,0.f,nz,  uLen, vHgt,
            a.x, height, a.y,  nx,0.f,nz,  0.f,  vHgt,
        };
        walls.verts.insert(walls.verts.end(), quad, quad+32);
        walls.idx.insert(walls.idx.end(),
            {base, base+1, base+2, base, base+2, base+3});
    }
    if (walls.idx.size() >= 6) out.meshes.push_back(std::move(walls));

    // ── Roof (earcut) ─────────────────────────────────────────────────────────
    using Coord = std::array<double,2>;
    std::vector<std::vector<Coord>> polygon(1);
    polygon[0].reserve(poly.size());
    for (auto& p : poly) polygon[0].push_back({(double)p.x, (double)p.y});

    auto roofIdx = mapbox::earcut<uint32_t>(polygon);
    if (roofIdx.size() < 3) return;

    RawMesh roof;
    roof.type = BLDROOF;
    roof.flatColor = {0.42f, 0.42f, 0.42f};
    roof.centroidXZ = centXZ;
    roof.verts.reserve(poly.size() * 3);
    for (auto& p : poly) {
        roof.verts.push_back(p.x);
        roof.verts.push_back(height);
        roof.verts.push_back(p.y);
    }
    roof.idx = std::move(roofIdx);
    out.meshes.push_back(std::move(roof));
}

// ── addRoad ───────────────────────────────────────────────────────────────────

void OSMManager::addRoad(const std::vector<std::pair<double,double>>& pts,
                          float halfW, glm::vec3 col, RawBatch& out) {
    // Use midpoint of the way as centroid for elevation
    size_t mid = pts.size() / 2;
    glm::vec2 centXZ = toWorld(pts[mid].first, pts[mid].second);

    RawMesh road;
    road.type = ROAD;
    road.flatColor = col;
    road.centroidXZ = centXZ;

    for (size_t i = 0; i+1 < pts.size(); i++) {
        glm::vec2 a = toWorld(pts[i].first,   pts[i].second)   - centXZ;
        glm::vec2 b = toWorld(pts[i+1].first, pts[i+1].second) - centXZ;
        float dx = b.x-a.x, dz = b.y-a.y;
        float len = std::sqrtf(dx*dx + dz*dz);
        if (len < 0.01f) continue;
        float nx = -dz/len * halfW;
        float nz =  dx/len * halfW;
        float Y  = 0.3f;  // slightly above terrain to avoid z-fighting
        uint32_t base = (uint32_t)(road.verts.size() / 3);
        float seg[] = {
            a.x-nx, Y, a.y-nz,
            a.x+nx, Y, a.y+nz,
            b.x-nx, Y, b.y-nz,
            b.x+nx, Y, b.y+nz,
        };
        road.verts.insert(road.verts.end(), seg, seg+12);
        road.idx.insert(road.idx.end(),
            {base, base+1, base+2, base+1, base+3, base+2});
    }
    if (road.idx.size() >= 6) out.meshes.push_back(std::move(road));
}

// ── addWater ──────────────────────────────────────────────────────────────────

void OSMManager::addWater(const std::vector<std::pair<double,double>>& pts,
                           RawBatch& out) {
    double sumLat=0, sumLon=0;
    for (auto& p : pts) { sumLat+=p.first; sumLon+=p.second; }
    glm::vec2 centXZ = toWorld(sumLat/pts.size(), sumLon/pts.size());

    std::vector<glm::vec2> poly;
    poly.reserve(pts.size());
    for (auto& p : pts)
        poly.push_back(toWorld(p.first, p.second) - centXZ);

    if (poly.size() > 1 &&
        glm::length(poly.front() - poly.back()) < 0.05f)
        poly.pop_back();
    if (poly.size() < 3) return;

    using Coord = std::array<double,2>;
    std::vector<std::vector<Coord>> polygon(1);
    polygon[0].reserve(poly.size());
    for (auto& p : poly) polygon[0].push_back({(double)p.x, (double)p.y});

    auto indices = mapbox::earcut<uint32_t>(polygon);
    if (indices.size() < 3) return;

    RawMesh water;
    water.type = WATER;
    water.centroidXZ = centXZ;
    water.verts.reserve(poly.size() * 3);
    for (auto& p : poly) {
        water.verts.push_back(p.x);
        water.verts.push_back(0.f);
        water.verts.push_back(p.y);
    }
    water.idx = std::move(indices);
    out.meshes.push_back(std::move(water));
}
