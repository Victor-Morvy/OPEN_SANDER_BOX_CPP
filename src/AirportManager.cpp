#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include "AirportManager.h"
#include "TileManager.h"
#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstdio>
#include <algorithm>

// ── CSV parser ────────────────────────────────────────────────────────────────

static std::vector<std::string> splitCSV(const std::string& line) {
    std::vector<std::string> out;
    std::string field;
    bool inQ = false;
    for (char c : line) {
        if      (c == '"')         inQ = !inQ;
        else if (c == ',' && !inQ) { out.push_back(field); field.clear(); }
        else                       field += c;
    }
    out.push_back(field);
    return out;
}

// Returns col→value map per row; header row used as key names.
static void readCSV(const std::string& path,
                    std::function<void(const std::unordered_map<std::string,std::string>&)> cb)
{
    std::ifstream f(path);
    if (!f) { fprintf(stderr,"[Airports] Arquivo não encontrado: %s\n", path.c_str()); return; }

    std::string line;
    std::vector<std::string> hdr;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        auto fields = splitCSV(line);
        if (hdr.empty()) { hdr = fields; continue; }
        std::unordered_map<std::string,std::string> row;
        for (size_t i = 0; i < hdr.size() && i < fields.size(); ++i)
            row[hdr[i]] = fields[i];
        cb(row);
    }
}

static float safeF(const std::unordered_map<std::string,std::string>& row,
                   const char* key, float def = 0.f)
{
    auto it = row.find(key);
    if (it == row.end() || it->second.empty()) return def;
    try { return std::stof(it->second); } catch(...) { return def; }
}

// ── Shaders ───────────────────────────────────────────────────────────────────

static const char* RW_VERT = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec2 aUV;
out vec2 vUV;
uniform mat4 uVP;
uniform vec3 uAcFull;
void main(){
    gl_Position = uVP * vec4(aPos - uAcFull, 1.0);
    vUV = aUV;
}
)glsl";

// Marcações ICAO procedurais em metros reais — vUV mapeado para metros,
// sem depender de razão de aspecto de textura.
static const char* RW_FRAG = R"glsl(
#version 330 core
in vec2 vUV;
out vec4 FragColor;
uniform float uDay;
uniform float uLengthM;
uniform float uWidthM;
uniform vec3  uSurfaceColor;

void main(){
    float alongM  = vUV.y * uLengthM;
    float acrossM = vUV.x * uWidthM;
    float halfW   = uWidthM * 0.5;
    float absFC   = abs(acrossM - halfW);

    bool mark = false;

    // Threshold bars: 6 barras 1.8 m × gap 1.8 m, zona de 30 m em cada cabeceira
    bool inThresh = (alongM < 30.0) || (alongM > uLengthM - 30.0);
    if (inThresh && absFC < 9.9) {
        if (mod(absFC, 3.6) < 1.8) mark = true;
    }

    // Centerline: 0.3 m largura, 30 m dash / 15 m gap, de 60 m ao centro
    if (!mark && absFC < 0.15 && alongM > 60.0 && alongM < uLengthM - 60.0)
        if (mod(alongM - 60.0, 45.0) < 30.0) mark = true;

    // TDZ: 3 m × largura da pista em 150 m e 300 m de cada cabeceira (L > 900 m)
    if (!mark && uLengthM > 900.0 && absFC > 3.0 && absFC < halfW - 2.0) {
        float q = uLengthM - alongM;
        if ((alongM > 149.0 && alongM < 152.0) || (alongM > 299.0 && alongM < 302.0) ||
            (q      > 149.0 && q      < 152.0) || (q      > 299.0 && q      < 302.0))
            mark = true;
    }

    vec3 col = mark ? vec3(0.85) : uSurfaceColor;
    col = pow(col, vec3(2.2));
    col *= mix(0.20, 1.0, uDay);
    FragColor = vec4(col, 1.0);
}
)glsl";

// Cor de superfície da pista (linear — shader aplica pow(x,2.2) depois)
static glm::vec3 surfaceColor(const std::string& surface) {
    std::string s = surface;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    bool isConcrete = (s == "con" || s == "conc" || s.find("concrete") != s.npos);
    bool isGrass    = (s == "grs" || s == "turf" || s == "gre" ||
                       s.find("grass") != s.npos || s.find("turf") != s.npos);
    bool isGravel   = (s == "gvl" || s.find("gravel") != s.npos ||
                       s.find("dirt") != s.npos || s.find("sand") != s.npos);
    if (isConcrete) return {0.596f, 0.596f, 0.596f};
    if (isGrass)    return {0.102f, 0.290f, 0.102f};
    if (isGravel)   return {0.361f, 0.239f, 0.157f};
    return {0.157f, 0.157f, 0.161f}; // asfalto
}

// Luzes de pista: GL_POINTS coloridos com halo suave
static const char* LT_VERT = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aCol;
out vec3 vCol;
uniform mat4 uVP;
uniform vec3 uAcFull;
void main(){
    vec4 clip = uVP * vec4(aPos - uAcFull, 1.0);
    // Tamanho aparente ~3m em world space, max 24px
    gl_PointSize = clamp(3000.0 / clip.w, 2.0, 24.0);
    gl_Position  = clip;
    vCol = aCol;
}
)glsl";

static const char* LT_FRAG = R"glsl(
#version 330 core
in vec3 vCol;
out vec4 FragColor;
void main(){
    vec2 c = gl_PointCoord - 0.5;
    float r = length(c) * 2.0;
    float a = smoothstep(1.0, 0.4, r);
    FragColor = vec4(vCol, a);
}
)glsl";

static GLuint makeShader(GLenum t, const char* s){
    GLuint sh=glCreateShader(t);
    glShaderSource(sh,1,&s,nullptr); glCompileShader(sh);
    GLint ok=0; glGetShaderiv(sh,GL_COMPILE_STATUS,&ok);
    if(!ok){char b[512];glGetShaderInfoLog(sh,512,nullptr,b);fprintf(stderr,"[Airports] Shader: %s\n",b);}
    return sh;
}
static GLuint linkProg(GLuint vs, GLuint fs){
    GLuint p=glCreateProgram();
    glAttachShader(p,vs); glAttachShader(p,fs); glLinkProgram(p);
    glDeleteShader(vs); glDeleteShader(fs);
    return p;
}

// ── Coordenadas ───────────────────────────────────────────────────────────────

glm::vec2 AirportManager::toWorld(float lat, float lon) const {
    return { (float)((lon - _originLon) * _mPerDegLon),
             (float)(-(lat - _originLat) * 111320.0) };
}

// ── Load ──────────────────────────────────────────────────────────────────────

bool AirportManager::load(const std::string& apCsv, const std::string& rwCsv,
                           double originLat, double originLon)
{
    _originLat   = originLat;
    _originLon   = originLon;
    _mPerDegLon  = 111320.0 * std::cos(originLat * 3.14159265358979 / 180.0);

    // Carregar pistas indexadas por airport_ident
    int rwCount = 0;
    readCSV(rwCsv, [&](const auto& row) {
        const std::string& id = row.at("airport_ident");
        float leLat  = safeF(row,"le_latitude_deg",  1e9f);
        float leLon  = safeF(row,"le_longitude_deg", 1e9f);
        float heLat  = safeF(row,"he_latitude_deg",  1e9f);
        float heLon  = safeF(row,"he_longitude_deg", 1e9f);
        if (std::abs(leLat)>90||std::abs(leLon)>180||std::abs(heLat)>90||std::abs(heLon)>180) return;
        Runway rw;
        rw.leLat   = leLat;  rw.leLon   = leLon;
        rw.heLat   = heLat;  rw.heLon   = heLon;
        rw.leElevM = safeF(row,"le_elevation_ft", 0.f) * 0.3048f;
        rw.heElevM = safeF(row,"he_elevation_ft", 0.f) * 0.3048f;
        rw.widthM  = std::max(15.f, safeF(row,"width_ft", 150.f) * 0.3048f);
        rw.lighted = (row.find("lighted") != row.end()) && (row.at("lighted") == "1");
        // Identificadores e superfície para gerar a textura da pista
        {
            auto it = row.find("le_ident"); rw.leIdent = (it != row.end()) ? it->second : "";
            it = row.find("he_ident");      rw.heIdent = (it != row.end()) ? it->second : "";
            it = row.find("surface");       rw.surface = (it != row.end()) ? it->second : "";
        }
        _runways[id].push_back(rw);
        ++rwCount;
    });

    // Carregar aeroportos com pistas válidas
    int apCount = 0;
    readCSV(apCsv, [&](const auto& row) {
        auto it = row.find("ident");
        if (it == row.end()) return;
        const std::string& ident = it->second;
        if (!_runways.count(ident)) return;
        float lat = safeF(row,"latitude_deg",  1e9f);
        float lon = safeF(row,"longitude_deg", 1e9f);
        if (std::abs(lat)>90||std::abs(lon)>180) return;
        Airport ap;
        ap.ident = ident;
        ap.lat   = lat;
        ap.lon   = lon;
        ap.elevM = safeF(row,"elevation_ft",0.f)*0.3048f;
        size_t idx = _airports.size();
        _airports.push_back(ap);
        int64_t key = (int64_t)(std::floor(lat)+90)*1000 + (int64_t)(std::floor(lon)+180);
        _grid[key].push_back(idx);
        ++apCount;
    });

    printf("[Airports] %d aeroportos | %d pistas carregados\n", apCount, rwCount);

    // Compilar shaders e criar VAO para luzes
    _rwProg = linkProg(makeShader(GL_VERTEX_SHADER,RW_VERT),
                       makeShader(GL_FRAGMENT_SHADER,RW_FRAG));
    _ltProg = linkProg(makeShader(GL_VERTEX_SHADER,LT_VERT),
                       makeShader(GL_FRAGMENT_SHADER,LT_FRAG));

    glGenVertexArrays(1,&_ltVAO);
    glGenBuffers(1,&_ltVBO);
    glBindVertexArray(_ltVAO);
    glBindBuffer(GL_ARRAY_BUFFER,_ltVBO);
    // aPos: xyz, aCol: rgb  →  6 floats per point
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,24,(void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,24,(void*)12);
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    return apCount > 0;
}

// ── buildGpuRwy ───────────────────────────────────────────────────────────────

void AirportManager::buildGpuRwy(GpuRwy& g) {
    float dx = g.wx2 - g.wx1, dz = g.wz2 - g.wz1;
    float len = std::sqrt(dx*dx + dz*dz);
    if (len < 1.f) return;
    float ux = dx/len, uz = dz/len;  // versor ao longo da pista
    float px = uz, pz = -ux;          // perpendicular (direita)
    float hw = g.widthM * 0.5f;
    g.lengthM = len;

    // 4 vértices: LE-left(0), LE-right(1), HE-right(2), HE-left(3)
    struct V { float x,y,z, u,v; };
    const float OFS = 0.5f; // leve offset acima do terreno para evitar z-fighting
    V verts[4] = {
        {g.wx1 - px*hw, g.elevLE + OFS, g.wz1 - pz*hw,  0,0},
        {g.wx1 + px*hw, g.elevLE + OFS, g.wz1 + pz*hw,  1,0},
        {g.wx2 + px*hw, g.elevHE + OFS, g.wz2 + pz*hw,  1,1},
        {g.wx2 - px*hw, g.elevHE + OFS, g.wz2 - pz*hw,  0,1},
    };
    // 2 triângulos
    V tris[6] = { verts[0],verts[1],verts[2], verts[0],verts[2],verts[3] };

    glGenVertexArrays(1,&g.vao);
    glGenBuffers(1,&g.vbo);
    glBindVertexArray(g.vao);
    glBindBuffer(GL_ARRAY_BUFFER,g.vbo);
    glBufferData(GL_ARRAY_BUFFER,sizeof(tris),tris,GL_STATIC_DRAW);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,20,(void*)0);   // aPos
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,2,GL_FLOAT,GL_FALSE,20,(void*)12);  // aUV
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    g.surfColor = surfaceColor(g.surface);
}

// ── clearActive ───────────────────────────────────────────────────────────────

void AirportManager::clearActive() {
    for (auto& g : _activeRwys) {
        if (g.vao) glDeleteVertexArrays(1,&g.vao);
        if (g.vbo) glDeleteBuffers(1,&g.vbo);
    }
    _activeRwys.clear();
}

// ── addAirportGpu ─────────────────────────────────────────────────────────────

void AirportManager::addAirportGpu(const Airport& ap,
                                    TileManager& close, TileManager& far_)
{
    const auto& rws = _runways.at(ap.ident);
    for (const auto& rw : rws) {
        glm::vec2 le = toWorld(rw.leLat, rw.leLon);
        glm::vec2 he = toWorld(rw.heLat, rw.heLon);
        float dx = he.x - le.x, dz = he.y - le.y;
        float len = std::sqrt(dx*dx + dz*dz);
        if (len < 10.f) continue;

        GpuRwy g;
        g.wx1 = le.x; g.wz1 = le.y;
        g.wx2 = he.x; g.wz2 = he.y;
        g.widthM  = rw.widthM;
        g.lighted = rw.lighted;
        g.leIdent = rw.leIdent;
        g.heIdent = rw.heIdent;
        g.surface = rw.surface;

        // Elevação: tenta terrain, cai no CSV se não disponível
        auto sampleElev = [&](float wx, float wz, float csvElev) -> float {
            float e = close.getElevAt(glm::vec3(wx, 0.f, wz));
            if (e != 0.f) return e;
            e = far_.getElevAt(glm::vec3(wx, 0.f, wz));
            if (e != 0.f) return e;
            return csvElev;
        };
        g.elevLE = sampleElev(g.wx1, g.wz1, rw.leElevM);
        g.elevHE = sampleElev(g.wx2, g.wz2, rw.heElevM);

        buildGpuRwy(g);
        if (!g.vao) continue;

        // Registra área plana no TileManager: achata terreno sob a pista
        float cx = (g.wx1 + g.wx2) * 0.5f;
        float cz = (g.wz1 + g.wz2) * 0.5f;
        float heading = std::atan2f(g.wx2 - g.wx1, g.wz2 - g.wz1);
        float halfW = g.widthM * 0.5f + 4.f;   // +4 m de acostamento
        float halfL = g.lengthM * 0.5f;
        close.registerFlatArea(cx, cz, halfW, halfL, heading, g.elevLE, g.elevHE, 200.f);
        far_ .registerFlatArea(cx, cz, halfW, halfL, heading, g.elevLE, g.elevHE, 200.f);

        _activeRwys.push_back(g);
    }
}

// ── rebuildLightVBO ───────────────────────────────────────────────────────────

void AirportManager::rebuildLightVBO(const glm::vec3& acWorld, float acMslM, float day)
{
    std::vector<LightPt> pts;
    pts.reserve(2000);

    for (const auto& g : _activeRwys) {
        if (!g.lighted) continue;

        float dx = g.wx2-g.wx1, dz = g.wz2-g.wz1;
        float len = g.lengthM > 0 ? g.lengthM : std::sqrt(dx*dx+dz*dz);
        if (len < 1.f) continue;
        float ux = dx/len, uz = dz/len;
        float px = uz, pz = -ux;         // perpendicular
        float hw = g.widthM*0.5f + 1.5f; // um pouco fora da borda

        int n = std::max(2, (int)(len / 60.f));
        for (int i = 0; i <= n; ++i) {
            float t  = (float)i / n;
            float ex = g.wx1 + ux*t*len;
            float ez = g.wz1 + uz*t*len;
            float ey = g.elevLE + (g.elevHE - g.elevLE)*t + 0.4f;

            float distFromHE = (1.f - t) * len;
            // Cor do lado esquerdo
            float rL, gL, bL;
            if (i == 0)            { rL=0.f; gL=0.9f; bL=0.2f; }  // threshold LE = verde
            else if (i == n)       { rL=0.9f;gL=0.1f; bL=0.1f; }  // threshold HE = vermelho
            else if (distFromHE < 600.f) { rL=0.9f;gL=0.5f;bL=0.0f; }  // âmbar
            else                   { rL=0.9f; gL=0.9f; bL=0.85f; } // branco

            // Lado esquerdo
            pts.push_back({ex - px*hw, ey, ez - pz*hw, rL,gL,bL});
            // Lado direito (mesma cor)
            pts.push_back({ex + px*hw, ey, ez + pz*hw, rL,gL,bL});
        }

        // PAPI: 4 luzes, 30 m à esquerda da cabeceira LE (approach vindo de HE→LE)
        // Ângulos de transição: 2.5°, 2.75°, 3.25°, 3.5°
        {
            float thAngles[4] = {2.5f, 2.75f, 3.25f, 3.5f};
            float papiX = g.wx1 - px*30.f;
            float papiZ = g.wz1 - pz*30.f;
            float papiY = g.elevLE + 0.5f;
            float dacX  = acWorld.x - papiX;
            float dacZ  = acWorld.z - papiZ;
            float hDist = std::sqrt(dacX*dacX + dacZ*dacZ);
            float angDeg = hDist > 10.f ? std::atan2(acMslM - papiY, hDist) * 180.f / 3.14159f : 0.f;
            for (int k = 0; k < 4; ++k) {
                float ox  = g.wx1 + px*(10.f + k*9.f) - px*30.f;
                float oz  = g.wz1 + pz*(10.f + k*9.f) - pz*30.f;
                bool white = angDeg >= thAngles[k];
                pts.push_back({ ox, papiY+0.5f, oz,
                    white ? 1.0f : 1.0f,
                    white ? 1.0f : 0.1f,
                    white ? 1.0f : 0.1f });
            }
        }
    }

    _ltCount = (int)pts.size();
    if (_ltCount == 0) return;

    int needed = _ltCount * 6 * sizeof(float);
    glBindBuffer(GL_ARRAY_BUFFER, _ltVBO);
    if (needed > _ltCap) {
        glBufferData(GL_ARRAY_BUFFER, needed, pts.data(), GL_DYNAMIC_DRAW);
        _ltCap = needed;
    } else {
        glBufferSubData(GL_ARRAY_BUFFER, 0, needed, pts.data());
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// ── update ────────────────────────────────────────────────────────────────────

void AirportManager::update(const glm::vec3& acWorld, float acMslM,
                             TileManager& close, TileManager& far_)
{
    // Converter posição do avião de volta para lat/lon (equirectangular reverso)
    double acLat = _originLat - acWorld.z / 111320.0;
    double acLon = _originLon + acWorld.x / _mPerDegLon;

    int cellX = (int)std::floor(acLat);
    int cellZ = (int)std::floor(acLon);
    if (cellX == _lastCellX && cellZ == _lastCellZ) return;
    _lastCellX = cellX; _lastCellZ = cellZ;

    clearActive();

    // Varrer células vizinhas ±1° lat, ±2° lon
    for (int dlat = -1; dlat <= 1; ++dlat) {
        for (int dlon = -2; dlon <= 2; ++dlon) {
            int64_t key = (int64_t)(cellX+dlat+90)*1000 + (int64_t)(cellZ+dlon+180);
            auto it = _grid.find(key);
            if (it == _grid.end()) continue;
            for (size_t idx : it->second) {
                const Airport& ap = _airports[idx];
                glm::vec2 w = toWorld(ap.lat, ap.lon);
                float dx = w.x - acWorld.x, dz = w.y - acWorld.z;
                if (dx*dx + dz*dz > SHOW_RADIUS_M*SHOW_RADIUS_M) continue;
                addAirportGpu(ap, close, far_);
            }
        }
    }
    printf("[Airports] %zu pistas ativas\n", _activeRwys.size());
}

// ── render ────────────────────────────────────────────────────────────────────

void AirportManager::render(const glm::mat4& VP,
                             const glm::vec3& acWorld, float acMslM, float day)
{
    if (_activeRwys.empty()) return;

    glm::vec3 acFull(acWorld.x, acMslM, acWorld.z);

    // ── Pistas ───────────────────────────────────────────────────────────────
    // Puxar pistas levemente para frente no depth buffer para sobrepor o terreno
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-2.f, -20.f);

    glUseProgram(_rwProg);
    glUniformMatrix4fv(glGetUniformLocation(_rwProg,"uVP"),    1,GL_FALSE,glm::value_ptr(VP));
    glUniform3fv      (glGetUniformLocation(_rwProg,"uAcFull"),1,glm::value_ptr(acFull));
    glUniform1f       (glGetUniformLocation(_rwProg,"uDay"),   day);
    glDisable(GL_CULL_FACE);
    for (const auto& g : _activeRwys) {
        if (!g.vao) continue;
        glUniform1f(glGetUniformLocation(_rwProg,"uLengthM"),     g.lengthM);
        glUniform1f(glGetUniformLocation(_rwProg,"uWidthM"),      g.widthM);
        glUniform3fv(glGetUniformLocation(_rwProg,"uSurfaceColor"),1,glm::value_ptr(g.surfColor));
        glBindVertexArray(g.vao);
        glDrawArrays(GL_TRIANGLES, 0, 6);
    }
    glEnable(GL_CULL_FACE);
    glBindVertexArray(0);

    glDisable(GL_POLYGON_OFFSET_FILL);

    // ── Luzes (apenas à noite / penumbra) ────────────────────────────────────
    float lightFade = 1.f - std::min(1.f, std::max(0.f, (day - 0.2f) / 0.35f));
    if (lightFade < 0.01f) return;

    rebuildLightVBO(acWorld, acMslM, day);
    if (_ltCount == 0) return;

    glUseProgram(_ltProg);
    glUniformMatrix4fv(glGetUniformLocation(_ltProg,"uVP"),    1,GL_FALSE,glm::value_ptr(VP));
    glUniform3fv      (glGetUniformLocation(_ltProg,"uAcFull"),1,glm::value_ptr(acFull));

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);   // aditivo para luzes
    glDepthMask(GL_FALSE);
    glEnable(GL_PROGRAM_POINT_SIZE);

    glBindVertexArray(_ltVAO);
    glDrawArrays(GL_POINTS, 0, _ltCount);
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glDisable(GL_PROGRAM_POINT_SIZE);
}

// ── cleanup ───────────────────────────────────────────────────────────────────

void AirportManager::cleanup() {
    clearActive();
    if (_ltVAO) glDeleteVertexArrays(1,&_ltVAO);
    if (_ltVBO) glDeleteBuffers(1,&_ltVBO);
    if (_rwProg)glDeleteProgram(_rwProg);
    if (_ltProg)glDeleteProgram(_ltProg);
}
