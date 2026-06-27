#include "AcModel.h"
#include <glm/gtc/type_ptr.hpp>
#include <stb_image.h>   // impl defined in TileManager.cpp
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

// ── Texture map (matIdx → filename) ──────────────────────────────────────────
const char* AcModel::TEX_MAP[4] = {
    "Embraer195.png",  // mat0 — fuselage/livery
    "ERJ-wing.png",    // mat1 — wing
    "Embraer195.png",  // mat2 — glass
    "Embraer195.png",  // mat3 — dark
};

// ── Texture loader ────────────────────────────────────────────────────────────

GLuint AcModel::loadTex(const std::string& file) {
    auto it = _texCache.find(file);
    if (it != _texCache.end()) return it->second;

    std::string path = _dir + file;
    int w, h, c;
    stbi_set_flip_vertically_on_load(false);
    unsigned char* data = stbi_load(path.c_str(), &w, &h, &c, 4);
    if (!data) {
        std::cerr << "[AcModel] texture not found: " << path << "\n";
        _texCache[file] = 0;
        return 0;
    }
    GLuint id;
    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    stbi_image_free(data);
    _texCache[file] = id;
    _textures.push_back(id);
    return id;
}

// ── OBJ mesh loader ───────────────────────────────────────────────────────────

struct ObjVert { float x,y,z, nx,ny,nz, u,v; };

bool AcModel::loadMesh(const std::string& objPath, std::vector<AcMesh>& out) {
    std::ifstream f(objPath);
    if (!f) { std::cerr << "[AcModel] cannot open " << objPath << "\n"; return false; }

    std::vector<glm::vec3> pos, nrm;
    std::vector<glm::vec2> uv;

    int curMat = 0;
    std::string curMatName;
    struct Group {
        std::vector<ObjVert> verts;
        std::vector<uint32_t> idx;
        int mat;
        std::string texFile;  // from map_Kd in MTL (empty = use TEX_MAP)
    };
    std::vector<Group> groups;
    std::unordered_map<std::string, int>         matIndex;
    std::unordered_map<std::string, std::string> matTex;   // mat name → texture file

    // MTL parser: reads newmtl names (→ index) and map_Kd (→ texture file)
    auto parseMtl = [&](const std::string& mtlPath) {
        std::ifstream mf(mtlPath);
        if (!mf) return;
        std::string line, curNm; int idx = -1;
        while (std::getline(mf, line)) {
            std::istringstream ss(line); std::string key; ss >> key;
            if (key == "newmtl") {
                ss >> curNm; ++idx; matIndex[curNm] = idx;
            } else if (key == "map_Kd" && !curNm.empty()) {
                std::string tex; ss >> tex;
                matTex[curNm] = tex;
            }
        }
    };

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0]=='#') continue;
        std::istringstream ss(line); std::string key; ss >> key;

        if (key == "mtllib") {
            std::string mf; ss >> mf;
            parseMtl(_dir + mf);
        } else if (key == "v") {
            float x,y,z; ss>>x>>y>>z; pos.push_back({x,y,z});
        } else if (key == "vt") {
            float u,v; ss>>u>>v; uv.push_back({u,v});
        } else if (key == "vn") {
            float x,y,z; ss>>x>>y>>z; nrm.push_back({x,y,z});
        } else if (key == "usemtl") {
            ss >> curMatName;
            auto it = matIndex.find(curMatName);
            curMat = (it != matIndex.end()) ? it->second : 0;
            // Find or create group for this (mat, texFile) combination
            std::string texFile;
            auto ti = matTex.find(curMatName);
            if (ti != matTex.end()) texFile = ti->second;
            bool found = false;
            for (auto& g : groups)
                if (g.mat==curMat && g.texFile==texFile){found=true;break;}
            if (!found) groups.push_back({{},{},curMat,texFile});
        } else if (key == "f") {
            // Find group for current (mat, texFile)
            std::string texFile;
            auto ti = matTex.find(curMatName);
            if (ti != matTex.end()) texFile = ti->second;
            Group* grp = nullptr;
            for (auto& g : groups)
                if (g.mat==curMat && g.texFile==texFile){grp=&g;break;}
            if (!grp){ groups.push_back({{},{},curMat,texFile}); grp=&groups.back(); }

            std::vector<ObjVert> fv;
            std::string tok;
            while (ss >> tok) {
                for (auto& c : tok) if (c=='/') c=' ';
                std::istringstream ts(tok);
                int vi=0,vti=0,vni=0; ts>>vi;
                if (!ts.eof()) ts>>vti;
                if (!ts.eof()) ts>>vni;
                ObjVert ov{};
                if (vi>0 && vi<=(int)pos.size()) { ov.x=pos[vi-1].x; ov.y=pos[vi-1].y; ov.z=pos[vi-1].z; }
                if (vni>0 && vni<=(int)nrm.size()){ ov.nx=nrm[vni-1].x; ov.ny=nrm[vni-1].y; ov.nz=nrm[vni-1].z; }
                if (vti>0 && vti<=(int)uv.size()) { ov.u=uv[vti-1].x; ov.v=uv[vti-1].y; }
                fv.push_back(ov);
            }
            for (int t=1; t<(int)fv.size()-1; t++)
                for (int k : {0,t,t+1}) {
                    grp->idx.push_back((uint32_t)grp->verts.size());
                    grp->verts.push_back(fv[k]);
                }
        }
    }

    for (auto& g : groups) {
        if (g.verts.empty()) continue;
        AcMesh m;
        m.matIdx = g.mat;
        m.count  = (int)g.idx.size();
        // Textura: usa map_Kd do MTL se disponível, senão recorre ao TEX_MAP fixo
        if (!g.texFile.empty()) {
            m.texId = loadTex(g.texFile);
        } else {
            int mi = g.mat;
            m.texId = loadTex((mi>=0 && mi<4) ? TEX_MAP[mi] : "Embraer195.png");
        }

        glGenVertexArrays(1,&m.vao); glGenBuffers(1,&m.vbo); glGenBuffers(1,&m.ibo);
        glBindVertexArray(m.vao);
        glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
        glBufferData(GL_ARRAY_BUFFER, g.verts.size()*sizeof(ObjVert), g.verts.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, g.idx.size()*sizeof(uint32_t), g.idx.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(ObjVert),(void*)offsetof(ObjVert,x));  glEnableVertexAttribArray(0);
        glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(ObjVert),(void*)offsetof(ObjVert,nx)); glEnableVertexAttribArray(1);
        glVertexAttribPointer(2,2,GL_FLOAT,GL_FALSE,sizeof(ObjVert),(void*)offsetof(ObjVert,u));  glEnableVertexAttribArray(2);
        glBindVertexArray(0);
        out.push_back(m);
    }
    return true;
}

// ── Main load ─────────────────────────────────────────────────────────────────

bool AcModel::load(const std::string& objPath) {
    auto sl = objPath.find_last_of("/\\");
    _dir = (sl != std::string::npos) ? objPath.substr(0, sl+1) : "";

    // Static mesh
    if (!loadMesh(objPath, _static)) return false;
    std::cout << "[AcModel] static: " << _static.size() << " groups\n";

    // Animated parts from JSON
    std::string base = objPath;
    auto dot = base.rfind('.');
    if (dot != std::string::npos) base = base.substr(0, dot);
    std::string jsonPath = base + "_parts.json";

    std::ifstream jf(jsonPath);
    if (!jf) {
        std::cerr << "[AcModel] parts JSON not found: " << jsonPath << " — no animation\n";
        _loaded = true;
        return true;
    }

    nlohmann::json jdoc;
    try { jf >> jdoc; } catch (...) {
        std::cerr << "[AcModel] failed to parse " << jsonPath << "\n";
        _loaded = true;
        return true;
    }

    for (auto& jp : jdoc["parts"]) {
        AcPart part;
        part.name = jp["name"].get<std::string>();
        std::string pobj = _dir + jp["obj"].get<std::string>();

        auto& pv = jp["pivot"];
        part.pivot = {pv[0].get<float>(), pv[1].get<float>(), pv[2].get<float>()};
        auto& ax = jp["axis"];
        part.axis  = {ax[0].get<float>(), ax[1].get<float>(), ax[2].get<float>()};

        if (jp.contains("max_deg") && !jp["max_deg"].is_null())
            part.maxRad = jp["max_deg"].get<float>() * 3.14159265f / 180.f;
        if (jp.contains("sign"))
            part.sign = jp["sign"].get<float>();

        if (!loadMesh(pobj, part.meshes)) {
            std::cerr << "[AcModel] skipping part " << part.name << "\n";
            continue;
        }
        _parts.push_back(std::move(part));
    }
    std::cout << "[AcModel] animated parts: " << _parts.size() << "\n";

    _loaded = true;
    return true;
}

// ── Animation angle mapping ───────────────────────────────────────────────────

float AcModel::partAngle(const AcPart& p, const AcAnimState& s) const {
    const std::string& name = p.name;
    const float mr  = p.maxRad;   // max rotation from JSON
    const float sgn = p.sign;

    // Control surfaces — [-1..1] × maxRad × sign
    if (name == "aileron.l")  return  s.aileronL * mr * sgn;
    if (name == "aileron.r")  return  s.aileronR * mr * sgn;
    if (name == "elevator.l" || name == "elevator.r") return s.elevL * mr * sgn;
    if (name == "rudder")     return  s.rudder   * mr * sgn;

    // Flaps [0..1] × maxRad × sign  (XML max 25° at full extension)
    if (name.size()>=4 && name.substr(0,4)=="flap")
        return s.flaps * mr * sgn;

    // Spoilers [0..1]
    if (name.size()>=10 && name.substr(0,10)=="spoiler.l") return s.spoilerL * mr * sgn;
    if (name.size()>=10 && name.substr(0,10)=="spoiler.r") return s.spoilerR * mr * sgn;

    // Landing gear legs/tires — retract angle = (1−gearPos) × maxRad
    if (name=="gear.l"||name=="gear.r"||name=="gear.n"||
        (name.size()>=4 && name.substr(0,4)=="tire"))
        return (1.f - s.gearPos) * mr * sgn;

    // Nose gear front doors: open during transit (sinusoidal), closed when stationary
    if (name=="geardoor.f.l"||name=="geardoor.f.r") {
        // FG: opens to ±90° at mid-transit (gearPos 0.2→0.8), closed at 0 and 1
        float t = s.gearPos;
        float open = std::sin(t * 3.14159265f);  // 0 at 0 and 1, 1 at 0.5
        return open * mr * sgn;
    }

    // Nose gear back doors: stay open when gear is down
    if (name=="geardoor.b.l"||name=="geardoor.b.r") {
        // FG: closed at pos=0 (retracted), open at pos>0.2 (stays open when down)
        float t = std::min(1.f, s.gearPos / 0.2f);
        return t * mr * sgn;
    }

    // Engine fans (continuous spin)
    if (name=="fan.l"||name=="fan.r") return s.fanAngle;

    return 0.f;
}

// ── Draw helpers ──────────────────────────────────────────────────────────────

void AcModel::drawMeshes(GLuint prog, const std::vector<AcMesh>& meshes) const {
    for (const auto& m : meshes) {
        glUniform1i(glGetUniformLocation(prog, "uHasTex"), m.texId ? 1 : 0);
        if (m.texId) {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, m.texId);
            glUniform1i(glGetUniformLocation(prog, "uTex"), 0);
        }
        glBindVertexArray(m.vao);
        glDrawElements(GL_TRIANGLES, m.count, GL_UNSIGNED_INT, nullptr);
    }
}

void AcModel::draw(GLuint staticProg, GLuint animProg,
                   const glm::mat4& MVP, const AcAnimState& anim) const {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // Static mesh
    glUseProgram(staticProg);
    glUniformMatrix4fv(glGetUniformLocation(staticProg,"uMVP"), 1, GL_FALSE, glm::value_ptr(MVP));
    glUniform1f(glGetUniformLocation(staticProg,"uAlpha"), 1.f);
    drawMeshes(staticProg, _static);

    // Animated parts
    glUseProgram(animProg);
    glUniformMatrix4fv(glGetUniformLocation(animProg,"uMVP"), 1, GL_FALSE, glm::value_ptr(MVP));
    glUniform1f(glGetUniformLocation(animProg,"uAlpha"), 1.f);
    for (const auto& part : _parts) {
        float angle = partAngle(part, anim);
        glUniform3fv(glGetUniformLocation(animProg,"uPivot"), 1, glm::value_ptr(part.pivot));
        glUniform3fv(glGetUniformLocation(animProg,"uAxis"),  1, glm::value_ptr(part.axis));
        glUniform1f (glGetUniformLocation(animProg,"uAngle"), angle);
        drawMeshes(animProg, part.meshes);
    }

    glBindVertexArray(0);
    glDisable(GL_CULL_FACE);
}

// ── Cleanup ───────────────────────────────────────────────────────────────────

void AcModel::cleanup() {
    auto freeMeshes = [](std::vector<AcMesh>& v){
        for (auto& m : v) {
            glDeleteVertexArrays(1,&m.vao);
            glDeleteBuffers(1,&m.vbo);
            glDeleteBuffers(1,&m.ibo);
        }
        v.clear();
    };
    freeMeshes(_static);
    for (auto& p : _parts) freeMeshes(p.meshes);
    _parts.clear();
    for (auto id : _textures) glDeleteTextures(1,&id);
    _textures.clear();
    _texCache.clear();
    _loaded = false;
}
