#include "Navaids.h"
#include <fstream>
#include <cmath>
#include <cstdio>
#include <cstdlib>

// ── CSV (mesmo formato/estilo do AirportManager) ──────────────────────────────

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

bool Navaids::load(const std::string& csvPath) {
    std::ifstream f(csvPath);
    if (!f) { fprintf(stderr, "[Navaids] Arquivo não encontrado: %s\n", csvPath.c_str()); return false; }

    // colunas fixas do OurAirports: 2=ident 3=name 4=type 5=frequency_khz
    //                               6=latitude_deg 7=longitude_deg
    std::string line;
    bool header = true;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        if (header) { header = false; continue; }
        auto c = splitCSV(line);
        if (c.size() < 8) continue;

        Type t;
        const std::string& ty = c[4];
        if      (ty == "VOR" || ty == "VOR-DME" || ty == "VORTAC") t = Type::VOR;
        else if (ty == "NDB" || ty == "NDB-DME")                   t = Type::NDB;
        else if (ty == "DME" || ty == "TACAN")                     t = Type::DME;
        else continue;

        char* end = nullptr;
        double lat = std::strtod(c[6].c_str(), &end);
        if (end == c[6].c_str()) continue;
        double lon = std::strtod(c[7].c_str(), &end);
        if (end == c[7].c_str()) continue;
        if (std::abs(lat) > 90.0 || std::abs(lon) > 180.0) continue;

        Nav n;
        n.ident   = c[2];
        n.name    = c[3];
        n.type    = t;
        n.lat     = lat;
        n.lon     = lon;
        n.freqKhz = std::atoi(c[5].c_str());

        size_t idx = _navs.size();
        _navs.push_back(std::move(n));
        int64_t key = (int64_t)(std::floor(lat) + 90) * 1000
                    + (int64_t)(std::floor(lon) + 180);
        _grid[key].push_back(idx);
    }
    printf("[Navaids] %zu navaids carregados\n", _navs.size());
    return !_navs.empty();
}

void Navaids::getNearby(double lat, double lon, double radiusM,
                        std::vector<const Nav*>& out) const
{
    out.clear();
    double mPerDegLat = 111320.0;
    double mPerDegLon = 111320.0 * std::cos(lat * 3.14159265358979 / 180.0);
    if (mPerDegLon < 1.0) mPerDegLon = 1.0;

    int cellX = (int)std::floor(lat);
    int cellZ = (int)std::floor(lon);
    int dLat  = (int)std::ceil(radiusM / mPerDegLat) + 1;
    int dLon  = (int)std::ceil(radiusM / mPerDegLon) + 1;

    for (int la = -dLat; la <= dLat; ++la) {
        for (int lo = -dLon; lo <= dLon; ++lo) {
            int64_t key = (int64_t)(cellX + la + 90) * 1000
                        + (int64_t)(cellZ + lo + 180);
            auto it = _grid.find(key);
            if (it == _grid.end()) continue;
            for (size_t idx : it->second) {
                const Nav& n = _navs[idx];
                double dx = (n.lon - lon) * mPerDegLon;
                double dy = (n.lat - lat) * mPerDegLat;
                if (dx * dx + dy * dy > radiusM * radiusM) continue;
                out.push_back(&n);
            }
        }
    }
}
