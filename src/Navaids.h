#pragma once
#include <string>
#include <vector>
#include <unordered_map>

// navaids.csv (OurAirports) → grade espacial 1°×1° para consulta por raio.
// Usado pelo minimapa (HSD e picker do menu de pausa).
class Navaids {
public:
    enum class Type { VOR, NDB, DME };

    struct Nav {
        std::string ident, name;
        Type   type;
        double lat, lon;
        int    freqKhz;   // VOR/DME: 114500 = 114.50 MHz | NDB: kHz direto
    };

    bool load(const std::string& csvPath);

    // Navaids num raio (aponta para o storage interno — estável após load)
    void getNearby(double lat, double lon, double radiusM,
                   std::vector<const Nav*>& out) const;

private:
    std::vector<Nav> _navs;
    std::unordered_map<int64_t, std::vector<size_t>> _grid;
};
