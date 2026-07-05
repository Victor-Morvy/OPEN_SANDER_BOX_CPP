#pragma once
#include <cmath>

// ── Projeção ÚNICA do mundo render ────────────────────────────────────────────
// Web Mercator esférico escalado para metros na latitude de origem.
//
// POR QUÊ: os tiles (ESRI/Terrarium) são Web Mercator. A projeção antiga era
// equiretangular com cos(lat) POR TILE — cada fileira de tiles usava um fator
// diferente e, longe da origem em longitude (ex.: São Paulo partindo do Rio),
// as fileiras deslocavam ~20 m entre si (saias verticais visíveis nas bordas)
// e as pistas (que usavam cos da origem) não batiam com a foto de satélite.
//
// Com Mercator escalado:
//   - o grid de tiles é perfeitamente retangular e alinhado (zero deslocamento)
//   - X é linear em longitude e Z linear em mercY — exatamente como os pixels
//     das texturas — pista desenhada casa com a imagem em qualquer lugar
//   - 1 unidade de mundo = 1 metro real na latitude de origem; longe dela a
//     escala varia por cos(lat0)/cos(lat) (0,4% Rio→SP — invisível)
//
// TODOS os sistemas (tiles, pistas, avião, OSM) DEVEM usar estas funções.
// Convenção render: X = Leste, Z = Sul (Norte = -Z).
namespace geo {

inline constexpr double PI_      = 3.14159265358979323846;
inline constexpr double D2R      = PI_ / 180.0;
inline constexpr double R_MERC   = 6378137.0;   // raio esférico do Web Mercator

inline double mercY(double latDeg) {
    return R_MERC * std::log(std::tan(PI_ * 0.25 + latDeg * D2R * 0.5));
}

// lat/lon → world XZ [m]
inline void toWorld(double latDeg, double lonDeg,
                    double lat0, double lon0,
                    double& x, double& z)
{
    double k = std::cos(lat0 * D2R);
    x =  (lonDeg - lon0) * D2R * R_MERC * k;
    z = -(mercY(latDeg) - mercY(lat0)) * k;
}

// world XZ [m] → lat/lon
inline void toLatLon(double x, double z,
                     double lat0, double lon0,
                     double& latDeg, double& lonDeg)
{
    double k = std::cos(lat0 * D2R);
    lonDeg = lon0 + x / (R_MERC * k * D2R);
    double my = mercY(lat0) - z / k;
    latDeg = (2.0 * std::atan(std::exp(my / R_MERC)) - PI_ * 0.5) / D2R;
}

} // namespace geo
