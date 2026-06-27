# WebFlight C++ Simulator

Simulador de voo nativo em C++ com física real (JSBSim), gráficos OpenGL 3.3 e aeronave padrão **Embraer E195-E2** com FBW Normal Law. Porte do simulador WebFlight (Node.js / Three.js / JSBSim-WASM) para aplicação nativa.

---

## Stack

| Camada | Biblioteca | vcpkg |
|--------|-----------|-------|
| Janela + input | GLFW 3 | `glfw3` |
| OpenGL loader | GLAD | `glad` |
| Math | GLM | `glm` |
| UI (HUD + menus) | Dear ImGui | `imgui` |
| HTTP (tiles, OSM) | libcurl | `curl` |
| Decode de imagens | stb_image | `stb` |
| JSON (OSM) | nlohmann/json | `nlohmann-json` |
| Física (FDM) | JSBSim | FetchContent (GitHub) |

> JSBSim é baixado e compilado automaticamente pelo CMake — não precisa de vcpkg para ele.

---

## Pré-requisitos

- **CMake** ≥ 3.20
- **Visual Studio 2022** (MSVC) com C++17
- **vcpkg** instalado

### Dependências via vcpkg

```bash
vcpkg install glfw3:x64-windows glad:x64-windows glm:x64-windows imgui:x64-windows curl:x64-windows stb:x64-windows nlohmann-json:x64-windows
```

---

## Build

```bash
git clone git@github.com:Victor-Morvy/OPEN_SANDER_BOX_CPP.git
cd OPEN_SANDER_BOX_CPP

cmake -B build -S . \
  -DCMAKE_TOOLCHAIN_FILE="C:/Users/Victor/Documents/Repositories/vcpkg/scripts/buildsystems/vcpkg.cmake" \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build --config Release --parallel
```

Executável: `build/Release/webflight.exe`

---

## Dados (self-contained)

Todos os dados estão na pasta `data/` do repositório — não depende do projeto Node.js:

```
data/
├── aircraft/        — XMLs JSBSim (E195, C172P, DC3, MD11, ...)
├── engine/          — GE CF34-10E, PT6A, Hamilton Standard, ...
├── systems/         — GNCUtilities.xml
├── nav/             — airports.csv, runways.csv
└── models/          — c172p.glb (placeholder 3D)
```

---

## Arquitetura

```
src/
├── main.cpp            game loop, input, câmera chase, HUD ImGui
├── FDM.cpp/.h          wrapper JSBSim: init, step, telemetria, E195 bridge
├── FlyByWire.cpp/.h    lei FBW C* pitch (roll/yaw direto por ora)
├── Sky.cpp/.h          shader Preetham/Hosek-Wilkie
├── TileManager.cpp/.h  tiles AWS Terrarium (elevação) + ESRI (textura), 2 LOD
├── Terrain.cpp/.h      mesh fallback checkerboard
├── Clouds.cpp/.h       nuvens billboard instanced
├── AirportManager.cpp  CSV aeroportos, luzes de pista, PAPI
├── OSMManager.cpp/.h   prédios + estradas via Overpass API (pausado)
└── PostFX.cpp/.h       FBO, bloom 2-pass, fog
```

### Sistema de coordenadas

- **Render**: aircraft sempre na origem (0,0,0). Todos os objetos subtraem `acWorld` antes do VP transform.
- **X** = Leste, **Y** = altitude AGL, **Z** = Sul (cauda)
- `wpos.y` = AGL em metros; `acMslM = terrainElev_m + wpos.y`
- Tiles LOD: zoom 13 (far, −5 m bias) + zoom 15 (close)

### Terrain tiles

```
TileManager farTiles  — zoom 13, 9×9 grid, yBias −5 m (sempre abaixo do close)
TileManager closeTiles — zoom 15, 9×9 grid, referência de altitude
```

Elevação: AWS Terrarium PNG → `R×256 + G + B/256 − 32768` metros MSL  
Textura: ESRI World Imagery

### FBW Normal Law (E195-E2)

| Eixo | Estado | Lei |
|------|--------|-----|
| Pitch | ativo | C\* = Nz + K·q, PID no erro |
| Roll | direto | `aileron = inp.wheel` (FBW desligado) |
| Yaw | direto | `rudder = inp.pedals` (damper desligado) |

---

## Controles

### Teclado

| Tecla | Ação |
|-------|------|
| `↑` / `↓` | Profundor |
| `←` / `→` | Aileron |
| `A` / `D` | Leme |
| `W` / `S` | Throttle |
| `B` | Freio |
| `F` / `V` | Flaps subir / descer |
| `G` | Trem de pouso (toggle) |
| `Num 8/5` | Trim pitch |
| `Num 4/6` | Trim roll |
| `T` / `Y` | Hora do dia ± |
| `P` | Pausa |
| `Esc` | Sair |

### Joystick (gamepad Xbox / PS)

| Eixo / Botão | Ação |
|---|---|
| Eixo 0 | Aileron |
| Eixo 1 | Elevator |
| Eixo 2 | Rudder |
| Eixo 3 | Throttle (incremental) |
| A / Cruz (0) | Freio |
| X / □ (2) | Trem (toggle) |
| LB / L1 (4) | Flaps subir |
| RB / R1 (5) | Flaps descer |
| Y / △ (3) | Reversor (pré-mapeado) |
| Start (7) | Pausa |
| D-pad ↑ / ↓ | Trim pitch picar / cabrar |
| D-pad ← / → | Trim roll esq / dir |

Mapeamento configurável via constantes `JS_AIL/ELV/THR/RDR` em `main.cpp`.

---

## Cache OSM

Respostas Overpass salvas em `%LOCALAPPDATA%/webflight/osm_cache/` com TTL de 7 dias.
