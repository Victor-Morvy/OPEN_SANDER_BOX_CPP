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
├── nav/             — airports.csv, runways.csv, navaids.csv (OurAirports)
└── models/          — erj195.obj + partes animadas (ailerons, flaps, spoilers,
                       gear, reversor, fans) + cockpit.obj; erj195.mtl aponta a
                       livery (Embraer195.png; blank195.png = fuselagem branca)
```

---

## Arquitetura

```
src/
├── main.cpp            game loop, input, câmeras (chase/órbita/cockpit), HUD ImGui
├── FDM.cpp/.h          wrapper JSBSim: init, step, telemetria, E195 bridge
├── FlyByWire.cpp/.h    FBW Normal Law: C* pitch, rate demand roll, yaw damper+beta
├── GuidanceModule.cpp  AFCS: altitude/heading/attitude hold, autothrottle, FD
├── Sky.cpp/.h          shader Preetham/Hosek-Wilkie
├── GeoProj.h           projeção ÚNICA do mundo (Web Mercator escalado)
├── TileManager.cpp/.h  tiles AWS Terrarium (elevação) + ESRI (textura), 4 LODs
├── Terrain.cpp/.h      mesh fallback checkerboard
├── Clouds.cpp/.h       nuvens billboard instanced
├── AcModel.cpp/.h      modelo 3D ERJ-195 (OBJ+MTL, partes animadas) e cockpit
├── AirportManager.cpp  CSV aeroportos, luzes de pista, PAPI, áreas planas,
│                       marcadores de aeroportos no HUD (até 250 km)
├── Navaids.cpp/.h      VOR/NDB/DME do navaids.csv (grade espacial 1°×1°)
├── MiniMap.cpp/.h      mapa raster OSM: HSD heading-up em runtime (tecla M) e
│                       mapa de teleporte no menu de pausa (pistas + espinha de
│                       peixe com marker beacons + navaids; clique na cabeceira
│                       posiciona para decolagem com a proa da pista)
├── OSMManager.cpp/.h   prédios, estradas e água via Overpass API em 7 batches
│                       (desativado: elevação dos 3D oscila vista de perto)
└── PostFX.cpp/.h       FBO (depth24+stencil8), bloom 2-pass
```

### Sistema de coordenadas

- **Projeção única** (`GeoProj.h`): Web Mercator escalado para metros na
  latitude de origem — tiles, pistas, avião e OSM usam as mesmas funções
  `geo::toWorld()`/`geo::toLatLon()`. Grid de tiles perfeitamente alinhado e
  pista desenhada casa com a foto de satélite em qualquer lugar do mundo.
- **Render**: aircraft sempre na origem (0,0,0). Todos os objetos subtraem `acWorld` antes do VP transform.
- **X** = Leste, **Y** = altitude AGL, **Z** = Sul (cauda)
- `wpos.x/z` derivados da lat/lon do JSBSim a cada frame (sem drift);
  `wpos.y` = AGL em metros; `acMslM = terrainElev_m + wpos.y`

### Terrain tiles — 4 LODs

```
ultraFarTiles — zoom 7,  9×9  grid (~1300 km raio)  pintor, sem depth
farTiles      — zoom 12, 17×17 grid (~78 km raio)   depth write, stencil 2
closeTiles    — zoom 15, 17×17 grid (~9.4 km raio)  depth write, stencil 3; ref. de altitude
nearTiles     — zoom 17, 9×9  grid (~1.3 km raio)   depth write, stencil 4; 1.2 m/px
```

- **Render fina → grossa com stencil por camada**: cada camada grava seu peso
  no stencil; a grossa nunca sobrescreve a fina, mas preenche onde a fina não
  carregou. Depth em near/close/far = oclusão real (montanha esconde o outro
  lado), sem z-fighting entre LODs e sem polygon offset.
- **Curvatura da Terra** no vertex shader (`y -= d²/2R`): horizonte físico real
  (~330 km a FL280) — esconde a borda do grid naturalmente
- **Fog atmosférico** de dia claro: ~40 km no solo, ~170 km a FL280 (cor
  idêntica ao earthHaze do céu)
- Far clip: 2000 km

Elevação: AWS Terrarium PNG → `R×256 + G + B/256 − 32768` metros MSL
(máx zoom 15 — a camada z17 amostra o sub-quadrante do tile pai z15)  
Textura: ESRI World Imagery

### FBW Normal Law (E195-E2)

| Eixo | Estado | Lei |
|------|--------|-----|
| Pitch | ativo | C\* = Nz + K·q, PID no erro + pitch envelope (+30°/−15°) |
| Roll | ativo | Rate demand (22°/s máx) / attitude hold, bank protection 33°/67° |
| Yaw | ativo | Yaw damper + auto-rudder PI no beta (elimina derrapagem residual) |

**Reversão de leis** (tecla `L`, anunciada no HUD — o E195-E2 tem apenas duas):
- **NORMAL** — tudo acima + estabilidade de velocidade: overspeed (>330 kt) → nariz sobe; underspeed (<150 kt com gear up) → nariz desce
- **DIRECT** — stick → superfície puro nos 3 eixos, sem aumentação

### AFCS (GuidanceModule) — painel F1

| Modo | Lei |
|------|-----|
| ALT SEL | Cascata alt → VS (PI c/ anti-windup por saturação) → pitch; V/S manual configurável |
| FLCH | Muda de nível pela velocidade: throttle fixo (climb 92% / idle 8%), pitch segura o CAS alvo; captura → ALT HOLD e devolve o throttle ao A/THR |
| HDG SEL | Erro de proa → bank demand (máx 25°) |
| LNAV | Flight plan por ICAO (airports.csv); bearing ao waypoint ativo → heading; sequencia a 1.5 NM; fim do plano → HDG HOLD |
| A/THR | PI bidirecional assimétrico — alvo é piso de velocidade (underspeed 2× mais forte) |
| ATT | Mantém pitch/bank capturados; override pelo painel sem desengajar |

Auto-desconexão de vert+lat com coluna/manche > 15%. Flight director sempre ativo com qualquer modo ligado.

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
| `R` | Reversor (toggle; só deploya no solo) |
| `L` | Lei FBW: NORMAL ↔ DIRECT |
| `Z` | Attitude Hold (toggle) |
| `H` | Altitude Hold (toggle) |
| `F1` | Painel AFCS Guidance |
| `1` / `2` | Cutoff motor 1 / 2 (toggle) |
| `Num 8/5` | Trim pitch |
| `Num 4/6` | Trim roll |
| `T` / `Y` | Hora do dia ± |
| `M` | Minimapa HSD (toggle; scroll = zoom) |
| `P` | Pausa (menu: teleporte por campos ou clicando no mapa; clique numa cabeceira = decolagem com a proa da pista) |
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
| Y / △ (3) | Reversor (toggle; só no solo) |
| LB / L1 (4) | Flaps subir |
| RB / R1 (5) | Flaps descer |
| Select (6) | Attitude Hold |
| Start (7) | Pausa |
| B / ○ (1) | Altitude Hold + câmera cockpit |
| L2+R2 + stick dir. | Órbita da câmera externa |
| R3 (click) | Reset câmera |
| D-pad ↑ / ↓ | Trim pitch picar / cabrar |
| D-pad ← / → | Trim roll esq / dir |

Mapeamento configurável via constantes `JS_AIL/ELV/THR/RDR` em `main.cpp`.

---

## Cache OSM

Respostas Overpass salvas em `%LOCALAPPDATA%/webflight/osm_cache/` com TTL de 7 dias.
