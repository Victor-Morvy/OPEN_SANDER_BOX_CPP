# WebFlight C++ Simulator

Simulador de voo nativo em C++ com física real (JSBSim), gráficos OpenGL e aeronave padrão **Embraer E195-E2** com FBW Normal Law.

---

## Stack

| Camada | Biblioteca | vcpkg name |
|--------|-----------|------------|
| Janela + input | GLFW 3 | `glfw3` |
| OpenGL loader | GLAD | `glad` |
| Math (vetores/matrizes) | GLM | `glm` |
| UI (HUD + menus) | Dear ImGui | `imgui` |
| HTTP (tiles de terreno) | libcurl | `curl` |
| Decode de imagens | stb (stb_image) | `stb` |
| Física (FDM) | JSBSim | via FetchContent (GitHub) |

> JSBSim é baixado e compilado automaticamente pelo CMake via `FetchContent` — **não precisa de vcpkg** para ele.

---

## Pré-requisitos

- **CMake** ≥ 3.20
- **Visual Studio 2022** (MSVC) ou equivalente com suporte a C++17
- **vcpkg** instalado e integrado ao CMake (`-DCMAKE_TOOLCHAIN_FILE=...`)

### Instalar dependências via vcpkg

```bash
vcpkg install glfw3 glad glm imgui curl stb
```

Se usar o triplet x64-windows (recomendado):

```bash
vcpkg install glfw3:x64-windows glad:x64-windows glm:x64-windows imgui:x64-windows curl:x64-windows stb:x64-windows
```

---

## Build

```bash
# 1. Clone o repositório
git clone <url> webflight-cpp
cd webflight-cpp

# 2. Configure — aponte para o toolchain do vcpkg
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake" -DCMAKE_BUILD_TYPE=Release

# 3. Compile (JSBSim será baixado automaticamente)
cmake --build build --config Release --parallel
```

O executável fica em `build/Release/webflight.exe`.

---

## Dados da aeronave (XMLs JSBSim)

Os arquivos de aeronave **não estão neste repositório**. O CMakeLists.txt espera encontrá-los em `../newGame/public/` (lado a lado com o projeto Node.js):

```
../newGame/public/
├── aircraft/E195/        ← modelo E195-E2 (FDM XML, flight-control.xml)
├── engine/               ← GE CF34-10E-L.xml, GE CF34-10E-R.xml
└── data/                 ← airports.csv, runways.csv
```

Se quiser rodar de forma independente, copie esses diretórios para `data/` e ajuste os paths no `CMakeLists.txt`:

```cmake
set(AC_ROOT "${CMAKE_SOURCE_DIR}/data")
```

---

## Arquitetura

```
main.cpp          — game loop (GLFW, ImGui, input, steps)
FDM.cpp/.h        — wrapper JSBSim: init, step, telemetria, E195 FBW bridge
FlyByWire.cpp/.h  — leis FBW: C* pitch, rate demand roll, yaw damper
Sky.cpp/.h        — shader de céu Preetham/Hosek-Wilkie + bloom
TileManager.cpp   — grid 9×9 de tiles de terreno (AWS Terrarium elevation + ESRI texture)
Terrain.cpp/.h    — mesh de terreno (VBO dinâmico, LOD por faixas)
Clouds.cpp/.h     — nuvens volumétricas (billboard instanced)
AirportManager.cpp— CSV de aeroportos, luzes de pista, PAPI
PostFX.cpp/.h     — framebuffer, bloom (UnrealBloom 2-pass), fog
```

### Loop de simulação

```
Input (GLFW)
    │ aileron / elevator / rudder / throttle / trim
    ▼
FlyByWire::update()      ← lei C* pitch, rate demand roll, yaw damper
    │ SurfaceCmd (deflexões normalizadas)
    ▼
FDM::setControlsE195()   ← escreve /fadec/throttle-cmd, superfícies no JSBSim
FDM::step()              ← FGFDMExec::Run() × N steps (JSB_HZ = 120 Hz)
    │
FDM::getTelemetry()      ← pitch, roll, yaw, CAS, N1/N2, alt AGL, ...
    ▼
GraphicsEngine::render() ← terreno, céu, aeronave wireframe, HUD ImGui
```

### Throttle / FADEC

O canal FADEC no `flight-control.xml` lê `/fadec/throttle-cmd[N]` (barra inicial obrigatória no SimGear). O bridge escreve via:

```cpp
_exec->SetPropertyValue("/fadec/throttle-cmd[0]", thrNorm);
```

Usar `GetPropertyManager()->GetNode("fadec/throttle-cmd[0]")` (sem barra) resolve para um nó diferente no SimGear e o canal FADEC lê zero — isso é o comportamento correto do SimGear, não um bug.

### FBW Normal Law (E195-E2)

| Eixo | Lei | Referência |
|------|-----|-----------|
| Pitch | C* = 0.8·q\_rad\_s + 0.2·Nz | Airbus/Embraer FBW típico |
| Roll | Rate demand (PD) | Taxa de rolagem proporcional ao comando |
| Yaw | Yaw damper (washout 1 Hz) | Suprime Dutch roll |

---

## Controles (teclado)

| Tecla | Ação |
|-------|------|
| `↑` / `↓` | Profundor (pitch) |
| `←` / `→` | Aileron (roll) |
| `A` / `D` | Leme (rudder) |
| `W` / `S` | Throttle ±0.5/s |
| `B` | Freio |
| `Num 8` / `Num 5` | Trim de pitch |
| `P` | Pausa / menu de reposicionamento |
| `Tab` | Alterna câmera (chase / cockpit) |
| `Esc` | Sair |
