# WebFlight C++ — Contexto para Claude

## O que é este projeto

Simulador de voo nativo em C++ com física JSBSim e gráficos OpenGL 3.3. Aeronave padrão: **Embraer E195-E2** com FBW Normal Law. Porte nativo do WebFlight (Node.js/Three.js/JSBSim-WASM).

---

## Stack

| O quê | Lib | Como entra |
|-------|-----|-----------|
| Janela/input | GLFW 3 | vcpkg |
| OpenGL loader | GLAD | vcpkg |
| Math | GLM | vcpkg |
| UI | Dear ImGui | vcpkg |
| HTTP (tiles, OSM) | libcurl | vcpkg |
| Imagens | stb_image | vcpkg (header-only) |
| JSON (OSM) | nlohmann/json | vcpkg |
| FDM | JSBSim | FetchContent (GitHub) |

---

## Arquivos principais

```
src/
├── main.cpp            game loop, input, câmeras, HUD ImGui, painel AFCS (F1)
├── FDM.cpp/.h          wrapper FGFDMExec — init, step, telemetria, E195 bridge
├── FlyByWire.cpp/.h    FBW Normal Law: C* pitch / rate demand roll / yaw damper+beta
├── GuidanceModule.cpp  AFCS: ALT/HDG/ATT hold, A/THR, flight director
├── Sky.cpp/.h          shader Preetham, bloom 2-pass
├── TileManager.cpp/.h  3 LODs AWS Terrarium elevation + ESRI texture, curvatura
├── Terrain.cpp/.h      mesh fallback checkerboard
├── Clouds.cpp/.h       nuvens billboard instanced
├── AirportManager.cpp  CSV aeroportos, luzes de pista, PAPI
├── OSMManager.cpp/.h   prédios + estradas Overpass API (pausado no main loop)
└── PostFX.cpp/.h       framebuffer, bloom, fog
CMakeLists.txt          build: vcpkg + FetchContent JSBSim
```

---

## Sistema de coordenadas (crítico)

- Aircraft sempre na **origem (0,0,0)** do render. Todos os objetos subtraem `acWorld`.
- **X** = Leste, **Y** = altitude AGL (metros), **Z** = Sul (cauda da aeronave)
- `wpos.y = altAgl * FT2M`  ← AGL, **não** MSL
- `acMslM = terrainElev_m + wpos.y`
- `TileManager::getElevAt()` retorna MSL bruto do Terrarium; componente Y do input é ignorado.
- `uYBias` uniform sobe/desce o mesh de terreno no espaço mundo.

### Três LODs de terreno

```
ultraFarTiles (zoom 7,  9×9,  vgrid 17) — ~1300 km raio — sem depth write
farTiles      (zoom 12, 17×17, vgrid 33) — ~78 km raio  — sem depth write
closeTiles    (zoom 15, 9×9,  vgrid 65) — ~10 km raio  — referência de altitude
```

- **Curvatura da Terra** no TM_VERT: `world.y -= d²/(2·6371000)` — horizonte
  físico (~330 km a FL280) esconde a borda do grid; getElevAt (CPU) não é afetado
- **Sem polygon offset**: camadas distantes com `depthWrite=false`, tiles
  ordenados do mais distante para o mais próximo (painter). NUNCA usar polygon
  offset com muitas unidades: 150 unidades estourava depth > 1.0 → fragmentos
  descartados = "barreira reta" cortando o terreno a ~11 km (cockpit near=0.05)
- **Fog atmosférico**: `vis = max(40 km, alt×20) × visScale` — dia claro real;
  cor = earthHaze do Sky shader
- Far clip 2000 km; near 0.5 (externa) / 0.05 (cockpit)

### Prédios / OSM (uBaseY)

Cada `GpuMesh` tem `baseY` = elevação MSL do centróide. Vertex shader: `world.y = localY + uBaseY + 0.3`.  
O `+0.3` é lift fixo para evitar z-fighting com o terreno (`glPolygonOffset(-1,-50)` também aplicado).  
**Bug conhecido**: quando o avião voa sobre terreno com elevação MSL diferente, o baseY pode variar. Correção arquitetural (bake MSL nos vértices em build-time) está pendente.

---

## FBW Normal Law (FlyByWire.cpp)

### Pitch — lei C* (ativo)

```
C*_atual = Nz + gains.cstarK × q_rps
erro     = C*_dem − C*_atual
elevator = −PID(erro)   // negativo = cabrar
```

### Roll — rate demand / attitude hold (ativo)

```
Stick ativo  → demanda taxa (máx 22°/s), P na taxa real
Stick neutro → attitude hold no banco capturado; proteção 33°, limite rígido 67°
```

### Yaw — yaw damper + auto-rudder PI no beta (ativo)

```
rudder = pedals − yawDamperK·r + betaKp·β_filt + betaKi·∫β
```

Beta filtrado (passa-baixo α=0.80); integrador ±8 °·s elimina derrapagem
residual em regime. No solo: pedais diretos, integrador zerado.

### AFCS — GuidanceModule (exportável, sem deps de ImGui/GLFW)

- **AltitudeHold**: alt → VS (KP_ALT=1.6, clamp ±3000 fpm) → pitch via PI
  (KP_VS=0.009, KI_VS=0.0006). Anti-windup: só integra com |vsErr| < 500 fpm,
  clamp ±6° — sem isso, windup na subida causava puxada + oscilação na captura.
  V/S manual configurável pelo painel (targets.vsManual/vsFpm).
- **HeadingHold**: hdgErr → bank demand (KP=3.0, máx 25°) via fbw.setTargetBank.
- **SpeedHold (A/THR)**: alvo é PISO de velocidade. PI bidirecional assimétrico:
  underspeed KP=0.025/KI=0.010, overspeed KP=0.015/KI=0.005 (integrador em
  unidades de throttle, ±0.6) + boost de persistência a cada 1 s.
- Pitch outer loop compartilhado: KP=0.022, KI=0.003, filtro COL_LP=0.70.
- Auto-desconexão vert+lat com |column| ou |wheel| > 0.15.

---

## Decisões críticas (não mudar sem entender)

### Throttle: barra inicial obrigatória no JSBSim

`/fadec/throttle-cmd[0]` e `fadec/throttle-cmd[0]` resolvem para **nós diferentes** no SimGear.  
O canal FADEC no `flight-control.xml` usa `<input>/fadec/throttle-cmd[0]</input>` (com barra).

```cpp
_exec->SetPropertyValue("/fadec/throttle-cmd[0]", thrNorm);  // CORRETO
setD("fadec/throttle-cmd[0]", thrNorm);                       // ERRADO
```

### FCS roda antes de Propulsion

Ordem: `FGFCS::Run()` → escreve `ThrottlePos` → `FGTurbine::Calculate()` lê `in.ThrottlePos`.  
O valor `/fadec/throttle-cmd[N]` deve ser escrito **antes** de cada `_exec->Run()`.

### Propriedades externas — seed antes de LoadModel()

`flight-control.xml` referencia props que o FlightGear criaria. Criar antes de `LoadModel()`:

```cpp
_exec->GetPropertyManager()->GetNode("fadec/throttle-cmd[0]", true)->setDoubleValue(0.0);
// SetPropertyValue não cria — apenas escreve em nó já existente
```

### Pausa e reposicionamento

Ao despausar com heading alterado, JSBSim mantém `vInertialVelocity` da orientação antiga → alpha extremo no primeiro frame.  
Solução: salvar/restaurar `vUVW` e `vPQR` em `FDM::pause()` / `FDM::resume()`.

### Engine spool rates ×10

Valores originais do GE CF34-10E (~1–23 %/s) são lentos demais. Arquivos em `data/engine/` têm valores ×10.

---

## Dados (self-contained em `data/`)

```
data/
├── aircraft/   — XMLs JSBSim (E195, C172P, ...)
├── engine/     — GE CF34-10E, PT6A, ...
├── systems/    — GNCUtilities.xml
├── nav/        — airports.csv, runways.csv
└── models/     — c172p.glb (placeholder 3D)
```

Paths compilados via macros CMake: `AIRCRAFT_PATH`, `ENGINE_PATH`, `SYSTEMS_PATH`, `DATA_PATH`.

---

## Câmera chase (main.cpp)

```cpp
static glm::vec3 sFwd{0,0,-1};
const float alpha = 0.80f;
sFwd = glm::normalize(glm::mix(sFwd, fwdTarget, alpha));  // lerp suaviza viradas
glm::vec3 camPos = -sFwd * 30.f + up * 8.f;
```

`alpha = 0.80` elimina camera shake nas viradas sem atraso perceptível.

---

## Input — Joystick (main.cpp)

Constantes de mapeamento:

```cpp
constexpr int JS_AIL=0, JS_ELV=1, JS_THR=3, JS_RDR=2;
constexpr float JS_DZ=0.08f, JS_THR_RATE=0.6f;
```

Deadzone via rescale linear após threshold (`applyDZ()`).  
Throttle incremental: eixo 3 trata como taxa (unidades/s), não posição absoluta.

### Mapeamento de botões (Xbox / PS)

| Botão | Ação |
|---|---|
| 0 (A / Cruz) | Freio (hold) |
| 2 (X / □) | Trem de pouso (toggle) |
| 3 (Y / △) | Reversor (pré-mapeado, inativo) |
| 4 (LB / L1) | Flaps subir |
| 5 (RB / R1) | Flaps descer |
| 7 (Start) | Pausa |
| 10 (D-pad ↑) | Trim pitch picar |
| 11 (D-pad →) | Trim roll direita |
| 12 (D-pad ↓) | Trim pitch cabrar |
| 13 (D-pad ←) | Trim roll esquerda |

Trim incremental: `JT = 0.25f` unidades/s enquanto botão pressionado.

---

## OSM (OSMManager.cpp) — pausado

`osm.update()` e `osm.render()` estão **comentados** no main loop para evitar freeze (39k+ meshes de upload). `CELL_DEG = 0.08` (8.9 km). Cache em `%LOCALAPPDATA%/webflight/osm_cache/` com TTL 7 dias.

---

## Comandos de build

```bash
# Configurar
cmake -B build -S . \
  -DCMAKE_TOOLCHAIN_FILE="C:/Users/Victor/Documents/Repositories/vcpkg/scripts/buildsystems/vcpkg.cmake" \
  -DCMAKE_BUILD_TYPE=Release

# Compilar
cmake --build build --config Release --parallel

# Matar processo antes de rebuildar (LNK1104 se aberto)
Stop-Process -Name "webflight" -Force -ErrorAction SilentlyContinue
```

Executável: `build/Release/webflight.exe`

---

## Pendências

- **FBW Alternate Law (reversão)**: modo de reversão que remove TODAS as
  proteções do Normal Law (alpha floor, bank protection, pitch envelope) e no
  lugar adiciona:
  - **Proteção de velocidade**: overspeed → puxa o nariz para cima; underspeed
    → baixa o nariz (estabilidade de alta/baixa velocidade, sem hard limits)
  - **Limites de pitch**: máx +30° pitch up, mín −15° pitch down
  - Precisa de: enum de lei ativa (Normal/Alternate) no FlyByWire, gatilho de
    reversão (falha/tecla), anúncio no HUD
- **OSM altitude bug**: bake MSL elevation nos vértices em build-time (ao invés de `uBaseY` runtime); reativar update/render sem freeze de upload
- **Modelo 3D E195**: substituir placeholder C172P (`data/models/erj195_parts.json` + OBJs já existem)
- **Luzes da aeronave**: nav lights, strobe, landing lights
- **Oclusão de objetos distantes**: terreno far/ultraFar não escreve depth — nuvens/luzes de aeroporto a 20+ km podem aparecer na frente de morros que deveriam ocluí-los (raro em cruzeiro)

## Feito (não re-implementar)

- **Reversor**: toggle Y/△ ou tecla R; trava de solo no FBW (`inp.reverser && st.wow`); auto-stow em voo; HUD `REV DEPLOYED`. JSBSim: `reverser-angle-rad = π` → thrust × cos(π) = −1
- **AFCS completo** (GuidanceModule): ALT/HDG/ATT hold, A/THR, flight director, painel F1
- **FBW roll/yaw**: rate demand + attitude hold roll, yaw damper + beta PI
- **Terreno 3 LODs** com curvatura da Terra e horizonte físico
