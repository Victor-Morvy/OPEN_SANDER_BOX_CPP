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
├── GeoProj.h           projeção ÚNICA lat/lon ↔ world (Web Mercator escalado)
├── TileManager.cpp/.h  4 LODs AWS Terrarium elevation + ESRI texture, curvatura
├── Terrain.cpp/.h      mesh fallback checkerboard
├── Clouds.cpp/.h       nuvens billboard instanced
├── AcModel.cpp/.h      modelo 3D ERJ-195 (OBJ+MTL, partes animadas) e cockpit
├── AirportManager.cpp  CSV aeroportos, luzes de pista, PAPI, marcadores HUD,
│                       getRunwaysNear (segmentos p/ minimapa)
├── Navaids.cpp/.h      navaids.csv (OurAirports) — VOR/NDB/DME, grade 1°×1°
├── MiniMap.cpp/.h      mapa raster OSM p/ UI: HSD runtime + picker de teleporte
├── OSMManager.cpp/.h   prédios/estradas/água Overpass API — 7 draw calls (batches)
└── PostFX.cpp/.h       framebuffer, bloom, fog
CMakeLists.txt          build: vcpkg + FetchContent JSBSim
```

---

## Sistema de coordenadas (crítico)

- **Projeção ÚNICA** (`GeoProj.h`): Web Mercator escalado — `geo::toWorld()` /
  `geo::toLatLon()`. TODOS os sistemas (tiles, pistas, avião, OSM) usam estas
  funções. NUNCA converter lat/lon com equiretangular/cos local por conta
  própria: misturar projeções deslocava fileiras de tiles ~23 m entre si longe
  da origem (Rio→SP) e desalinhava a pista da foto de satélite.
- Aircraft sempre na **origem (0,0,0)** do render. Todos os objetos subtraem `acWorld`.
- **X** = Leste, **Y** = altitude AGL (metros), **Z** = Sul (cauda da aeronave)
- `wpos.x/z` derivados da lat/lon do JSBSim a cada frame via `geo::toWorld`
  (não integrados por velocidade — sem drift); `wpos.y = altAgl * FT2M` ← AGL, **não** MSL
- `acMslM = terrainElev_m + wpos.y`
- `TileManager::getElevAt()` retorna MSL bruto do Terrarium; componente Y do input é ignorado.
- `uYBias` uniform sobe/desce o mesh de terreno no espaço mundo.

### Quatro LODs de terreno

```
ultraFarTiles (zoom 7,  9×9,   vgrid 17) — ~1300 km raio — pintor, sem depth write
farTiles      (zoom 12, 17×17, vgrid 33) — ~78 km raio   — depth write, stencil 2
closeTiles    (zoom 15, 17×17, vgrid 65) — ~9.4 km raio  — depth write, stencil 3; referência de altitude (getElevAt)
nearTiles     (zoom 17, 9×9,   vgrid 33) — ~1.3 km raio  — depth write, stencil 4; 1.2 m/px no solo
```

- **Render FINA → GROSSA com stencil por camada** (main.cpp): cada camada grava
  seu peso no stencil (GEQUAL + REPLACE); camada grossa nunca sobrescreve pixel
  da fina, mas preenche buracos onde a fina não carregou. Depth write em
  near/close/far dá oclusão real (montanha esconde o outro lado) sem
  z-fighting entre LODs. ultraFar continua pintor sem depth (fundo).
- **Elevação acima de z15**: Terrarium AWS só existe até z15 — nearTiles (z17)
  busca o PNG do ancestral z15, amostra o sub-quadrante (bilinear) e cacheia o
  pai em memória. Sem isso o tile fica plano em 0 m MSL.
- **Curvatura da Terra** no TM_VERT: `world.y -= d²/(2·6371000)` — horizonte
  físico (~330 km a FL280) esconde a borda do grid; getElevAt (CPU) não é afetado
- **NUNCA usar polygon offset com muitas unidades**: 150 unidades estourava
  depth > 1.0 → fragmentos descartados = "barreira reta" cortando o terreno a
  ~11 km (cockpit near=0.05)
- **AirportManager registra flat areas nas 3 camadas finas** (close, far, near) —
  camada nova de tiles precisa entrar em addAirportGpu/update
- **Fog atmosférico**: `vis = max(40 km, alt×20) × visScale` — dia claro real;
  cor = earthHaze do Sky shader
- Far clip 2000 km; near 0.5 (externa) / 0.05 (cockpit)

### Prédios / OSM (elevação assada)

A elevação MSL é **assada nos vértices** no bake (main thread, `bakeElevation`):
prédios/telhados/água num nível único (centróide, +0.3 de lift; água +0.1),
estradas por vértice (acompanham o relevo). Sem uniform de base em runtime;
`glPolygonOffset(-1,-3)` contra z-fighting (offset grande estoura o depth em
distância).

---

## FBW Normal Law (FlyByWire.cpp)

### Pitch — lei C* (ativo)

```
C*_atual = Nz + gains.cstarK × q_rps
erro     = C*_dem − C*_atual
elevator = −PID(erro)   // negativo = cabrar
```

**Modo solo (wow)**: stick → profundor DIRETO (senão não há rotação na
decolagem — elevator neutro no chão foi bug). Integradores zerados; ao
decolar a C* reinicializa capturando o estado atual (`_initialized=false`).

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
  unidades de throttle, ±0.6) + boost de persistência a cada 1 s. ATENÇÃO:
  engageSpeed() captura a velocidade ATUAL como alvo — setar targets.speedKt
  DEPOIS do engage.
- **FLCH**: throttle fixo (0.92 climb / 0.08 idle, suspende o SpeedHold), pitch
  segura o CAS via PI (KP=0.15, KI=0.02). Captura a 250 ft → AltitudeHold e
  re-inicializa o SpeedHold a partir do throttle do FLCH.
- **LNAV**: fplan de waypoints (lat/lon/nome; painel adiciona por ICAO via
  AirportManager::findAirport). Bearing plano (equiretangular) ao wpt ativo →
  targets.headingDeg → mesma cascata do HeadingHold. Sequencia a 1.5 NM; fim do
  plano → HeadingHold na proa atual.
- **Anti-windup por SATURAÇÃO** (VS e FLCH): o integrador congela apenas quando
  o pitch demandado saturou no mesmo sentido do erro. Congelar por magnitude do
  erro causa deadlock (integrador preso em trim errado após captura) — testado
  offline nos dois loops.
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
├── nav/        — airports.csv, runways.csv, navaids.csv (OurAirports)
└── models/     — erj195.obj + partes animadas + cockpit.obj (AcModel.cpp);
                  erj195.mtl → livery Embraer195.png (blank195.png = branca)
```

### Modelo 3D — decisões (AcModel.cpp / data/models)

- **Overlays coplanares** (mat2 = vidro, mat3 = faixa de janelas): desenhados
  com `glPolygonOffset(-1,-2)` em drawMeshes — sem isso z-fighting com a pele
  da fuselagem. Índices de material seguem a ORDEM dos `newmtl` no erj195.mtl.
- **Reversor tem 2 OBJs**: `erj195_reverser.obj` (esq.) e
  `erj195_reverser_r.obj` (dir., gerado espelhando X + winding invertido +
  UV espelhada em torno do centro dos dois blocos de nacele da textura, que
  são simétricos — logos legíveis). Registrados como `reverser`/`reverser.r`
  no erj195_parts.json. A parte NÃO anima (partAngle retorna 0) — é a manga
  fixa da nacele.

Paths compilados via macros CMake: `AIRCRAFT_PATH`, `ENGINE_PATH`, `SYSTEMS_PATH`, `DATA_PATH`.

---

## Minimapa (MiniMap.cpp) — HSD + picker de teleporte

Mapa raster `tile.openstreetmap.org` (256 px) para a UI, independente do
terreno 3D. Fetch em `std::async` (máx 6 simultâneos, reusa
`TileManager::httpGet`), upload GL em `processUploads()` na **thread
principal**, cache LRU de 220 texturas. Tile ausente desenha o **ancestral
(até 4 níveis) com sub-UV** — sem buraco preto ao trocar zoom. Download que
falha fica marcado (tex=0) e não é repetido na sessão.

- **drawHSD** (tecla M, canto inferior esquerdo, 250 px): heading-up — o mapa
  gira `rot = -hdg` e o avião fica fixo no centro; textos ficam retos. Anel de
  alcance com distância real (`m/px = 40075016·cos(lat)/(256·2^z)`), "N"
  girando no anel, proa no topo. Scroll = zoom z5–16.
- **drawPicker** (menu de pausa P, north-up): arrastar = pan, scroll = zoom
  ancorado no cursor, **clique = lat/lon do teleporte**. Clique a ≤14 px de uma
  cabeceira "gruda" (RwyPick): posição na cabeceira + **proa de decolagem**
  (bearing cabeceira→outra ponta), alt = elev + 8 ft, CAS 0 — main.cpp aplica
  em repoParams. Elevação de cabeceira ausente no CSV cai para a do aeroporto.
- **Overlays** (compartilhados pelos dois mapas via `Xform` com rotação):
  - Pistas: linha com espessura real (`widthM/mpp`, mín 3 px), contorno claro,
    idents nas pontas (zoom ≥12); fonte = `AirportManager::getRunwaysNear`.
  - **Espinha de peixe** nas duas aproximações (pistas >900 m, zoom ≥9/10):
    tracejado central até 10 NM, costelas a cada 1 NM com distância, marker
    beacons OM (3.9 NM, ciano) e MM (0.6 NM, âmbar).
  - POIs: aeroporto = círculo azul; VOR = hexágono magenta; NDB = círculo
    duplo laranja; DME/TACAN = quadrado magenta (fonte `Navaids`, gate zoom ≥8).
- Rotação de tiles: `AddImageQuad` com cantos girados em torno do centro;
  cobertura pelo círculo circunscrito do retângulo do widget.
- Proa de pista é **verdadeira** (bearing geodésico) — bate com o psi do JSBSim;
  o número do ident é magnético (Rio: RWY 10 ≈ proa 074° true).

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

## OSM (OSMManager.cpp) — ativo, 7 draw calls

`osm.update()`/`osm.render()` ativos no main loop. `CELL_DEG = 0.08` (bbox 0.16°).
Cache em `%LOCALAPPDATA%/webflight/osm_cache/` com TTL 7 dias.

**Batching**: em vez de 1 VBO por mesh (~40k draw calls — congelava), os meshes
são incorporados em **7 batches** (`struct Batch`): 5 de paredes (um por variante
de fachada, stride 8 = pos+normal+uv), 1 de telhados+estradas (stride 6 =
pos+**cor por vértice** — shader flat lê atributo, não uniform), 1 de água
(stride 3, translúcida, sem depth write). Fluxo por frame (`uploadPending`):
até 200 meshes da `_staging` passam por `bakeElevation` (adiado se o terreno
não carregou) → `appendToBatch` (cópia CPU, índices com offset de base) →
`syncBatch` sobe **só a região nova** via `glBufferSubData`; o buffer GPU cresce
1.5× quando estoura (id não muda — VAO continua válido). Troca de célula:
`clearMeshes` zera o conteúdo mas mantém buffers/capacidade GPU.

**Água**: só anéis fechados (`pts.front()==pts.back()`) — way aberto (margem de
multipolygon / rio como linha) earcutado vira triângulos degenerados cruzando a
cidade. Baías/oceano (multipolygon gigante) não aparecem — só a foto ESRI.

---

## AFCS como projeto independente (afcs/)

`afcs/` compila FlyByWire+GuidanceModule como lib estática SEM dependências
(só C++17) + suíte de testes offline (`afcs_test`, 12 asserções: estabilidade
de velocidade por configuração, chattering, LNAV, FLCH subida/descida).
Documentação completa da API/ganhos em `afcs/README.md`. Um esqueleto
`AutopilotModule` (padrão AbstractModule, wiring pendente) foi criado no
E195-E2Sim em `apps/E195-E2Sim/src/modules/afcs/`.

```bash
cd afcs && cmake -B build -S . && cmake --build build --config Release
build/Release/afcs_test.exe
```

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

- **Cockpit 3D**: texturas/instrumentos do cockpit.obj ainda genéricos
- **Luzes da aeronave**: nav lights, strobe, landing lights
- **Oclusão além de 78 km**: só ultraFar (z7) não escreve depth — objetos atrás
  de morros a 78+ km não são ocluídos (irrelevante na prática)

## Feito (não re-implementar)

- **Projeção única Web Mercator** (`GeoProj.h`) + **oclusão real do terreno**
  (depth em near/close/far, stencil por camada) — ver "Sistema de coordenadas"
- **OSM batching** (~40k meshes → 7 draw calls) + update/render reativados no
  main loop — ver seção "OSM (OSMManager.cpp)"

- **Reversão de lei FBW** (tecla L alterna NORMAL ↔ DIRECT; HUD anuncia).
  O E195-E2 tem SÓ duas leis — não existe Alternate:
  - Normal: C* + envelope (+30°/−15°), rate demand + bank protection, damper +
    beta, e estabilidade de velocidade: >330 kt nariz sobe; <150 kt COM GEAR UP
    nariz desce (gear down não empurra — pouso é lento por natureza)
  - Direct: stick → superfície puro nos 3 eixos, sem aumentação
- **Reversor**: toggle Y/△ ou tecla R; trava de solo no FBW (`inp.reverser && st.wow`); auto-stow em voo; HUD `REV DEPLOYED`. JSBSim: `reverser-angle-rad = π` → thrust × cos(π) = −1
- **AFCS completo** (GuidanceModule): ALT/HDG/ATT hold, A/THR, flight director, painel F1
- **FBW roll/yaw**: rate demand + attitude hold roll, yaw damper + beta PI
- **Terreno 3 LODs** com curvatura da Terra e horizonte físico
