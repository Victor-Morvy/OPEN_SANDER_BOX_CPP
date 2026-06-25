# WebFlight C++ — Contexto para Claude

## O que é este projeto

Simulador de voo nativo em C++ com física JSBSim e gráficos OpenGL 3.3. Aeronave padrão: **Embraer E195-E2** com FBW Normal Law. É o porte nativo do simulador WebFlight (Node.js/Three.js/JSBSim-WASM).

## Stack rápida

| O quê | Lib | Como entra |
|-------|-----|-----------|
| Janela/input | GLFW 3 | vcpkg |
| OpenGL loader | GLAD | vcpkg |
| Math | GLM | vcpkg |
| UI | Dear ImGui | vcpkg |
| HTTP tiles | libcurl | vcpkg |
| Imagens | stb_image | vcpkg (header-only) |
| FDM | JSBSim | FetchContent (GitHub) |

## Arquivos principais

```
src/
├── main.cpp          game loop, input GLFW, HUD ImGui, câmera
├── FDM.cpp/.h        wrapper FGFDMExec — init, step, telemetria, E195 bridge
├── FlyByWire.cpp/.h  leis FBW: C* pitch / rate demand roll / yaw damper
├── Sky.cpp/.h        shader Preetham, bloom 2-pass
├── TileManager.cpp   grid 9×9 tiles AWS Terrarium elevation + ESRI texture
├── Terrain.cpp/.h    mesh OpenGL, LOD por faixas
├── Clouds.cpp/.h     nuvens billboard instanced
├── AirportManager.cpp CSV aeroportos, luzes de pista, PAPI
└── PostFX.cpp/.h     framebuffer, bloom, fog
CMakeLists.txt        build system (vcpkg + FetchContent JSBSim)
```

## Decisões críticas

### Throttle: barra inicial obrigatória

No SimGear (property tree do JSBSim), `/fadec/throttle-cmd[0]` e `fadec/throttle-cmd[0]` resolvem para **nós diferentes** quando o path vem de dentro do FCS XML.

```cpp
// CORRETO — mesmo path que o FADEC channel usa no flight-control.xml
_exec->SetPropertyValue("/fadec/throttle-cmd[0]", thrNorm);

// ERRADO — cria nó diferente, FADEC channel lê zero
setD("fadec/throttle-cmd[0]", thrNorm);               // sem barra
_exec->GetFCS()->SetThrottleCmd(0, thrNorm);           // não propaga ao canal FADEC
```

O canal FADEC no `flight-control.xml` tem `<input>/fadec/throttle-cmd[0]</input>`. Use sempre a barra inicial.

### FCS roda ANTES do Propulsion

A ordem dos modelos no `FGFDMExec` coloca `FGFCS` antes de `FGPropulsion`. Então:
1. `FGFCS::Run()` → lê `fadec/throttle-cmd`, escreve `ThrottlePos`
2. `LoadInputs(ePropulsion)` → copia `ThrottlePos` para `in.ThrottlePos`
3. `FGTurbine::Calculate()` → lê `in.ThrottlePos[EngineNumber]`

Por isso, o valor precisa estar na property `/fadec/throttle-cmd[N]` antes de cada `_exec->Run()`.

### Engine spool rates ×10

Os valores originais do XML do motor GE CF34-10E eram ~1–23 %/s (lento demais para simulação). Foram multiplicados por 10 nos arquivos em `newGame/public/engine/`:

```xml
<function name="N1SpoolUp">
    <table>
        <independentVar lookup="row">propulsion/engine[0]/n1</independentVar>
        <tableData>
             24   11.0   <!-- original: 1.1 -->
             28   29.0
             46   97.0
             68  154.0
             80  196.0
             94  214.0
            103  236.0
        </tableData>
    </table>
</function>
```

### Propriedades externas — seed antes de LoadModel()

O `flight-control.xml` referencia propriedades que o FlightGear normalmente criaria (ex: `fadec/throttle-cmd`, hidráulico, PCUs). Precisam existir antes de `LoadModel()`. Ver `FDM::seedE195Properties()` em `FDM.cpp`.

O método correto para **criar** a propriedade é:
```cpp
_exec->GetPropertyManager()->GetNode("fadec/throttle-cmd[0]", /*create=*/true)->setDoubleValue(0.0);
```

`SetPropertyValue` não cria — só escreve em nós existentes.

### Pausa e reposicionamento

Ao despausar após mudança de heading, JSBSim mantém `vInertialVelocity` da orientação antiga. Isso gera alpha extremo no primeiro frame. Solução: salvar `vUVW` e `vPQR` no momento da pausa e restaurar no unpause via `SetUVW` / `SetPQR`. Ver `FDM::pause()` e `FDM::resume()`.

### Paths de dados

Os XMLs de aeronave ficam fora deste repositório (`../newGame/public/`). Os paths são compilados via macros CMake:

```cmake
set(AC_ROOT "${CMAKE_SOURCE_DIR}/../newGame/public")
target_compile_definitions(webflight PRIVATE AIRCRAFT_PATH="${_AC_PATH}" ...)
```

## FBW Normal Law (E195-E2)

### C* pitch (FlyByWire.cpp)
```
C* = 0.8 · q_body [rad/s]  +  0.2 · Nz [g]
erro = C*_cmd − C*_atual
elevator = PID(erro)
```

### Rate demand roll
```
rollRate_cmd = cmd_aileron × maxRollRate
erro = rollRate_cmd − p_atual
aileron_cmd = PD(erro)
```

### Yaw damper
```
filtro washout: hz = r − r_prev (high-pass ~1 Hz)
rudder_corr = −Kyd × hz
```

## Comandos de build

```bash
# Configurar (ajuste o path do vcpkg)
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"

# Compilar
cmake --build build --config Release --parallel

# Executável
build/Release/webflight.exe
```

## O que NÃO está neste repo

- `build/` — artifacts compilados, deps JSBSim (~GB)
- `temp/` — arquivos scratch de desenvolvimento
- XMLs de aeronave — ficam em `../newGame/public/aircraft/` e `../newGame/public/engine/`
