# AFCS — Fly-By-Wire + Guidance/Autopilot (módulo exportável)

Núcleo de leis de voo e guidance do E195-E2, desenvolvido e validado no
WebFlight C++ (sandbox com JSBSim). **Zero dependências externas** — apenas
C++17 padrão. Projetado para ser exportado como biblioteca para o E195-E2Sim.

```
afcs/
├── CMakeLists.txt    projeto independente: lib estática `afcs` + testes
├── afcs_test.cpp     suíte offline (modelo cinemático no lugar do JSBSim)
└── README.md         este documento

fontes (compartilhadas com o webflight, em ../src):
├── FlyByWire.h/.cpp       leis FBW Normal/Direct + proteções
└── GuidanceModule.h/.cpp  AFCS: modos, targets, flight director, LNAV, FLCH
```

## Build e teste

```bash
cd afcs
cmake -B build -S .
cmake --build build --config Release
build/Release/afcs_test.exe      # ou: ctest --test-dir build -C Release
```

---

## Arquitetura

```
                      ┌─────────────────────┐
  targets (painel) ──►│   GuidanceModule    │──► Output{column,wheel,throttle}
  AircraftState  ────►│  (loops externos)   │──► FlightDirector{pitch,bank}
                      └────────┬────────────┘
                               │ modifica PilotInput / setTargetBank
                               ▼
                      ┌─────────────────────┐
  PilotInput ────────►│     FlyByWire       │──► SurfaceCmd (superfícies norm.)
  AircraftState ─────►│  (leis + proteções) │
                      └─────────────────────┘
```

O chamador roda, a cada passo de simulação:

```cpp
GuidanceModule gm;   FlyByWire fbw;

FlyByWire::AircraftState st = /* telemetria do FDM */;
FlyByWire::PilotInput    inp = /* stick/manete do piloto */;

GuidanceModule::Output gmOut;
gm.update(dt, st, inp, fbw, gmOut);          // AP modifica inp (column/wheel)
if (gmOut.overrideThrottle) {
    inp.throttle[0] = gmOut.throttle[0];
    inp.throttle[1] = gmOut.throttle[1];
}

FlyByWire::SurfaceCmd cmd;
fbw.update(dt, inp, st, cmd);                // leis FBW → superfícies
/* escrever cmd no FDM */
```

### Contrato de entrada (AircraftState)

| Campo | Unidade | Usado por |
|---|---|---|
| pitchDeg, rollDeg | ° | C*, attitude/bank hold |
| pitchRateDegS, rollRateDegS, yawRateDegS | °/s | C*, rate demand, yaw damper |
| loadFactorNz | g | lei C* |
| betaDeg | ° | auto-rudder (coordenação) |
| casKt | kt | speed stability, A/THR, FLCH |
| altBaro | ft MSL | ALT hold, FLCH |
| altAgl | ft | pitch envelope (>200 ft) |
| vsFpm | fpm | cascata alt→VS |
| hdgDeg | ° (0=N, CW) | HDG hold, LNAV |
| latDeg, lonDeg | ° | LNAV |
| wow | bool | trava de solo (reversor, elevator, rudder) |

---

## FlyByWire — leis (E195-E2: apenas Normal e Direct)

### NORMAL
- **Pitch — C\***: `C* = Nz + (Vo/g)·q`; PID no erro; envelope +30°/−15°
  (pushback com stick neutro, >200 ft AGL)
- **Roll — rate demand**: stick comanda taxa (máx 22°/s); stick neutro =
  attitude hold; proteção 33° (retorno) / 67° (trava)
- **Yaw**: damper (−K·r) + auto-rudder PI no beta (elimina derrapagem residual)
- **Estabilidade de velocidade** (soft, age mesmo com stick):
  - overspeed > 330 kt → comanda nariz para cima
  - underspeed < 150 kt **e** gear UP **e** (flaps ≤ curso 1 **ou** spd brake) →
    nariz para baixo. Configuração de pouso não empurra o nariz.
- **Reversor**: só deploya com WOW (`inp.reverser && st.wow`)

### DIRECT
Stick → superfície puro nos 3 eixos, ganho fixo, sem aumentação.
`fbw.law = FlyByWire::Law::Direct;` — anunciar no HUD/EICAS.

---

## GuidanceModule — modos

| Eixo | Modo | Lei |
|---|---|---|
| vert | **AttitudeHold** | mantém pitch/bank capturados; override sem desengajar |
| vert | **AltitudeHold** | alt → VS (KP=1.6, ±3000 fpm) → pitch (PI, anti-windup por saturação); V/S manual via `targets.vsManual/vsFpm` |
| vert | **Flch** | throttle fixo (0.92 climb / 0.08 idle), pitch segura o CAS (PI); captura a 250 ft → AltitudeHold e devolve throttle ao A/THR |
| lat | **HeadingHold** | erro de proa → bank demand (KP=3.0, máx 25°) |
| lat | **Nav (LNAV)** | bearing ao waypoint ativo → heading; sequencia a 1.5 NM; fim do plano → HeadingHold |
| thr | **SpeedHold (A/THR)** | alvo é PISO de velocidade: PI assimétrico (underspeed 2× mais forte) + boost de persistência |

Auto-desconexão de vert+lat com |column| ou |wheel| > 0.15.
Flight director sempre ativo com qualquer modo ligado.

### Armadilhas conhecidas (aprendidas em teste)

1. **`engageSpeed()` captura a velocidade ATUAL como alvo** — setar
   `targets.speedKt` DEPOIS do engage.
2. **Anti-windup por SATURAÇÃO, nunca por magnitude do erro** — os integradores
   de VS e FLCH congelam apenas quando o pitch demandado saturou no mesmo
   sentido do erro. Congelar por erro grande causou: 27 kt de erro em regime
   (FLCH) e deadlock a 590 ft do alvo pós-captura (ALT HOLD).
3. **P puro não segura V/S** — precisa do integrador de trim (543/800 fpm de
   déficit sem ele).
4. **A/THR precisa reduzir também** — versão só-piso ficou presa 15 kt acima
   do alvo (P zerado em overspeed + integrador sem autoridade).

---

## Ganhos de referência (struct Gains / constexpr)

| Grupo | Valores |
|---|---|
| C* pitch | Kp 1.8, Ki 0.30, Kd 0.15, cstarK 7.0, maxDemand 1.5 g |
| Roll | rollKp 0.12, holdKp 0.03, maxRate 22°/s |
| Yaw | damperK 0.10, betaKp 0.20, betaKi 0.06, filtro α 0.80 |
| Spd stability | VHi 330 kt, VLo 150 kt, K 0.008/kt (±0.5 máx) |
| ALT/VS | KP_ALT 1.6, KP_VS 0.009, KI_VS 0.0006, rate 4°/s, máx ±12° |
| FLCH | KP 0.15 °/kt, KI 0.02, rate 3°/s, thr 0.92/0.08, captura 250 ft |
| A/THR | under KP 0.025 KI 0.010; over KP 0.015 KI 0.005; integ ±0.6 |
| Pitch outer | KP 0.022, KI 0.003, filtro column 0.70 |
| LNAV | KP_HDG 3.0, bank máx 25°, sequência 1.5 NM |

---

## Exportação para o E195-E2Sim

Um esqueleto `AutopilotModule` (padrão `AbstractModule` do projeto) existe em
`apps/E195-E2Sim/src/modules/afcs/`. Passos para ativar lá:

1. Alimentar `ApState` no update a partir do `FlightDynamicsModule`
2. Consumir `ApCommand` no `FlightControlModule` (column/wheel demand somados
   ao stick; throttle override no FADEC)
3. Ligar o painel FCU/FGCP aos targets e engage/disengage
4. Anunciar modos no PFD/FMA

Futuro: extrair `afcs` como lib compartilhada entre os dois projetos
(este CMakeLists já produz a lib estática).
