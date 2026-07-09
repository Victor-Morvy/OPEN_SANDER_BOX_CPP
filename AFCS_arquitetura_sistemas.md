# AFCS E170/E190 — Arquitetura de Sistemas, Inputs e Transições

> Fonte: transcript do vídeo *"E170/E190 Automatic Flight Control System (AFCS) - Pilot Training Video"* (`E170-E190_AFCS_transcript_EN_clean.txt`).
> Objetivo: servir de base para o desenvolvimento de uma simulação/software dos sistemas de voo automático.
> Onde o vídeo não especifica um comportamento, está marcado como **[não especificado no vídeo]**.

---

## Parte 1 — Hierarquia geral de sistemas

```
AFCS (Automatic Flight Control System)
│
├── FGCS (Flight Guidance Control System)
│   ├── Autopilot (AP)               — 2 canais (1 ativo + 1 hot spare)
│   ├── Flight Director (FD)         — guidance lateral (roll) e vertical (FPA)
│   ├── Yaw Damper (YD)              — Dutch roll damping, independente do AP
│   ├── Turn Coordination            — junto com o YD, comanda o rudder
│   ├── Automatic Pitch Trim         — acoplado ao AP, comanda o horizontal stabilizer
│   └── Mach Trim                    — funciona só com AP desengajado
│
└── TMS (Thrust Management System)   — arquitetura dual redundante
    ├── Autothrottle (AT)            — posiciona os thrust levers
    ├── TRS (Thrust Rating Selection)— calcula o rating do motor por fase de voo
    └── E-TTS (Electronic Thrust Trim System) / TLA Trim
                                     — ajuste fino do AT + sincronização de N1
```

### Interfaces e sistemas externos citados

| Interface | Papel |
|---|---|
| **Guidance Panel (GP)** | Painel principal de seleção de modos (lateral, vertical, speed, flight guidance) |
| **MCDU** | Seleção de canal do AP, página TRS (rating manual, TLA trim), prioridade de canais AT/TRS/E-TTS |
| **Manche (control wheel, ×2)** | AP quick disconnect, TCS, manual pitch trim |
| **Thrust levers** | AT disconnect, TOGA switches, posição TLA (input do AT) |
| **PFD / FMA** | Exibição dos modos ativos/armados e anunciações (verde/âmbar/vermelho, flash) |
| **EICAS** | Mensagens de alerta (ex.: FLT CTRL FAULT) e exibição de thrust rating / N1 |
| **FMS (1 e 2)** | Speed target, VNAV path, navegação lateral |
| **FADEC** | Recebe comandos de sincronização do E-TTS; detecta falha/shutdown de motor; transmite rating e N1 para o EICAS |
| **Servos** | 1 elevator servo, 1 aileron servo, 1 rudder servo |

### Componentes físicos do FGCS (lista do vídeo)

- Guidance panel dual channel
- 2 autopilot quick disconnect switches
- 2 TCS (Touch Control Steering) switches
- 2 TOGA switches
- 4 processing modules
- 1 elevator servo, 1 aileron servo, 1 rudder servo

---

## Parte 2 — Hierarquia de inputs (por origem)

Esta é a árvore de inputs que a simulação precisa modelar. Nível 1 = origem; nível 2 = grupo; nível 3 = sinal/evento.

### 2.1 Sensores / estado do avião (dados contínuos)

| Sinal | Usado por |
|---|---|
| Radio altitude (RA) | AP (limite de roll na aproximação), AT (RETARD < 30 ft), AT engage em voo (> 400 ft AGL) |
| Baro corrected altitude | FD/AP (ALT, ASEL), TRS (transição CLB→CRZ), SRC transfer |
| Altitude pré-selecionada (ALT SEL) | FD/AP, TRS (fase CLB/CRZ) |
| IAS / CAS | AT (HOLD > 60 kt), Mach Trim (> 0.7 Mach), speed control |
| Mach | Mach Trim, speed control (toggle IAS/Mach) |
| Weight-on-wheels / wheel spin | AP (engage inibido no solo), AT (disconnect na transição para solo), TRS (fase TO) |
| Posição de slat/flap | AT (gust compensation com slat/flap > 0; landing config p/ RETARD) |
| Trem de pouso (retraído/estendido) | TRS (transição TO→CLB), AT (landing config) |
| Motores running / falha (via FADEC) | TRS (TO→CLB exige 2 motores; OEI→CON), AT (single engine) |
| Thrust lever angle (TLA) de cada manete | AT (engage TO exige TLA > 50°; disconnect se split > 8°; OVRD), E-TTS |
| Limites VMO / MMO / placards (gear, flap) | AT (speed limiting) |
| N1 (por motor) | TRS/EICAS, E-TTS (sincronização) |
| Vertical speed atual | AT SPDE (schedule de FLCH pequeno) |
| Heading atual | Push-to-sync do HDG SEL |

### 2.2 Guidance Panel (eventos discretos + knobs)

**Seção lateral guidance:**
- Botão APP (arma approach mode, CAT 1/CAT 2)
- Botão NAV (ativa lateral navigation)
- Botão LOW BANK / AUTO BANK (limite de bank: automático ou fixo)
- Botão HDG (ativa heading mode; usa o heading pré-selecionado)
- Knob HDG SEL (seleciona/pré-seleciona heading) + botão central push-to-sync (sincroniza com heading atual)

**Seção flight guidance control:**
- Botão AP engage/disengage
- Botão YD (yaw damper)
- Botão SRC (transfere fonte PFD usada pelo FGCS: VOR/LOC 1, VOR/LOC 2, baro corrected altitude, FMS 1, FMS 2)
- Botão FD off (remove command bars do PFD do lado não acoplado, se AP engajado)

**Seção speed control:**
- Botão AT (arma/desengaja autothrottle)
- Knob externo de speed: posição FMS (target do FMS) / posição MAN (target manual)
- Knob interno de speed: seleciona o valor do target manual (horário aumenta)
- Botão IAS/Mach push-to-change (alterna controle por CAS ↔ Mach)

**Seção vertical guidance:**
- Botão ALT (Altitude Hold; se já ativo, transiciona para FPA)
- Botão VNAV (interceptação/tracking do path vertical do FMS)
- Knob ALT SEL (altitude alvo; horário aumenta) + botão central (exibe altitude em metros no PFD)
- Botão FLCH (Flight Level Change)
- Roda VS (Vertical Speed Select wheel) + botão VS
- Botão FPA + knob FPA SEL

**Outros:**
- 2 course select knobs (um por PFD) + botão central direct-to (curso direto à estação)
- Botão Preview no display controller (preview de nav source com FMS como fonte primária)

**Regra geral dos botões de modo:** apertar engaja; apertar de novo o mesmo botão desengaja.

### 2.3 Manche (control wheel, ×2 — comandante e copiloto)

| Input | Efeito |
|---|---|
| AP quick disconnect (momentâneo) | 1º press: desengaja AP + inicia alarme; 2º press: cancela alarme aural/visual; também usado para acknowledge de falha |
| TCS (Touch Control Steering, momentâneo) | Pressionado: solta clutches do AP e re-sincroniza o FD; solto: AP re-engaja servos |
| Manual pitch trim | Trima o stabilizer manualmente; desativa o Automatic Pitch Trim e inibe o Mach Trim |

### 2.4 Thrust levers

| Input | Efeito |
|---|---|
| AT disconnect button (em cada manete) | Desconecta o AT (1º press) / cancela alarmes (2º press) |
| TOGA switches (×2) | Takeoff / go-around **[detalhes de acionamento não especificados no vídeo]** |
| Movimento manual das manetes | OVRD do AT (sem disconnect); disconnect se passar da posição TO/GA ou se split entre manetes > 8° |

### 2.5 MCDU

| Página/ação | Efeito |
|---|---|
| Setup page — canal do AP | Alterna manualmente o canal ativo do autopilot |
| Página TRS (tecla TRS) | Seleção manual do thrust rating |
| Página TRS — TLA trim ON | Mantém o TLA trim funcionando mesmo com AT desconectado |
| Prioridade de canais | Seleciona canal prioritário do AT, TRS e E-TTS |

### 2.6 Sistemas fonte (dados de outros computadores)

- **FMS 1/2**: speed target (posição FMS do speed knob), VNAV path, navegação lateral (NAV), fonte primária do PFD
- **Nav sources**: VOR/Localizer 1 e 2 (SRC, approach, preview)
- **FADEC**: status dos motores, N1, execução dos comandos de sincronização do E-TTS

---

## Parte 3 — Sistemas separadamente

Formato padrão: **Função | Inputs | Outputs | Modos | Engajamento | Desengajamento/Inibição**.

### 3.1 Autopilot (AP)

- **Função:** controle automático de flight path (pitch via elevator) e roll (via aileron); acopla-se aos modos vertical e lateral do FD; comanda pitch trim para aliviar carga do elevator servo.
- **Inputs:** modos do FD, radio altitude, botão AP (GP), quick disconnect (manche), TCS (manche), canal selecionado no MCDU, estado do YD, weight-on-wheels.
- **Outputs:** comandos ao elevator servo e aileron servo; comandos de trim ao Automatic Pitch Trim; anunciação no FMA (verde; vermelho em falha).
- **Limites de autoridade:**
  - Roll: ±25°; em approach, redução linear de ±25° (200 ft RA) até ±5° (0 ft RA)
  - Pitch: ±20°
- **Arquitetura:** 2 canais, 1 ativo + 1 hot spare; failover automático em falha do canal ativo; troca manual via MCDU.
- **Engajamento:**
  - Manual: botão AP no guidance panel
  - Ao engajar: ativa automaticamente o Automatic Pitch Trim e o YD/Turn Coordination (se ainda não ativos)
  - Se nenhum modo FD ativo: FD entra nos modos básicos FPA (vertical) + Roll Hold (lateral)
  - **Inibido no solo**
- **Desengajamento:**
  - Manual: botão AP no GP ou quick disconnect no manche
  - Automático: falha do sistema AP/FD
  - Ao desengajar: Automatic Pitch Trim desliga junto; **YD permanece engajado**
  - Alarme: aural "autopilot" + anunciação no FMA piscando ≥ 5 s (fluxo completo na Parte 4)

### 3.2 Automatic Pitch Trim

- **Função:** posiciona o horizontal stabilizer para descarregar a força aerodinâmica mantida pelo elevator (offload do AP servo).
- **Inputs:** engajamento do AP, atividade de manual trim, carga no elevator servo.
- **Outputs:** comando de trim ao stabilizer; mensagem EICAS **FLT CTRL FAULT** se a função ficar inoperante.
- **Ativação:** somente com um canal do AP engajado **e** manual trim inativo.
- **Desativação:** desengajamento (manual ou automático) do AP; manual trim ativo.

### 3.3 Mach Trim

- **Função:** compensação automática de trim em alta velocidade (com AP desengajado).
- **Condições para funcionar (todas simultâneas):**
  1. AP **não** engajado
  2. IAS acima de 0.7 Mach
  3. Manual trim do stabilizer não em progresso
  4. Nenhum quick disconnect pressionado
  5. Nenhuma outra função de trim comandando
- **Inibição adicional:** monitor do Mach Trim detecta falha no trim rate command; AP engajado (o autopilot trim assume).

### 3.4 Yaw Damper (YD) / Turn Coordination

- **Função:** amortecimento de Dutch roll e coordenação de curvas; comanda indiretamente o rudder.
- **Inputs:** botão YD no GP; engajamento do AP (ativa o YD junto).
- **Outputs:** comandos ao rudder servo.
- **Independência:** funciona independentemente do AP e do flight guidance.
- **Relações de falha:** desengajar o AP **não** desengaja o YD; falha do YD **não** desengaja nem impede engajamento do AP (desde que a falha não afete a função AP/FD).

### 3.5 Flight Director (FD)

- **Função:** computa comandos de guidance lateral (roll) e vertical (FPA) para exibição (command bars no PFD) e acoplamento ao AP; integrado ao FMS ou comandado manualmente.
- **Inputs:** todos os botões/knobs do GP (Parte 2.2), fonte PFD (SRC), FMS, TCS (re-sincronização).
- **Outputs:** command bars no PFD; modos ativos/armados no FMA; comandos ao AP quando acoplado.
- **Regras de modos:**
  - 1 modo vertical + 1 modo lateral **ativos** por vez
  - Até 2 modos verticais + 1 lateral **armados** por vez
  - Botão engaja; mesmo botão de novo desengaja
  - Modos citados — laterais: Roll Hold (básico), HDG, NAV, APP (CAT 1/CAT 2), Low Bank/Auto Bank (limite); verticais: FPA (básico), ALT, ASEL, VS, FLCH, VNAV, GS (Glide Slope), Overspeed
  - FD pode estar **coupled** ou **uncoupled** ao AP
- **Transições especiais:**
  - AP engajado sem modos FD ativos → FD ativa Roll + FPA (básicos)
  - Botão SRC com AP engajado → FD transiciona para modo básico (Roll/FPA)
  - Botão SRC com AP desengajado → FD cai para **standby** e limpa a fila de comandos + anunciações
  - FD off remove as command bars do lado não acoplado do PFD (se AP engajado)
  - Preview: com FMS como fonte primária, botão Preview exibe o preview pointer; AP intercepta o curso selecionado mantendo FMS como fonte; ao armar APP e interceptar, a fonte primária muda para LOC/back course

### 3.6 Autothrottle (AT)

- **Função:** posiciona automaticamente os thrust levers para controlar thrust em todo o regime de voo; integrado ao FD; **independente do estado do AP**; mantém o avião dentro dos envelopes de thrust e velocidade.
- **Inputs:** botão AT (GP), speed knob (FMS/MAN + valor), IAS/Mach toggle, TLA de cada manete, AT disconnect buttons, radio altitude, IAS, weight-on-wheels/wheel spin, slat/flap, modo vertical ativo do FD, rating do TRS, FADEC (falha de motor), FMS (speed target), VMO/MMO/placards.
- **Outputs:** movimento dos thrust levers; anunciações no FMA (AT verde; LIM âmbar; OVRD verde; AT vermelho em falha); alarme aural "throttle"; mensagem EICAS em disconnect anormal e em HOLD com TLA abaixo de TO/GA.
- **Limites:** thrust limiting pelo rating N1 ativo; speed limiting por low speed e limites estruturais (VMO/MMO, gear, flaps placard); gust compensation eleva o limite inferior de velocidade acima de 1.2Vs em até 5 kt (condições de rajada, slat/flap > 0).
- **Engajamento no solo (todas):** parâmetros necessários válidos + AT capable; modo TO armado; ambas as manetes acima de 50° TLA.
- **Engajamento em voo (todas):** parâmetros válidos + AT capable; botão AT pressionado; avião acima de 400 ft AGL.
- **Modos:**

| Modo | Nome | Comportamento |
|---|---|---|
| SPDT | Speed on Thrust | Manetes comandadas para manter a velocidade selecionada via thrust (climb/descent/cruise). Modos verticais associados: FPA, VS, GS, ALT, ASEL, função FMS |
| SPDE | Speed on Elevator (FLCH) | AT mantém thrust fixo; AP mantém a velocidade via elevator. FLCH pequeno: thrust apenas o necessário (schedule por VS); FLCH grande: climb rating na subida / idle na descida. Modos verticais: FLCH, Overspeed, função FMS |
| TO | Takeoff thrust | Avança as manetes para a posição TO/GA na decolagem |
| GA | Go-around thrust | Posiciona as manetes em TO/GA com rating de go-around |
| HOLD | Takeoff thrust hold | Ativa com modo TO ativo + IAS > 60 kt; servos desenergizados; sem comandos até 400 ft AGL; EICAS se TLA abaixo de TO/GA |
| RETARD | Retard | Reduz manetes a idle no flare; ativa com RA válida < 30 ft + landing config; no touchdown o AT desconecta automaticamente |

- **Estados especiais:**
  - **LIM (Limited Thrust, âmbar):** modo vertical exige mais/menos thrust que o disponível no rating; AT não consegue manter a velocidade; associado ao SPDT
  - **OVRD (Override, verde):** piloto move as manetes manualmente sem causar disconnect; manetes voltam à posição comandada ao soltar; disconnect só se passar da posição TO/GA
- **Operação monomotor:** FADEC detecta falha/shutdown → AT desativa a manete do motor afetado; a manete do motor operante continua ativa; reduzir manete manualmente para simular falha causa disconnect por split de TLA.
- **Disconnect (qualquer uma):**
  1. AT disconnect button em qualquer manete
  2. Botão AT no guidance panel
  3. Diferença de TLA entre manetes > 8°
  4. AT monitor tripped
  5. Parâmetros de sistema necessários inválidos
  6. Transição para solo (weight on wheels ou wheel spin) + manetes em idle + AT em RETARD
  7. Manete movida além da posição TO/GA (durante OVRD)

### 3.7 TRS (Thrust Rating Selection)

- **Função:** determina automaticamente o thrust rating apropriado à fase de voo; **opera independentemente do AT**; ratings e N1 transmitidos pelo FADEC para exibição no EICAS.
- **Inputs:** fase de voo, altitude (AGL e pressão/AFE), modo vertical, motores running, trem de pouso, altitude pré-selecionada vs. baro altitude, seleção manual na página TRS do MCDU.
- **Ratings transmitidos:** Takeoff (TO), Go-Around (GA), Climb 1 (CLB1), Climb 2 (CLB2), Cruise (CRZ), Continuous (CON).
- **Lógica de transição automática (auto-rating):**
  - **TO:** selecionado no solo; mantido abaixo de 400 ft AGL
  - **TO → CLB** quando simultaneamente: mudança de modo vertical detectada + acima de 400 ft AGL + ambos os motores operando + trem recolhido
  - **TO → CLB (fallback):** se nenhuma mudança de modo vertical for detectada, troca a 3.000 ft de altitude pressão acima da elevação do campo (AFE)
  - **Fase CLB:** avião no ar + altitude pré-selecionada acima da baro altitude atual
  - **CLB → CRZ:** avião no ar + baro altitude entre ±100 ft da altitude pré-selecionada por mais de 90 s
  - **OEI (one-engine inoperative): TO → CON** a 3.000 ft AFE
- **Seleção manual:** página TRS no MCDU (tecla TRS).

### 3.8 E-TTS (Electronic Thrust Trim System) / TLA Trim

- **Função:** ajuste fino do AT; sincronização de comando N1; repassa comandos de sincronização de motor ao FADEC.
- **Acoplamento:** acoplado ao AT — desabilitado quando o AT está desengajado (exceção abaixo).
- **TLA Trim — funções:** pequenos ajustes de thrust com autoridade limitada; reduz movimentos excessivos das manetes; sincroniza rotação N1 (conforto).
- **Ativação:** ON sempre que o AT está engajado; **também funciona com AT desconectado** se "TLA trim ON" for setado manualmente na página TRS do MCDU.
- **Redundância:** apenas 1 canal do TMS (1 AT + 1 TRS + 1 E-TTS) operando por vez; prioridade selecionável via MCDU.

### 3.9 Apêndice — Cabin Pressure Controller (fora do AFCS)

- 2 canais idênticos; em **auto**, um controla e o outro fica em standby.
- Inputs do canal ativo: pressão real da cabine (sensores), pressão ambiente, sinais de potência dos motores, informação do trem de pouso, baro correction, elevação do campo de pouso, cruise flight level → calcula a pressão de referência da cabine.
- Em **manual**: ambos os canais vão a standby; pilotos controlam diretamente a outflow valve.
- Atenção: modo manual é limitado por pressão diferencial, **não** por altitude de cabine, e não há despressurização automática no solo.

---

## Parte 4 — Transições e desengajamentos (máquina de estados)

### 4.1 Acoplamentos entre sistemas

```
AP ENGAGE  ──ativa──▶  Automatic Pitch Trim
AP ENGAGE  ──ativa──▶  YD / Turn Coordination (se inativos)
AP ENGAGE  ──se FD sem modos──▶  FD entra em ROLL + FPA (básicos)
AP DISENGAGE ──desliga──▶ Automatic Pitch Trim
AP DISENGAGE ──NÃO afeta──▶ YD (permanece)
YD FALHA   ──NÃO afeta──▶ AP (desde que não atinja a função AP/FD)
AP ENGAGE  ──inibe──▶ Mach Trim (autopilot trim assume)
AT ENGAGE  ──habilita──▶ E-TTS  (AT DISENGAGE desabilita, salvo TLA trim ON no MCDU)
AT         ──independente de──▶ AP (engajamento de um não depende do outro)
TRS        ──independente de──▶ AT
```

### 4.2 Tabela de transições — Autopilot

| Estado origem | Evento | Estado destino / efeito |
|---|---|---|
| AP OFF (solo) | Botão AP | Sem efeito (engage inibido no solo) |
| AP OFF (voo) | Botão AP | AP ON + Pitch Trim ON + YD ON + (FD básico se sem modos) |
| AP ON | Quick disconnect (1º press) | AP OFF; FMA pisca ≥ 5 s; alarme aural "autopilot" |
| — | Quick disconnect (2º press) | Cancela alarme aural + visual |
| AP ON | Desengajamento sem o botão (outra causa) | FMA pisca ≥ 5 s até o piloto pressionar o quick disconnect (acknowledge) |
| AP ON | Falha do sistema AP | Anunciação FMA verde → vermelha; pisca 5 s; fica steady até acknowledge via quick disconnect |
| AP ON (canal A) | Falha do canal ativo | Failover automático para o hot spare |
| AP ON | Piloto seleciona canal no MCDU | Troca manual de canal |
| AP ON | TCS pressionado | Clutches soltos (piloto voa manualmente); FD re-sincroniza |
| TCS pressionado | TCS solto | AP re-engaja servos; FD re-sincronizado |
| AP ON | Botão SRC | FD → modo básico ROLL/FPA (AP continua) |
| AP OFF | Botão SRC | FD → standby; limpa fila de comandos e anunciações |

### 4.3 Tabela de transições — Autothrottle

| Estado origem | Evento | Estado destino / efeito |
|---|---|---|
| AT OFF (solo) | Parâmetros válidos + TO armado + ambas TLA > 50° | AT ON em modo TO |
| AT OFF (voo) | Parâmetros válidos + botão AT + > 400 ft AGL | AT ON (modo conforme FD) |
| TO | IAS > 60 kt | HOLD (servos desenergizados; sem comandos) |
| HOLD | > 400 ft AGL | Volta a comandar (modo conforme fase/FD) |
| HOLD | TLA abaixo de TO/GA | Mensagem EICAS |
| SPDT | Modo vertical FLCH/Overspeed/FMS | SPDE |
| SPDE | Modo vertical FPA/VS/GS/ALT/ASEL/FMS | SPDT |
| SPDT | Thrust disponível insuficiente p/ modo vertical | LIM (âmbar) — velocidade não mantida |
| AT ON | Piloto move manetes manualmente | OVRD (verde) — sem disconnect |
| OVRD | Piloto solta as manetes | Manetes voltam à posição comandada |
| OVRD | Manete além da posição TO/GA | AT DISCONNECT |
| Qualquer | Go-around iniciado | GA — manetes em TO/GA **[gatilho exato (TOGA) não detalhado no vídeo]** |
| Em voo | RA < 30 ft + landing config | RETARD — manetes a idle no flare |
| RETARD | Touchdown | AT DISCONNECT automático |
| AT ON | Disconnect button / botão AT / TLA split > 8° / monitor tripped / parâmetros inválidos / transição p/ solo + idle + RETARD | AT OFF + alarme aural "throttle" + FMA pisca ≥ 5 s |
| AT OFF (alarme ativo) | Disconnect button (2º press) | Cancela alarme visual + aural |
| AT ON | Disconnect anormal | Aural + FMA vermelho piscando + mensagem EICAS; acknowledge cancela alarmes, EICAS permanece |
| AT ON (2 motores) | FADEC detecta falha/shutdown de motor | Manete do motor afetado desativada; a outra segue ativa |

### 4.4 Tabela de transições — TRS (fases de rating)

| Estado origem | Evento | Estado destino |
|---|---|---|
| (solo) | Seleção no solo | TO |
| TO | Mudança de modo vertical + > 400 ft AGL + 2 motores + gear up | CLB |
| TO | 3.000 ft AFE sem mudança de modo vertical | CLB |
| TO (OEI) | 3.000 ft AFE | CON |
| CLB | Baro altitude dentro de ±100 ft da pré-selecionada por > 90 s | CRZ |
| Qualquer | Go-around | GA **[condição exata não detalhada no vídeo]** |
| Qualquer | Seleção manual na página TRS do MCDU | Rating manual |

### 4.5 Fluxo de alarme/acknowledge (padrão comum AP e AT)

```
Desconexão manual (botão dedicado):
  1º press ──▶ sistema OFF + FMA pisca ≥ 5 s + alarme aural
  2º press ──▶ cancela aural + visual

Desconexão sem o botão dedicado / anormal:
  evento ──▶ sistema OFF + FMA pisca (AT anormal: vermelho + EICAS)
  press do botão ──▶ acknowledge (cancela alarmes; EICAS permanece no caso AT)

Falha com sistema engajado (AP):
  falha ──▶ FMA verde → vermelho, pisca 5 s ──▶ steady ──▶ acknowledge via botão
```

---

## Parte 5 — Caminho de desenvolvimento da simulação

Ordem derivada da hierarquia de dependências: primeiro o que não depende de nada (estado do avião), por último o que depende de tudo (integração e alarmes).

### Fase 0 — Modelo de estado do avião (sem lógica)
Estruturas de dados para todos os sinais da Parte 2.1 (RA, baro alt, IAS/Mach, WOW, flaps, gear, motores, TLA, N1, VS, heading) + valores simulados/mock.
**Pronto quando:** o estado pode ser lido/escrito e avançado no tempo (tick).

### Fase 1 — Barramento de inputs
Eventos discretos do Guidance Panel (Parte 2.2), manche (2.3), thrust levers (2.4) e MCDU (2.5), publicados num barramento/fila que os sistemas consomem.
**Pronto quando:** cada botão/knob gera evento identificável e testável.

### Fase 2 — Sistemas sem dependências
- **TRS**: máquina de estados pura da Parte 4.4 (fases TO/CLB/CRZ/CON/GA + manual)
- **YD/Turn Coordination**: liga/desliga; independência do AP
**Pronto quando:** as tabelas 4.4 passam como testes unitários.

### Fase 3 — Flight Director
Gerenciador de modos (1 vertical + 1 lateral ativos; 2 verticais + 1 lateral armados), modos básicos ROLL/FPA, standby, SRC transfer, FMA (anunciações ativas/armadas).
**Pronto quando:** as regras de modos da Parte 3.5 passam como testes.

### Fase 4 — Autopilot + trims
AP acoplado ao FD (engage/disengage, limites de roll/pitch, inibição no solo, 2 canais + failover), TCS, Automatic Pitch Trim e Mach Trim (com suas condições de ativação/inibição).
**Pronto quando:** a tabela 4.2 passa como testes.

### Fase 5 — Autothrottle + E-TTS
Modos SPDT/SPDE/TO/GA/HOLD/RETARD, estados LIM/OVRD, condições de engage (solo/voo), as 7 condições de disconnect, operação monomotor, E-TTS/TLA trim.
**Pronto quando:** a tabela 4.3 passa como testes.

### Fase 6 — Integração e alarmes
Acoplamentos da Parte 4.1, fluxos de alarme/acknowledge (4.5), FMA completo (cores, flash, steady), mensagens EICAS, redundância de canais (AP e TMS) e seleção via MCDU.
**Pronto quando:** cenários ponta-a-ponta passam — ex.: decolagem completa (TO → HOLD → 400 ft → CLB), aproximação com RETARD e touchdown, falha de motor com OEI → CON, desconexão de AP com acknowledge.

---

*Documento gerado a partir exclusivamente do conteúdo do vídeo transcrito; comportamentos não citados foram marcados como não especificados.*
