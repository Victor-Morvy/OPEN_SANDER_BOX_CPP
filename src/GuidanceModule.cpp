#include "GuidanceModule.h"
#include <algorithm>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────

void GuidanceModule::engageAttitude(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    targets.pitchDeg = st.pitchDeg;
    targets.bankDeg  = st.rollDeg;
    _pitchInteg      = 0.f;
    _columnFilt      = 0.f;
    fbw.setTargetBank(targets.bankDeg);
    mode.vert = VertMode::AttitudeHold;
}

void GuidanceModule::engageAltitude(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    targets.altFt    = st.altBaro;
    targets.bankDeg  = st.rollDeg;
    targets.pitchDeg = st.pitchDeg;
    _pitchInteg      = 0.f;
    _vsInteg         = st.pitchDeg;  // parte do pitch atual — engate sem degrau
    _columnFilt      = 0.f;
    fbw.setTargetBank(targets.bankDeg);
    mode.vert = VertMode::AltitudeHold;
}

void GuidanceModule::engageHeading(const FlyByWire::AircraftState& st)
{
    targets.headingDeg = st.hdgDeg;
    mode.lat = LatMode::HeadingHold;
}

void GuidanceModule::engageLNAV()
{
    if (fplan.empty()) return;
    if (activeWpt >= (int)fplan.size()) activeWpt = 0;
    mode.lat = LatMode::Nav;
}

void GuidanceModule::engageFlch(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    // Alvo de altitude vem de targets.altFt (campo ALT do painel);
    // alvo de velocidade vem de targets.speedKt (campo SPD do painel)
    targets.bankDeg  = st.rollDeg;
    targets.pitchDeg = st.pitchDeg;
    _flchInteg       = st.pitchDeg;   // parte do pitch atual — engate sem degrau
    _flchThrTrim     = 0.f;
    _pitchInteg      = 0.f;
    _columnFilt      = 0.f;
    fbw.setTargetBank(targets.bankDeg);
    mode.vert = VertMode::Flch;
}

void GuidanceModule::engageApproach(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    // LOC (lateral) intercepta geometricamente qualquer desvio — não precisa
    // de arm/capture (ver update()). GS (vertical) É arm/capture de verdade:
    // "armado" = segura a altitude do engage (como ALT HOLD) até a rampa de
    // 3° cruzar essa altitude vindo de BAIXO; só então "captura" e passa a
    // seguir gsDevDeg. Sem isso, ativar o APP acima ou muito longe da rampa
    // fazia o avião "descer errado" perseguindo um desvio enorme desde o
    // primeiro frame (bug reportado) em vez de esperar alinhar.
    targets.bankDeg  = st.rollDeg;
    targets.pitchDeg = st.pitchDeg;
    targets.altFt    = st.altBaro;   // ARM: altitude a manter até capturar o GS
    _pitchInteg      = 0.f;
    _vsInteg         = st.pitchDeg;
    _columnFilt      = 0.f;
    _gsCaptured      = false;
    _gsPrevDev       = 999.f;
    fbw.setTargetBank(targets.bankDeg);
    mode.lat  = LatMode::Approach;
    mode.vert = VertMode::Approach;
}

void GuidanceModule::engageVnav(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    (void)fbw;   // VNAV só atua no eixo vertical — lateral fica com quem já estiver selecionado
    vnavTargetWpt = -1;
    for (int i = std::max(activeWpt, 0); i < (int)fplan.size(); ++i)
        if (fplan[i].hasAlt) { vnavTargetWpt = i; break; }
    if (vnavTargetWpt < 0) return;   // nenhum waypoint com restrição à frente — não engaja

    targets.pitchDeg = st.pitchDeg;
    _pitchInteg       = 0.f;
    _vsInteg          = st.pitchDeg;
    _columnFilt       = 0.f;
    mode.vert = VertMode::Vnav;
}

void GuidanceModule::engageGoAround(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    // TOGA: procedimento padrão — asas niveladas na proa atual (a da pista,
    // já que o avião estava alinhado no LOC), potência de arremetida fixa
    // (ver update()) e atitude fixa até subir 1500 ft acima da altitude de
    // início (aproximação simplificada da altitude de missed approach —
    // este simulador não tem procedimentos publicados). A/THR sai porque a
    // potência do TOGA é fixa (firewall), não uma velocidade perseguida.
    engageHeading(st);
    disengageThrottle();
    targets.pitchDeg   = st.pitchDeg;
    targets.bankDeg    = st.rollDeg;
    _pitchInteg         = 0.f;
    _vsInteg            = st.pitchDeg;
    _columnFilt         = 0.f;
    _gsCaptured         = false;
    _gsPrevDev          = 999.f;
    _goAroundTargetAlt  = st.altBaro + 1500.f;
    fbw.setTargetBank(0.f);
    mode.vert = VertMode::GoAround;
}

void GuidanceModule::engageAP(const FlyByWire::AircraftState& st, FlyByWire& fbw)
{
    if (!isEngaged())        // nenhum modo selecionado: cai no básico (ROLL+FPA)
        engageAttitude(st, fbw);
    apCoupled = true;
}

void GuidanceModule::engageSpeed(const FlyByWire::AircraftState& st, float currentThrottle)
{
    targets.speedKt  = st.casKt;
    _baseThrottle    = currentThrottle;
    _throttleInteg   = 0.f;
    _thrBoost        = 0.f;
    _errTimer        = 0.f;
    _lastSpdErr      = 0.f;
    mode.thr = ThrMode::SpeedHold;
}

void GuidanceModule::disengageVert()
{
    mode.vert     = VertMode::Off;
    vnavTargetWpt = -1;
    _pitchInteg   = 0.f;
    _vsInteg      = 0.f;
    _flchInteg    = 0.f;
    _flchThrTrim  = 0.f;
    _gsCaptured   = false;
    _gsPrevDev    = 999.f;
}

void GuidanceModule::disengageLat()
{
    mode.lat = LatMode::Off;
}

void GuidanceModule::disengageThrottle()
{
    mode.thr        = ThrMode::Off;
    _throttleInteg  = 0.f;
    _thrBoost       = 0.f;
    _errTimer       = 0.f;
}

void GuidanceModule::disengageAll()
{
    disengageVert();
    disengageLat();
    disengageThrottle();
}

// ─────────────────────────────────────────────────────────────────────────────

bool GuidanceModule::update(float dt, const FlyByWire::AircraftState& st,
                            FlyByWire::PilotInput& inp, FlyByWire& fbw, Output& out)
{
    out = {};

    // ── Transferência bump-free do servo de pitch ao acoplar o AP ────────────
    if (apCoupled && !_prevApCoupled) {
        _pitchInteg = 0.f;
        _columnFilt = 0.f;
    }
    _prevApCoupled = apCoupled;

    // ── AP acoplado sem nenhum modo selecionado → cai no básico (ROLL+FPA) ───
    // Mesma regra do avião real: desengajar o único modo ativo não desliga o
    // AP, ele volta pro modo básico. Cobre tanto quem chamou engageAP() sem
    // nada selecionado quanto quem desengajou o último modo com AP acoplado.
    if (apCoupled && mode.lat == LatMode::Off && mode.vert == VertMode::Off)
        engageAttitude(st, fbw);

    // ── Auto-desconexão por intervenção do piloto ─────────────────────────────
    // Só desacopla o AP (equivalente ao quick disconnect) — os modos do FD
    // continuam selecionados/computando, exatamente como no avião real
    // ("AP DISENGAGE NÃO afeta o FD, só descola dos servos").
    if (std::abs(inp.column) > DISC_THRESH || std::abs(inp.wheel) > DISC_THRESH) {
        apCoupled = false;
    }

    // ── A/THR: auto-desconexão após 3 s consecutivos de peso nas rodas ───────
    // Pouso completo → não faz sentido o A/THR continuar segurando velocidade
    // no solo (o piloto já está gerenciando reverso/freio manualmente).
    if (st.wow) {
        _wowTimer += dt;
        if (_wowTimer > 3.f && mode.thr == ThrMode::SpeedHold)
            disengageThrottle();
        // ── AP: auto-desconexão completa após o pouso ─────────────────────────
        // Autoland real desliga o AP sozinho logo após o toque — não faz
        // sentido continuar "voando" a lei do flare com o avião no solo.
        if (mode.vert == VertMode::Flare && _wowTimer > 2.f) {
            disengageAP();
            disengageVert();
            disengageLat();
        }
    } else {
        _wowTimer = 0.f;
    }

    // ── LNAV: bearing ao waypoint ativo → heading target ─────────────────────
    if (mode.lat == LatMode::Nav) {
        if (activeWpt >= (int)fplan.size()) {
            // Fim do plano: mantém a proa atual (vira HeadingHold)
            targets.headingDeg = st.hdgDeg;
            mode.lat = LatMode::HeadingHold;
        } else {
            constexpr double D2R = 3.14159265358979 / 180.0;
            const Waypoint& w = fplan[activeWpt];
            double dNorth = (w.lat - st.latDeg) * 60.0;                        // NM
            double dEast  = (w.lon - st.lonDeg) * 60.0 * std::cos(st.latDeg * D2R);
            navDistNm = (float)std::sqrt(dNorth*dNorth + dEast*dEast);
            navBrgDeg = (float)std::fmod(std::atan2(dEast, dNorth) / D2R + 360.0, 360.0);

            // Sequenciamento: dentro de 1.5 NM avança para o próximo waypoint
            if (navDistNm < 1.5f)
                activeWpt++;
            else
                targets.headingDeg = navBrgDeg;
        }
    }

    // ── Heading Hold / LNAV / APP LOC: erro → bank_demand → FBW attitude hold ─
    if (mode.lat == LatMode::HeadingHold || mode.lat == LatMode::Nav ||
        mode.lat == LatMode::Approach) {
        float bankLimit = lowBank ? 15.f : 25.f;
        constexpr float KP_HDG = 3.0f;
        float bankDemand;
        if (mode.lat == LatMode::Approach) {
            // Armado (sinal fora do cone): nivela as asas até capturar.
            // Capturado: NÃO comanda bank direto por locDevDeg (isso satura
            // no bank máx pra qualquer desvio grande e pode estourar/orbitar
            // em vez de convergir — reportado: avião assentava numa proa bem
            // longe do curso, ex. curso 308° virava ~247°). Em vez disso,
            // computa uma PROA de interceptação (curso + ângulo proporcional
            // ao desvio, capado em 45°) e usa a MESMA lei de erro de proa do
            // HDG — o ângulo de intercept encolhe pra 0 conforme o desvio
            // some, convergindo suave até alinhar com o curso.
            constexpr float KP_INTERCEPT   = 4.0f;
            constexpr float MAX_INTERCEPT  = 45.f;
            if (ils.valid) {
                float interceptDeg = std::clamp(KP_INTERCEPT * ils.locDevDeg,
                                                -MAX_INTERCEPT, MAX_INTERCEPT);
                float desiredHdg = ils.courseDeg + interceptDeg;
                float hdgErr = desiredHdg - st.hdgDeg;
                while (hdgErr >  180.f) hdgErr -= 360.f;
                while (hdgErr < -180.f) hdgErr += 360.f;
                bankDemand = std::clamp(KP_HDG * hdgErr, -bankLimit, bankLimit);
            } else {
                bankDemand = 0.f;
            }
        } else {
            float hdgErr = targets.headingDeg - st.hdgDeg;
            // normaliza para -180..+180
            while (hdgErr >  180.f) hdgErr -= 360.f;
            while (hdgErr < -180.f) hdgErr += 360.f;
            bankDemand = std::clamp(KP_HDG * hdgErr, -bankLimit, bankLimit);
        }
        targets.bankDeg  = bankDemand;   // sempre atualizado → alimenta fd.bankBar
        if (apCoupled) {
            fbw.setTargetBank(bankDemand);
            inp.wheel = 0.f;
        }
    }

    // ── Altitude Hold / APP GS / VNAV / Flare: cascata alt|GS|path → VS → pitch
    if (mode.vert == VertMode::AltitudeHold || mode.vert == VertMode::Approach ||
        mode.vert == VertMode::Vnav || mode.vert == VertMode::Flare) {
        constexpr float KP_ALT      = 1.6f;
        constexpr float KP_VS       = 0.009f;
        constexpr float KI_VS       = 0.0006f; // integrador: elimina erro de regime do VS
        constexpr float PITCH_RATE  = 4.0f;   // °/s
        constexpr float KP_GS       = 120.f;   // fpm por grau de desvio — só a CORREÇÃO
        constexpr float TAN_3DEG    = 0.05241f;
        constexpr float GS_CAPTURE_DEG = 0.35f; // janela de captura do glideslope
        constexpr float FLARE_AGL_FT   = 50.f;  // altura AGL onde a rampa vira flare
        constexpr float FLARE_TAU      = 4.5f;  // s — h_dot = -h/tau (lei clássica de flare)

        float vsDemand;
        if (mode.vert == VertMode::Approach) {
            // GS é arm/capture DE VERDADE — não basta o sinal estar "válido"
            // (dentro do cone/alcance): precisa cruzar a janela de captura
            // vindo de BAIXO (_gsPrevDev > janela → dev atual <= janela). Isso
            // é o que corrige o bug de ativar o APP numa altitude errada: se
            // o avião está acima da rampa, o desvio começa negativo e nunca
            // cruza a janela por baixo — fica ARMADO segurando a altitude do
            // engage (como ALT HOLD) para sempre, em vez de "descer errado"
            // perseguindo um desvio enorme desde o primeiro frame.
            if (ils.valid) {
                if (!_gsCaptured && _gsPrevDev > GS_CAPTURE_DEG &&
                    ils.gsDevDeg <= GS_CAPTURE_DEG) {
                    _gsCaptured = true;
                    _vsInteg    = st.pitchDeg;   // transferência bump-free arm→capture
                }
                _gsPrevDev = ils.gsDevDeg;
            }

            if (_gsCaptured && ils.valid) {
                // Capturado: baseline = taxa de descida que a rampa de 3° exige
                // NA VELOCIDADE ATUAL (kt → ft/min: ×101.3), mais a correção
                // proporcional ao desvio. Sem o baseline, desvio pequeno (already
                // tracking) demandava VS≈0 — nivelava em vez de descer a rampa.
                float baselineVsFpm = -st.casKt * 101.3f * TAN_3DEG;
                vsDemand = std::clamp(baselineVsFpm + KP_GS * ils.gsDevDeg,
                                      -1200.f, 600.f);
                if (st.altAgl < FLARE_AGL_FT) {
                    // Perto do chão: a rampa de 3° levaria a um pouso duro —
                    // transfere pro flare (lei própria, ver branch abaixo).
                    mode.vert = VertMode::Flare;
                    _vsInteg  = st.pitchDeg;
                }
            } else {
                // ARM: ainda não capturou (ou sinal fora do cone) — segura a
                // altitude capturada no engageApproach, exatamente como ALT
                // HOLD, até a rampa "subir" até o avião.
                float altErr = targets.altFt - st.altBaro;
                vsDemand = std::clamp(KP_ALT * altErr, -MAX_VS_FPM, MAX_VS_FPM);
            }
        } else if (mode.vert == VertMode::Flare) {
            // Lei clássica de flare: sink demandado decai exponencialmente
            // com a altura — nunca chega a zero de verdade (assintótico), o
            // toque acontece fisicamente quando o trem encosta (WOW), não
            // quando este VS demand zera.
            vsDemand = -std::max(st.altAgl, 0.f) / FLARE_TAU * 60.f;
        } else if (mode.vert == VertMode::Vnav) {
            // Recalcula o próximo waypoint com restrição de altitude — o LNAV
            // pode sequenciar o fplan em paralelo (VNAV não mexe em activeWpt).
            vnavTargetWpt = -1;
            for (int i = std::max(activeWpt, 0); i < (int)fplan.size(); ++i)
                if (fplan[i].hasAlt) { vnavTargetWpt = i; break; }

            if (vnavTargetWpt < 0) {
                // Sem mais restrição à frente: captura na altitude atual
                mode.vert     = VertMode::AltitudeHold;
                targets.altFt = st.altBaro;
                vsDemand      = 0.f;
            } else {
                constexpr double D2R = 3.14159265358979 / 180.0;
                const Waypoint& w = fplan[vnavTargetWpt];
                double dNorth = (w.lat - st.latDeg) * 60.0;
                double dEast  = (w.lon - st.lonDeg) * 60.0 * std::cos(st.latDeg * D2R);
                float distNm  = (float)std::sqrt(dNorth * dNorth + dEast * dEast);
                float altErr  = w.altFt - st.altBaro;   // + = precisa subir

                if (distNm < 1.f || std::abs(altErr) < 100.f) {
                    // Captura: perto do waypoint e/ou da altitude → segue reto
                    mode.vert     = VertMode::AltitudeHold;
                    targets.altFt = w.altFt;
                    vsDemand      = 0.f;
                } else {
                    // VS necessário pra bater a restrição na distância restante,
                    // dado o CAS atual (≈ groundspeed, sem modelo de vento) —
                    // recalculado todo frame, então se autocorrige sozinho.
                    float gsKt = std::max(st.casKt, 60.f);
                    float timeToWptMin = distNm / gsKt * 60.f;
                    vsDemand = std::clamp(altErr / std::max(timeToWptMin, 0.1f),
                                          -MAX_VS_FPM, MAX_VS_FPM);
                }
            }
        } else if (targets.vsManual) {
            vsDemand = std::clamp(targets.vsFpm, -MAX_VS_FPM, MAX_VS_FPM);
            float altErr = targets.altFt - st.altBaro;
            if (std::abs(altErr) < 200.f)
                targets.vsManual = false;
        } else {
            float altErr = targets.altFt - st.altBaro;
            vsDemand = std::clamp(KP_ALT * altErr, -MAX_VS_FPM, MAX_VS_FPM);
        }

        // PI no VS: P amortece, I acumula o pitch de trim necessário para
        // sustentar o VS demandado (P puro deixava ~30% de déficit: 543/800)
        // Anti-windup POR SATURAÇÃO: congela o integrador apenas quando o pitch
        // demandado já saturou no mesmo sentido do erro. (Congelar por magnitude
        // do erro causava dois defeitos: windup mascarado na subida E deadlock
        // após captura com o integrador preso no trim errado.)
        float vsErr = vsDemand - st.vsFpm;
        float rawUn = KP_VS * vsErr + _vsInteg;
        if (std::abs(rawUn) < MAX_PITCH_AP || rawUn * vsErr < 0.f)
            _vsInteg += KI_VS * vsErr * dt;
        _vsInteg = std::clamp(_vsInteg, -6.f, 6.f);
        float rawPitch  = std::clamp(KP_VS * vsErr + _vsInteg, -MAX_PITCH_AP, MAX_PITCH_AP);
        // Rate limiter: pitch target não pula — máximo PITCH_RATE °/s
        float delta     = std::clamp(rawPitch - targets.pitchDeg, -PITCH_RATE * dt, PITCH_RATE * dt);
        targets.pitchDeg += delta;
    }

    // ── FLCH: pitch controla a velocidade; throttle cede se o pitch saturar ──
    if (mode.vert == VertMode::Flch) {
        constexpr float FLCH_KP    = 0.15f;   // ° por kt de erro
        constexpr float FLCH_KI    = 0.02f;   // °/s por kt (trim)
        constexpr float PITCH_RATE = 3.0f;    // °/s máx no target
        constexpr float PITCH_CLB  = 17.f;    // teto de pitch na subida
        constexpr float PITCH_DES  = -12.f;   // piso de pitch na descida

        float altErr = targets.altFt - st.altBaro;
        if (std::abs(altErr) < 250.f) {
            // Captura: transição para Altitude Hold, devolve o throttle ao A/THR
            mode.vert        = VertMode::AltitudeHold;
            targets.vsManual = false;
            _vsInteg         = st.pitchDeg;
            _flchThrTrim     = 0.f;
            if (mode.thr == ThrMode::SpeedHold) {
                _baseThrottle  = _flchThr;   // retoma speed hold do throttle atual
                _throttleInteg = 0.f;
                _thrBoost      = 0.f;
            }
        } else {
            bool  climb = altErr > 0.f;
            float pMax  = climb ? PITCH_CLB : 6.f;    // descendo não cabra além de +6
            float pMin  = climb ? -4.f : PITCH_DES;   // subindo não pica além de −4

            // Speed-on-pitch: rápido → nariz sobe; lento → nariz desce
            // (mesma conversão grosseira Mach→kt do A/THR, ver KT_PER_MACH acima)
            float spdErr = targets.speedIsMach
                ? (st.mach - targets.machTarget) * 660.f
                : (st.casKt - targets.speedKt);   // + = rápido demais
            // Anti-windup por saturação (nunca por magnitude do erro)
            float rawUn = FLCH_KP * spdErr + _flchInteg;
            bool  satHi = rawUn >= pMax, satLo = rawUn <= pMin;
            if (!(satHi && spdErr > 0.f) && !(satLo && spdErr < 0.f))
                _flchInteg += FLCH_KI * spdErr * dt;
            _flchInteg = std::clamp(_flchInteg, pMin, pMax);

            float rawPitch = std::clamp(FLCH_KP * spdErr + _flchInteg, pMin, pMax);
            float delta = std::clamp(rawPitch - targets.pitchDeg,
                                     -PITCH_RATE * dt, PITCH_RATE * dt);
            targets.pitchDeg += delta;

            // Throttle dinâmico: a velocidade do A/THR tem prioridade sobre a
            // razão de subida/descida. Cede potência proporcionalmente ao
            // overspeed (era fixo 0.92/0.08 e a velocidade escapava do alvo
            // quando o pitch não bastava — ex: motor forte em baixa altitude)
            if (climb) {
                if (spdErr > 3.f)
                    _flchThrTrim -= 0.015f * spdErr * dt;  // rápido → tira potência
                else if (spdErr < 0.f)
                    _flchThrTrim += 0.10f * dt;            // devagar → devolve
                _flchThrTrim = std::clamp(_flchThrTrim, -0.7f, 0.f);
                _flchThr = 0.92f + _flchThrTrim;
            } else {
                if (spdErr < -3.f)
                    _flchThrTrim += 0.015f * -spdErr * dt; // devagar → põe potência
                else if (spdErr > 0.f)
                    _flchThrTrim -= 0.10f * dt;            // rápido → volta ao idle
                _flchThrTrim = std::clamp(_flchThrTrim, 0.f, 0.7f);
                _flchThr = 0.08f + _flchThrTrim;
            }
        }
    }

    // ── GO-AROUND / TOGA: atitude fixa de arremetida até a altitude alvo ─────
    if (mode.vert == VertMode::GoAround) {
        constexpr float TOGA_PITCH_DEG = 15.f;  // atitude padrão de arremetida
        constexpr float PITCH_RATE     = 5.f;   // °/s — resposta rápida (procedimento de emergência)

        float delta = std::clamp(TOGA_PITCH_DEG - targets.pitchDeg,
                                 -PITCH_RATE * dt, PITCH_RATE * dt);
        targets.pitchDeg += delta;

        if (st.altBaro >= _goAroundTargetAlt) {
            // Capturou a altitude de missed approach → assume Altitude Hold
            mode.vert     = VertMode::AltitudeHold;
            targets.altFt = _goAroundTargetAlt;
            _vsInteg      = st.pitchDeg;
        }
    }

    // ── Loop externo de pitch — servo do AP (compartilhado att/alt/vnav/app) ──
    // targets.pitchDeg já foi computado acima (alimenta fd.pitchBar mesmo sem
    // AP acoplado); só escreve em inp.column se o AP estiver de fato acoplado.
    if (apCoupled && mode.vert != VertMode::Off) {
        constexpr float KP      = 0.022f;  // reduzido: evita oscilação 1 Hz
        constexpr float KI      = 0.003f;
        constexpr float COL_LP  = 0.70f;   // filtro passa-baixo: α alto = mais suave
        float err      = targets.pitchDeg - st.pitchDeg;
        _pitchInteg   += err * dt;
        _pitchInteg    = std::clamp(_pitchInteg, -12.f, 12.f);
        float rawCol   = std::clamp(KP * err + KI * _pitchInteg, -0.8f, 0.8f);
        _columnFilt    = COL_LP * _columnFilt + (1.f - COL_LP) * rawCol;
        inp.column     = _columnFilt;
        inp.wheel      = 0.f;
    }

    // ── Speed Hold (autothrottle) — suspenso durante FLCH/GO-AROUND/FLARE
    // (throttle fixo nesses modos, ver overrides logo abaixo) ────────────────
    if (mode.thr == ThrMode::SpeedHold && mode.vert != VertMode::Flch &&
        mode.vert != VertMode::GoAround && mode.vert != VertMode::Flare) {
        constexpr float KP_SPD    = 0.025f;  // P underspeed [throttle/kt]
        constexpr float KP_OVR    = 0.015f;  // P overspeed — mais suave (piso é prioridade)
        constexpr float KI_SPD    = 0.010f;  // I underspeed [throttle/(kt·s)]
        constexpr float STEP_GAIN = 0.018f;  // throttle adicionado por kt de erro a cada 1 s
        // IAS/Mach: converte o erro de Mach pra "kt equivalente" (escala
        // grosseira, Mach 1 ≈ 660 kt) só pra reaproveitar os mesmos ganhos —
        // não é uma conversão real de Mach→CAS (que dependeria de temp/alt).
        constexpr float KT_PER_MACH = 660.f;
        float spdErr = targets.speedIsMach
            ? (targets.machTarget - st.mach) * KT_PER_MACH
            : (targets.speedKt - st.casKt);

        // Integrador em unidades de throttle: elimina erro em regime nos DOIS sentidos.
        // Underspeed integra 2× mais rápido (nunca ficar abaixo do alvo é prioridade).
        float kI = (spdErr > 0.f) ? KI_SPD : KI_SPD * 0.5f;
        _throttleInteg += spdErr * kI * dt;
        _throttleInteg  = std::clamp(_throttleInteg, -0.6f, 0.6f);

        // Boost de persistência: a cada 1 segundo que o erro não melhora,
        // somar o erro diretamente ao thrBoost (degrau de throttle)
        _errTimer += dt;
        if (_errTimer >= 1.0f) {
            _errTimer = 0.f;
            if (spdErr > 0.5f) {
                // Erro ainda presente e não melhorou mais que 0.5 kt → boost
                float improvement = _lastSpdErr - spdErr;  // positivo = melhorou
                if (improvement < 0.5f)
                    _thrBoost += spdErr * STEP_GAIN;
            } else if (spdErr < -0.5f) {
                // Overspeed: remove boost mais rápido (era 0.02 — não descia nunca)
                _thrBoost -= 0.05f;
            }
            _lastSpdErr = spdErr;
            _thrBoost   = std::clamp(_thrBoost, 0.f, 0.35f);
        }

        // P bidirecional: underspeed forte, overspeed mais suave
        float pTerm = (spdErr >= 0.f) ? spdErr * KP_SPD : spdErr * KP_OVR;
        float thr   = _baseThrottle + pTerm + _throttleInteg + _thrBoost;
        thr              = std::clamp(thr, 0.f, 1.f);
        out.throttle[0]  = thr;
        out.throttle[1]  = thr;
        out.overrideThrottle = true;
    }

    // ── FLCH: throttle fixo (climb power ou idle) ─────────────────────────────
    if (mode.vert == VertMode::Flch) {
        out.throttle[0]      = _flchThr;
        out.throttle[1]      = _flchThr;
        out.overrideThrottle = true;
    }

    // ── GO-AROUND: potência de arremetida fixa no máximo (firewall) ──────────
    if (mode.vert == VertMode::GoAround) {
        out.throttle[0]      = 1.f;
        out.throttle[1]      = 1.f;
        out.overrideThrottle = true;
    }

    // ── FLARE: retarda pra idle — autothrottle real reduz a potência perto
    // do toque, senão o avião não desacelera/afunda pro pouso ────────────────
    if (mode.vert == VertMode::Flare) {
        out.throttle[0]      = 0.f;
        out.throttle[1]      = 0.f;
        out.overrideThrottle = true;
    }

    // ── Flight Director ───────────────────────────────────────────────────────
    // active = algum modo lat/vert SELECIONADO (armado) — independe de
    // apCoupled. A/THR não tem barra própria, então não entra aqui.
    fd.pitchBar = targets.pitchDeg;
    fd.bankBar  = targets.bankDeg;
    fd.active   = isEngaged();

    out.column = inp.column;
    out.wheel  = inp.wheel;

    return isEngaged() || athrEngaged();
}
