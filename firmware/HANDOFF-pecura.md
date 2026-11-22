# HANDOFF — R1A Testfirmware 0.4.0 (Testplan Kap. 3 + Erkennung Phase 1)

## Firmware-Stände (Stand 2026-10-05)
| Version | Commit | Was | Auf Hardware |
|---|---|---|---|
| (Bring-up, ID `I1.0.1` fest) | d986538 | Sensor-Telemetrie 30 s, Dauer-MQTT, ohne MCUboot | ✅ lief bis 2026-10-05 auf P0.1.1 |
| 0.1.1 | 32d7ffe | + MCUboot Dual-Slot/FOTA, A-GNSS, P-ID-Schema | ✅ Modem ok (Bisektion) |
| 0.2.0 | 2e53e6b | Modi normal/live/drain, devcfg, PMIC, Telemetrie, Flash-Station | ❌ Modem startet nie (UICR-Bug) |
| 0.3.0 | d781152 | LED Farbe/Blinken | ❌ gleicher UICR-Bug |
| **0.3.1** | e853db1 | Fix: Factory-ID aus Flash-Seite 0x000FF000 statt UICR | ✅ per J-Link geflasht, läuft |
| 0.3.2 | (= 0.3.1, nur Version) | FOTA-Testimage | ✅ **per FOTA** auf P0.1.1 |
| **0.3.3** | 508e13c | GNSS-Erstsuche max. 10 min, dann periodisch | ✅ **per FOTA** auf P0.1.1 (16:08) |
| 0.4.0 | (s. git) | Bewegung 50 Hz (FIFO), Verhalten, Schritte, Gang, Rohdaten-Bursts | ❌ nur gebaut + Host-Test, **noch nicht auf Hardware** |

Builds: `build/merged.hex` (Erstflash mit MCUboot, für die Flash-Station),
`build/CDC-INCO-Bringup-R1A/zephyr/zephyr.signed.bin` (FOTA-Image, in der App
unter FOTA hochladen, Name mit Version, z. B. `pecu-r1a-0.3.3.signed.bin`).

## 0.4.0: Bewegungsmerkmale (docs/PLAN-ERKENNUNG.md Phase 1, Stand 2026-10-24)
Gebaut (pristine sysbuild grün): **FOTA-Image 390 KB / Slot 448 KB**, App-RAM
160 KB / 207 KB (76 %; davon Rohdaten-Puffer 18 KB, Ring 96 Datensätze).

- **IMU:** BMI323 jetzt 50 Hz in den FIFO (`imu.c`, ACC_CONF 0x4017, FIFO_CONF
  0x0200 nur Accel, FIFO_FILL 0x15 in Worten, Dummy-Frame x=0x7F01). Die
  Schleife liest pro 100-ms-Slice die neuen ~5 Frames (Polling, kein
  Interrupt). FIFO fasst 6,8 s — bei langem Attach im Modus normal gehen die
  ältesten Samples verloren, die Filter sehen nur eine Lücke. Registerwerte
  aus dem Datenblatt, **auf Hardware zu prüfen** (`bu imu` setzt den Chip
  zurück → FIFO wird im nächsten Fenster neu gestartet).
- **motion.c** (reines C, Host-Test `tools/motion_test.c`):
  `cc -std=c11 -O2 -Wall -Wextra -Isrc tools/motion_test.c src/motion.c -lm -o build_host/motion_test && ./build_host/motion_test`
  Schwerkraft-Tiefpass 1 s, Schritt-Bandpass 1,2 Hz, Kau-Bandpass 1,25 Hz,
  2-s-Epochen → Klassen graze/rum/walk/run/rest/other, Seitenlage als Merkmal,
  Schritte, Lagewechsel (>30° zwischen Ruhe-Epochen), Gangphasen ≥6 Schritte
  → Frequenz, Regelmäßigkeit, Asymmetrie (Kopfnicken), Nick-Amplitude.
  Schwellen sind erste Regeln; per retained cfg einstellbar (flache Schlüssel,
  ganze Zahlen): `m_still` 15, `m_walk` 40, `m_run` 250, `m_chew` 6 (mg),
  `m_down` 25, `m_lat` 60 (°), Ruhelage vom Server `g0x/g0y/g0z` (mg),
  Nickachse `axx/axy/axz` (×1000), `raw_every` (min, 0 = aus).
- **Neue Telemetriefelder** (nur wenn der FIFO lief): `min_graze, min_rum,
  min_walk, min_run, min_rest, min_other, min_lat` (**Sekunden** je Klasse
  im Fenster, Name historisch), `steps, pos_chg, still_max` (s), `acc_peak`
  (m/s²), `pitch, roll, pitch_sd` (°), `gait_n, gait_f` (Hz), `gait_reg,
  gait_asym, gait_bob` (m/s²), immer: `t_skin_min, t_skin_max, t_skin_sd`.
  JSON jetzt ~1,1 KB (Puffer 1536 B).
- **Rohdaten-Burst:** `{"cmd":"raw","s":60}` auf `projekt/cmd/<id>` (max 60 s)
  → im nächsten Sendezyklus je 2 s ein Block auf `projekt/raw/<id>`:
  `{"b":<Start unix s>,"k":<Block>,"n":<Blöcke>,"hz":50,"d":[x,y,z,...]}` in mg.
  Bestätigte Blöcke werden übersprungen, ein Abbruch setzt beim nächsten
  Zyklus fort. Backend speichert unter `data/raw/<id>/<b>.jsonl`, CSV-Export
  in der Test-Ansicht.

## 2026-10-05: erster Hardware-Lauf + erster echter FOTA
Board: R1A **CU-Platine allein** (P0.1.1, IMEI nur im Admin), VSYS 3,5 V +
VDD_1V8 1,8 V vom Labornetzteil an P400/P401, **keine Power-Platine**
(PMIC → „not present“, Akku-Felder `null`).

**UICR-Bug (0.2.0/0.3.0):** Modem kam nie hoch — `nrf_modem_lib_init` sofort
-116, danach ASSERT `kernel/timeout.c:111`, VSYS-Strom konstant 3 mA. Per
Bisektion (d986538 ✅ → 32d7ffe ✅ → 2e53e6b ❌; Watchdog, MCUboot, PMIC,
hwinfo, Settings einzeln ausgeschlossen) gefunden: `devcfg.c` las die
Factory-ID per TF-M `soc_secure_mem_read` aus dem UICR — das bricht das Modem,
egal ob vor oder nach dem Modem-Init (danach hängt der Aufruf). **Fix 0.3.1:**
ID liegt in der letzten Flash-Seite **0x000FF000** (Bereich EMPTY_1 hinter
settings_storage, gleiche 32-KB-SPU-Region → non-secure direkt lesbar; FOTA
schreibt nie dorthin; Mass-Erase löscht sie, die Station schreibt sie mit jedem
Flash neu). `apps/backend/flash-station/pecura-flash.py` schreibt dorthin —
**die Station muss nach dem NAS-Rebuild neu eingerichtet werden** (lädt den
Agent vom Backend).

**FOTA-Test A-4 ✅ (0.3.1 → 0.3.2, Modus normal):** Rollout 15:52 → Befehl kam
mit der nächsten Telemetrie (15:55:57, Backend-Mailbox) → `fota:started` →
**Download 380 KB in 14 s** → `downloaded` → `rebooting` → **0.3.2 online
15:57:06** (51 s inkl. MCUboot-Swap + Attach). Rollback nicht getestet.
- ⚠️ **Nur über HTTP:** `fw.cadenced.at` (Cloudflare) liefert nur eine
  ECDSA-Kette bis **ISRG Root X2**; das Gerät hat nur ISRG Root X1 (HiveMQ) →
  HTTPS-Handshake scheitert. Darum NAS-`.env`: `FW_BASE_URL=http://fw.cadenced.at`
  (Backup `.env.vor-fw-http`). Integrität sichert MCUboot (Signatur) — aber
  mit dem öffentlichen Default-Key. **Vor der Serie:** Root X2 als eigener
  FOTA_SEC_TAG (Key-Write nur bei CFUN=4!), zurück auf https, eigener Signierschlüssel.

**GNSS-Strom (0.3.3):** Indoor ohne Fix suchte GNSS bis zum ersten Fix
**unbegrenzt kontinuierlich** → 30–50 mA dauerhaft. Jetzt: Erstsuche max.
`FIRST_FIX_TIMEOUT_S` = 600 s (JOB 2026-10-05), danach periodisch nach `gnss_s`
(Suche je max. 120 s), auch ohne Fix. Modus `drain` (gnss=1) bleibt dauernd.

**Debug-Hinweise CU-Board:** J-Link über DK DEBUG OUT → J600. Nach Kaltstart
ist der Chip gesperrt → Connect = Mass-Erase; RTT nur in EINER Sitzung (flash,
`r`, `g`, `Sleep`, `h`, `savebin`). Pin-Reset (`RSetType 2`) NICHT benutzen —
hat den Flash-Vorgang zerschossen und SWD bis zum Neustecken des Kabels
blockiert. „Could not connect“ ohne SW-DP = Kontakt am J600 → neu stecken.

## 0.2.0-Beschreibung (Stand 2026-09-29)

Firmware liefert jetzt alle Felder aus Testplan v3.4 Kap. 3,
hat drei Betriebsmodi und bindet den nPM1300 ein. **Gebaut (pristine sysbuild
grün, FOTA-Image 379 KB / Slot 448 KB), aber noch NICHT auf Hardware getestet** —
PMIC-Teile erst, wenn die Power-Platine da ist.

## Modi (devcfg.c, persistent in Settings)
| Modus | Senden | Verbindung | GNSS | LEDs |
|---|---|---|---|---|
| `normal` (Default) | alle 300 s | pro Zyklus: verbinden → senden → ~4 s auf Befehle hören → trennen → **PSM** | alle 300 s | grün auf Wunsch, blau = lädt, rot = Ladefehler |
| `live` | alle 30 s | Dauer-MQTT, Befehle sofort | alle 120 s | wie normal |
| `drain` | alle 10 s | Dauer-MQTT | **dauernd** | alle 3 an |

- Befehle kommen in `normal` mit max. einem Intervall Verzögerung an. Möglich,
  weil das Backend die Soll-Konfiguration **retained** auf `projekt/cfg/<id>`
  legt (kommt bei jedem Connect) und ein FOTA-Kommando erneut schickt, sobald
  das Gerät Telemetrie sendet (`_fota_mailbox`). PSM + Dauer-MQTT bleibt tabu
  ([[deep-sleep-vs-mqtt]]); hier wird pro Zyklus getrennt.
- `drain` bleibt über Resets gesetzt (D-4: leer → Qi → wieder leer). Danach im
  Dashboard auf `normal` zurückstellen!
- Umschalten: Dashboard → „Test- & Energiesteuerung“ (nur Admin), RTT
  `bu mode drain` / `bu int 60 30`, MQTT `{"cmd":"cfg","mode":"live"}`.
  Das Backend ist maßgeblich: seine retained Config überschreibt lokale Änderungen.

## Telemetrie (telemetry.c) — `projekt/data/<id>`
`device_id, ts, seq, fw, mode, int, up, boot, rst, bl` · `lat, lon, sat, fix,
hacc, speed, cn0, ttff, fix_age` · `acc_x/y/z` (Mittel, m/s²), `act` (Std-Abw.
von |a|) · `t_skin, t_amb, t_sun` (3× Pt1000), `t_bat` (Akku-NTC), `t_pmic` ·
`vbat, ibat (mA, + = laden), soc, chg (0/1/2 = aus/lädt/voll), chg_st, chg_err,
vbus, chg_red` · `rsrp, rsrq, snr, cell, tac, band, plmn` · `led` (Bitmaske).
Ungültig = `null`. Backend: `temp_in` = t_skin, `temp_out` = t_amb,
`battery` = vbat (für die bestehenden Dashboard-Charts).
- **Puffer:** RAM-Ring mit 96 Datensätzen (8 h bei 300 s), Versand ältester
  zuerst, Löschen erst nach PUBACK, Backend nimmt den Geräte-`ts` → A-6/C-2
  passen. Geht bei Reset verloren. Flash-Puffer erst, wenn nach FOTA Platz
  bleibt (Entscheidung JOB 2026-09-29).
- Ohne Netz blockiert die Schleife nicht: nach gescheitertem Attach sucht das
  Modem im Hintergrund, neuer Versuch erst bei Registrierung oder nach 10 min.
- `seq` wird alle 16 Werte gespeichert → nach Stromausfall Sprung < 16, nie
  Wiederholung. `boot` zählt Starts, `rst` = Grund (por/pin/sw/wdt/bor/lockup).
- **Watchdog** 5 min (hängende FW → Reset `rst:"wdt"`, und MCUboot-Rollback
  greift auch bei Hängern). Gefüttert in der Messschleife, vor/nach jedem
  Sendeschritt und in der MQTT-Warteschleife (alles mit Zeitlimit; längster
  Block = LTE-Attach 180 s). Hängt der GNSS-Thread > 2 min, wird nicht mehr
  gefüttert → Reset. Pausiert, wenn der Debugger den Core anhält.

## nPM1300 (Overlay + pmic.c)
- BUCK1 1,8 V / BUCK2 3,0 V per VSET-Widerstand — Software fasst sie nicht an.
  **LDO1 = VDD_GNSS 1,8 V** (aus VDD_3V0), wird beim Boot eingeschaltet. LDO2 unbenutzt.
- LEDs: LED0 rot (LED501), LED1 grün (LED500), LED2 blau (D500), alle „host“.
- **0.3.0:** Soll-Config `led` ist jetzt Farbe + Blinken: 0 aus, 1 grün, 2 blau,
  3 rot, +4 = blinken (Pecura-App → LEDs, mehrere Tiere auf einmal).
  `{"cmd":"led","on":1,"c":6}` setzt sie sofort (live). Blinken läuft als
  Work-Item auf der System-Workqueue (0,3 s an / 0,7 s aus), also auch während
  Attach/Senden; Telemetrie `led` meldet die leuchtenden Farben + bit3 = blinkt.
  Blau leuchtet zusätzlich (dauerhaft) beim Laden, rot bei Ladefehler.
- Akku XINJ PL405070 3000 mAh. Laden 800 mA (nPM-Maximum, ~0,27 C), Ende 4,20 V.
  **Temperaturfenster:** < 0 °C und > 45 °C kein Laden (Hardware-JEITA + Firmware
  zusätzlich), 0–10 °C halber Strom (Hardware), 35–45 °C halber Strom
  (Firmware) + Ladeschluss 4,10 V (Hardware), dazwischen 800 mA.
- ⚠️ **NTC-Typ unbekannt** — angenommen 10 k / B 3435. Vor dem ersten Laden
  mit Multimeter prüfen (weißes Kabel gegen Minus ≈ 10 kΩ bei 25 °C), sonst
  `thermistor-ohms/-beta` im Overlay anpassen. Falscher NTC = falsches Fenster!
- `soc` vorerst aus der Ruhespannung (grob, ±10 %). Für D-5 den Akku mit nPM
  PowerUP profilieren → `src/battery_model.inc` + `CONFIG_NRF_FUEL_GAUGE=y`.
- Ship-Mode: `bu ship` (nur RTT) — aufwecken nur per Qi (SHPHLD nicht verdrahtet).
- Ohne Power-Platine: PMIC „not ready“, Power-Felder `null`, Rest läuft.

## Geräte-ID (Flash-Station in der Pecura-App)
**Flashen nur noch über die Pecura-App → Verwaltung → Flashen.** Die App nimmt
die nächste freie ID; die Flash-Station (`apps/backend/flash-station/pecura-flash.py`
auf dem Mac mit dem J-Link) schreibt Firmware **und** ID in EINER J-Link-Sitzung
(ID in der letzten Flash-Seite `0x000FF000`: `"PID1"` + ID, außerhalb der MCUboot-Slots →
übersteht FOTA), startet das Gerät und liest per Core-Halt den RAM-Block
`PECUFAC1 … PECUEND1` (ID, IMEI, FW; `devcfg_factory_info_update`) zurück. Das
Backend trägt ID ↔ IMEI ins Geräte-Register ein. „Neu flashen …“ am Halsband
behält die ID. Einrichtung: `docs/APP.md` → „Halsbänder flashen“.
- Reihenfolge der ID in der Firmware: Settings-Override (`bu id`, MQTT
  `{"cmd":"id",...}`) → Factory-ID (Flash-Seite) → `IMEI<imei>` (nur Laborboards, die
  ohne Flash-Station geflasht wurden; nie einem Kunden zugeordnet).
- Die IMEI sehen nur Admins (Verwaltung → Halsbänder).
- Firmware für die Flash-Station: `build/merged.hex` (nicht das .signed.bin) —
  die Station bietet den lokalen Build direkt an; sonst in der App unter
  „Werks-Firmware“ als `pecu-r1a-<version>.merged.hex` hochladen.

## RTT-Shell (neu)
`bu status` · `bu mode normal|live|drain` · `bu int <s> [gnss_s]` · `bu id [ID]` ·
`bu ship` · `bu reboot` (Rest wie bisher; `bu mon` entfernt).

## Offen / als Nächstes
1. ✅ 0.3.3 per FOTA (2026-10-05): indoor ohne Fix fällt VSYS-Strom ~12 min
   nach dem Start von ~50 mA auf ~0 (danach nur Suchfenster ≤ 120 s alle 300 s).
2. Mit Power-Platine: Boot-Log prüfen (PMIC erkannt, LDO1 an → GNSS-LNA), `bu status`.
3. NTC messen (s. o.), dann A-7 (Ladesperre im Gefrierschrank).
4. PSM-Grant des Netzes prüfen (A1/LTE-M): Strom zwischen den Meldungen.
5. FOTA über HTTPS (ISRG Root X2) + eigener MCUboot-Key; FOTA-Rollback testen.
6. Flash-Station nach NAS-Rebuild neu einrichten und den ersten Flash über die
   App wiederholen (ID jetzt in 0x000FF000).
7. Firmware hängt im Assert, wenn das Modem nicht startet — sollte sauber
   weiterlaufen bzw. neu starten.
8. Flash-Puffer, falls nach FOTA Platz (~69 KB frei im Slot).
9. `NRF_PROVISIONING_INTERVAL_S` für die Serie auf 86400.

---

# (älter) R1A Bring-up: echte Sensor-Telemetrie (Accel + Pt1000)

Stand 2026-09-17. **CDC-INCO-Prototyp-R1A** (CU-Platine, nRF9151) publisht per
LTE-M alle 30 s Telemetrie an HiveMQ → NAS → Dashboard. Fokus jetzt: **echte
Sensorwerte** statt Platzhalter. GPS ist **beiseitegelegt** (kein Outdoor-Prüf-
platz bis zum PMIC/Akku-Aufbau nächste Woche) — läuft nur noch nebenher.

## Aktueller Flow (main.c) — Sensor-Fokus
1. **LTE-M attach + TLS/MQTT connect** zu HiveMQ → `{"...","status":"online"}`.
2. **BMI323 init** (für die Accel-Mittelung). **GNSS bleibt AUS** (Low-Power,
   `ENABLE_BACKGROUND_GNSS = 0`) → `sat`/`lat`/`lon` = 0. Kein exklusives LTE-Aus-
   Fenster; die Telemetrie fließt sofort. (Makro auf 1 → GNSS nebenher, Outdoor.)
3. **Telemetrie-Schleife, alle `PUBLISH_INTERVAL_S` = 30 s** eine Message. Während
   der 30 s wird **gesampelt und gemittelt** (siehe unten), die MQTT-Verbindung per
   `session_poll` am Leben gehalten, danach genau **eine** Publish mit den
   gemittelten Werten. Session-Drop → transparenter Reconnect.

## Sensor-Implementierung
**Beschleunigung (BMI323, IC203 @0x68, I²C P0.30/31):**
`bringup_imu_read_accel()` liefert einen stillen Sample in g. Die Schleife liest
alle ~20 ms (`SAMPLE_SLICE_MS`) → **~50 Hz**, mittelt **alle 3 Achsen über die
komplette 30-s-Sendedauer** und meldet **m/s²** (`acc_x/y/z`, g × 9.80665). Der
Mittelwert ist bewusste Rausch-/Bewegungs-Mittelung über das Fenster.
> Hinweis: Der Mittelwert der Rohbeschleunigung ≈ mittlere Lage/Schwerkraft­
> projektion. Für ein *Bewegungs­maß* (Aktivität) wäre Std-Abw./VeDBA nötig —
> auf Wunsch nachrüstbar, würde aber ein zusätzliches JSON-Feld bedeuten.

**temp_in = Pt1000 IC201 (ratiometrisch):**
Aus dem Schaltplan-Netz (shematic-R1A-ordered.pdf, Sensors-Sheet) verifiziert:
**IC200→T1→AIN0, IC201→T2→AIN1, IC202→T3→AIN2, V_TEMP→AIN3**. `temp_in` ist also
**IC201 = T2 = AIN1 (P0.14)**. `bringup_temp_in_read()`:
- P0.07-Anregung **gepulst** nur für die Messung (~2 % Duty → keine Selbst­
  erwärmung), Settling ≥450 µs (9 τ, τ≈50 µs) — hier 2 ms.
- **AIN1 (Sensorknoten) UND AIN3 (V_TEMP)** unter **demselben Puls** gesampelt →
  Vref/Vexc/GPIO-Ron kürzen sich im Verhältnis exakt raus.
- **Auflösung — drei gestapelte Tricks (2026-09-17 „genauer"-Runde):**
  1. **Knotenkanäle AIN0/1/2 auf Gain 1/2** (FS 1,2 V statt 2,4 V) → ein ADC-Code
     ist ~0,18 °C wert statt ~0,37 °C (2× feiner). V_TEMP/AIN3 bleibt Gain 1/4,
     weil ~1,8 V nicht in 1,2 V passt. Ratiometrik nutzt mV pro Kanal-Gain → korrekt.
     Knoten bleibt bis ~180 °C unter FS, kein Clipping-Problem.
  2. **256× SAADC-Oversampling + Offset-Kalibrierung** (Schaltplan-Vorgabe).
  3. **Software-Mittel über das 30-s-Fenster, jetzt ~120 Reads** (alle 250 ms
     statt 1 s) → mehr Sub-Code-Auflösung. Duty ~10 %, keine Selbsterwärmung.
  Erwartung: Jitter ~0,3–0,6 °C → grob **2–3× enger**.
- Rpt = 1 kΩ · V_node/(V_top−V_node), Temp via Callendar-Van-Dusen (T ≥ 0 °C).
- **Guard = Plausibilitätsband −50…+150 °C** (nötig seit Gain 1/2: ein offener
  Pt1000 klippt am FS ~1,2 V < V_top, würde sonst als ~256 °C durchrutschen).
  Ungültig → `temp_in` wird 0 gesendet.
- **1-Punkt-Kalibrier-Hook:** `TEMP_IN_OFFSET_C` in temp.c (Default 0) — Referenz
  messen, Differenz eintragen, reflashen → absolute °C ohne Bad-Abgleich.

**Rest (bewusst 0):** `puls`, `temp_out`, `led`, `battery` = 0 (kein Sensor
verbaut / nicht angefordert). `temp_out` = 0, obwohl T2/T3 (AMBIENT/SUN) da wären.

JSON: `{device_id,ts,lat,lon,sat,fix,hacc,speed,acc_x,acc_y,acc_z,puls,temp_out,
temp_in,led,battery}` — Bau in `mqtt.c: build_default_json`, gefüttert über
`bringup_telemetry_set_sensors()`.

## Bestätigt im Live-Log (2026-09-17 ~17:20–17:51)
- **Temp-Auflösung sichtbar besser (Gain-1/2-Runde, commit 78c43a8):** Kurzzeit-
  Jitter von `temp_in` fiel von ~0,2–0,4 °C (Gain 1/4) auf **teils <0,1 °C**
  (z. B. 25,79→25,80→25,81→25,82 in Folge). Es zeichnet jetzt eine **echte
  langsame Drift** sauber nach (25,0 → 25,9 → 25,6 über ~30 min = Eigen-/Raum-
  erwärmung), ohne im Raster zu springen. Grob **2–3× ruhiger** wie vorhergesagt.
- **Pt1000 IC201 ist bestückt und plausibel** (~25 °C Raumtemperatur).
- **Accel-Mittelung + Orientierung funktioniert:** beim Bewegen des Boards springt
  `acc_x/y/z` (z. B. −0,52/−0,30/9,82 → −0,37/−0,33/9,82), |a|≈9,82 m/s² ≈ 1 g
  in Ruhe.
- **Strom-Baseline (Messung User):** **40–45 mA aktiv / 0–3 mA idle** — passt zu
  LTE-M RRC-connected(+GNSS/LNA) vs. RRC-idle (DRX). Der 0–3-mA-Boden ist bereits
  idle-DRX (die CPU idlet via `zsock_poll`, ~95 %). Hebel = Dauer/Häufigkeit des
  40-mA-Fensters senken → **GNSS-Hintergrundbetrieb abschalten** (nächster Schritt).

## Low-Power (2026-09-17 ~17:55) — Sendefrequenz bleibt 30 s
Ziel: möglichst wenig Strom bei gleicher 30-s-Sendefrequenz und **erhaltener
Dauer-MQTT-Verbindung** (PSM/eDRX/RAI bleiben tabu — killen die Always-on-Session,
siehe [[deep-sleep-vs-mqtt]]).
- **GNSS AUS** (`ENABLE_BACKGROUND_GNSS = 0` in main.c) — **der Haupthebel**. Der
  GNSS-Empfänger + externe LNA (SKY65943 via COEX0) war der große Dauerverbraucher
  und fixt indoor eh nie. Ohne GNSS fällt LTE-M zwischen den Publishes auf **idle
  DRX** (~1–3 mA) statt dass die RF busy bleibt. `sat` steht jetzt auf 0. Nächste
  Woche fürs Outdoor: Makro auf 1 → GNSS läuft wieder nebenher.
- **CPU idlet in größeren Blöcken** (`SAMPLE_SLICE_MS` 20→100 ms): der Accel-
  Sampler weckt nur noch ~10×/s, `bringup_mqtt_session_poll` blockiert 100 ms in
  `zsock_poll` (CPU ~98 % idle). Output ist der 30-s-Mittelwert → 10 Hz reicht
  dafür locker. (Für ein Aktivitätsmaß später wieder hochdrehen.)
- **✅ BESTÄTIGT (User-Messung 2026-09-17 ~18:15):** idle jetzt **meist 0 mA**
  (vorher 0–3 mA), **nur beim Senden ~40 mA** kurz. GNSS-aus war der Haupttreiber.
  Der Durchschnitt hängt jetzt fast nur noch an der Länge/Häufigkeit des Sende-
  Bursts (30-s-Takt) — tiefer käme man nur mit RAI (Tail kürzen) oder PSM (bricht
  Always-on-MQTT), siehe „weitere Hebel" unten.
- **Noch drin (weitere Hebel, bewusst NICHT gemacht):**
  1. **RAI (Release-Assistance)** würde den ~40-mA-RRC-Tail nach dem Publish
     sofort beenden (Netz-Inactivity-Timer entfällt) — echter Gewinn, ABER
     [[deep-sleep-vs-mqtt]] verbietet RAI (hat die Session gekillt). Nur mit
     kontrolliertem Test aktivieren, User-Entscheidung.
  2. **BMI323 Low-Power-Duty-Cycle-Mode** statt Normal (~0,2–0,5 mA) — klein,
     und Fehlkonfig würde die Accel-Werte verfälschen → mit Datenblatt separat.
  3. **PSM** ist bei 30 s Intervall + Always-on-MQTT nicht sinnvoll (Re-Attach
     teurer als der Sleep-Gewinn) — daher bewusst nicht.

## GPS (beiseitegelegt, nicht entfernt)
GNSS ist per `ENABLE_BACKGROUND_GNSS` (main.c, aktuell **0**) komplett aus (Strom).
Outdoor fixt es (Beweis: 15:43-Lauf, 47.262972/11.393586, 4 Sat). Der ganze GNSS-
Weg (gnss.c, assistance.c, A-GNSS) und die Fenster-Helfer (`bringup_cell_lte_off()`
CFUN=20 in cell.c, `bringup_gnss_agnss_done()` in gnss.c) bleiben im Code — Makro
auf 1 setzen → läuft wieder nebenher. Für zuverlässig Indoor-Fix bräuchte es
A-GNSS (Onboarding) oder freien Himmel.

## Bauen / Flashen
```
export PATH=/opt/nordic/ncs/toolchains/322ac893fe/bin:$PATH
export ZEPHYR_BASE=/opt/nordic/ncs/v3.2.1/zephyr
export ZEPHYR_SDK_INSTALL_DIR=/opt/nordic/ncs/toolchains/322ac893fe/opt/zephyr-sdk
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
west build -b nrf9151dk/nrf9151/ns --sysbuild
```
Flash **frei laufen lassen, kein Halt** (run.jlink: `connect / loadfile merged.hex
/ r / g / q`). Stand: **FLASH 29.55 %, RAM 50.72 %**.

## FOTA (Firmware-over-the-air) — vorbereitet 2026-09-22

**Status:** Fundament + selbst-gehosteter HTTP(S)-Pfad eingebaut und **gebaut**
(pristine sysbuild grün, Image 340 KB / 76 % des 448-KB-Slots, 107 KB Reserve).
**Noch NICHT auf echter Hardware getestet** (kein Board da) — Bench-Validierung
steht aus (Checkliste unten).

### Was FOTA-fähig macht (das Fundament, muss in JEDE Erst-Charge)
- `sysbuild.conf`: `SB_CONFIG_BOOTLOADER_MCUBOOT=y` → MCUboot + **Dual-Slot**.
  Layout (partitions.yml, alles im internen 1-MB-Flash, **kein externer Flash**):
  mcuboot `0x0`, primary `0x10000..0x80000` (448 KB, enthält TF-M+App),
  secondary `0x80000..0xf0000` (448 KB), settings_storage `0xfa000` (24 KB).
- `prj.conf`: `FOTA_DOWNLOAD` + `DFU_TARGET(_MCUBOOT)` + `IMG_MANAGER` +
  `MCUBOOT_IMG_MANAGER` (Download → secondary-Slot → confirm/rollback).
- ⚠️ **Ein ohne MCUboot geflashtes Gerät kann NIE OTA** — dann nur SWD vor Ort.
  Also von der allerersten Serien-Platine an mit diesem sysbuild flashen.

### Ablauf im Code
- `src/fota.c`: `bringup_fota_start_url()` startet `fota_download_start()` in den
  secondary-Slot. HTTPS nutzt **denselben ISRG-Root-X1-CA auf sec-tag 16842**
  wie HiveMQ (NAS = Let's-Encrypt) → kein zusätzlicher Key-Write.
- Trigger **remote** über MQTT: NAS published auf `projekt/cmd/P0.1.1`
  `{"cmd":"fota","url":"https://.../zephyr.signed.bin"}` → `mqtt.c: handle_command()`.
- Trigger **lokal** (Bench) per RTT-Shell: `bu fota <url>`.
- Fertig → main() published `{"fota":"downloaded"}`, dann `{"fota":"rebooting"}`,
  Reboot → **MCUboot swappt**. Nach dem ersten erfolgreichen Publish der neuen FW
  ruft main() `bringup_fota_confirm()` (`boot_write_img_confirmed`) → Swap wird
  permanent. Crasht die neue FW vorher → **MCUboot-Rollback** auf die alte.
- Status-Topics laufen auf `projekt/data/P0.1.1` mit: `started`/`downloaded`/
  `rebooting`/`error`/`start_failed` → auf dem Dashboard sichtbar.

### Neues Update ausrollen (täglicher Workflow)
1. Version hochziehen und bauen (nur die App ändern → inkrementell reicht):
   ```
   west build -b nrf9151dk/nrf9151/ns --sysbuild
   ```
   Ergebnis: `build/CDC-INCO-Bringup-R1A/zephyr/zephyr.signed.bin` (= das
   signierte FOTA-Image; `build/dfu_application.zip` = dasselbe + Manifest für
   nRF Cloud).
2. `zephyr.signed.bin` aufs NAS legen, per HTTPS erreichbar (Let's-Encrypt-Host,
   z. B. über den bestehenden cloudflared-Tunnel).
3. Kommando publishen (Broker `9d3a3c61...hivemq.cloud:8883`, TLS):
   ```
   mosquitto_pub -h 9d3a3c61e89e4384ab8e326039d9f567.s1.eu.hivemq.cloud -p 8883 \
     --cafile isrgrootx1.pem -u innocowuser -P '<pw>' \
     -t projekt/cmd/P0.1.1 \
     -m '{"cmd":"fota","url":"https://<nas-host>/fw/zephyr.signed.bin"}'
   ```
4. RTT-Log/Dashboard beobachten: `started → 0..100 % → downloaded → rebooting`,
   nach dem Reboot meldet sich die neue FW mit dem nächsten Telemetrie-Publish.

### Bench-Validierung (sobald Hardware da ist — ZUERST machen)
1. Neu bauen (pristine) + `merged.hex` per JLink flashen (enthält jetzt MCUboot).
   RTT: Boot-Zeile `FOTA ready (MCUboot). swap_type=1, running image confirmed`.
2. Version B bauen (z. B. Log-Zeile ändern), `zephyr.signed.bin` lokal per HTTP
   servern (`python3 -m http.server`), `bu fota http://<pc-ip>:8000/zephyr.signed.bin`.
3. Prüfen: Progress 0→100 %, Reboot, **neue** FW läuft, `bu fotaconfirm` bzw.
   Auto-Confirm nach erstem Publish. Danach Rollback testen: eine absichtlich
   crashende FW pushen → MCUboot muss auf B zurückrollen.

### Für die Serie / Produktion (offen)
- **Eigener MCUboot-Signaturschlüssel** statt Nordic-Default (sonst kann jeder
  ein akzeptiertes Image signieren) — `SB_CONFIG_BOOT_SIGNATURE_KEY_FILE`, Key
  aus git raushalten + sichern (Details als Kommentar in `sysbuild.conf`).
- **MQTT-Kommando absichern**: der Trigger vertraut aktuell jeder Nachricht auf
  `projekt/cmd/...`. Broker-ACLs so setzen, dass nur der Backend-User publishen
  darf; optional die URL gegen einen erlaubten Host prüfen.
- **Alternative nRF Cloud FOTA** (managed Campaigns/Rollback-Reporting/Fleets):
  gleiches MCUboot-Fundament, dann `CONFIG_NRF_CLOUD_FOTA_POLL=y` + Board bei
  nRF Cloud claimen (aktuell noch der JWT-Blocker -77/-8) und `dfu_application.zip`
  im Portal hochladen. Nichts vom Fundament ist dann umsonst.

## Kritische Regeln (Modem-Sicherheit)
- **Chip bootet immer secured** → jedes JLink `connect` macht Mass-Erase der App
  → Flashen + Lesen IMMER in EINER Session. Modem-Credentials bleiben unberührt.
- **Kein Core-Halt während eines Credential-Writes** → bricked das Modem
  persistent; nur VSYS-Power-Cycle heilt es, kein JLink-Warmreset.
- **Kein eDRX/PSM/RAI** — bricht die Dauer-MQTT-Verbindung.
- RTT nur per gehaltener SWD-Session lesen (Live-Client = 0xFF); 4-KB-Buffer im
  DROP-Mode hält nur die ersten ~41 s.

## Device / Broker
- DEVICE_ID `P0.1.1`, Topic `projekt/data/P0.1.1`.
- Broker `9d3a3c61e89e4384ab8e326039d9f567.s1.eu.hivemq.cloud:8883` (CA sec tag 16842).

## Offen / zu verifizieren
- **Pt1000 IC201 IST bestückt** (bestätigt 2026-09-17: temp_in publisht ~24 °C,
  Raumtemperatur plausibel). Bei anfassen/erwärmen → `temp_in` steigt (pos. TK).
- **temp_in unkalibriert** (CVD, kein Bad-Abgleich) — der ratiometrische Aufbau
  kürzt ADC-Ref/Gain raus, Restfehler ≈ Pt1000-Elementtoleranz (~±0,3 °C, Klasse
  B). 1-Punkt-Feldabgleich über `TEMP_IN_OFFSET_C` (temp.c) holt das meiste raus;
  echter 2-Punkt-Abgleich = Bad, Testplan S-1.
- **Falls „temp_in" eigentlich die Körper/Haut-Temp sein soll:** das ist COW =
  **IC200 = T1 = AIN0** (nicht IC201). Aktuell exakt nach Ansage IC201 = AIN1
  umgesetzt — bei Bedarf `TEMP_IN_IDX` in temp.c auf 0 ändern.
- Accel-Einheit ist m/s² (wie der bisherige Platzhalter acc_z:9.81). Falls das
  Dashboard g erwartet: G_TO_MS2 in main.c entfernen.
- GPS: nächste Woche mit PMIC/Akku Outdoor gegenprüfen (freier Himmel → Fix).
- Für Serie: `NRF_PROVISIONING_INTERVAL_S` auf 86400 (aktuell 600).
