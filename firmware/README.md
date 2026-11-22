# Versor-Firmware: Sendungslogger auf Pecura-Hardware R1A

Version **0.1.0** (2026-11-22). Basis: Pecura-Bring-up-Firmware 0.4.0 (pecura `cb42fd2`).
Die Pecura-Historie steht in [HANDOFF-pecura.md](HANDOFF-pecura.md).

Gleiche Hardware wie Pecura: nRF9151 mit LTE-M und GNSS, BMI323, 3× Pt1000 und nPM1300.
Statt der Kuh-Bewegungsauswertung (`motion.c`) erkennt `shipment.c` Ereignisse am Paket.
Daraus entsteht je Fahrt ein Bericht, die Bescheinigung.

## Bauen und Flashen

```bash
export PATH=/opt/nordic/ncs/toolchains/322ac893fe/bin:$PATH
export ZEPHYR_BASE=/opt/nordic/ncs/v3.2.1/zephyr
export ZEPHYR_SDK_INSTALL_DIR=/opt/nordic/ncs/toolchains/322ac893fe/opt/zephyr-sdk
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
cp src/app_credentials.h.example src/app_credentials.h   # echte HiveMQ-Daten eintragen (gitignored)
west build -b nrf9151dk/nrf9151/ns --sysbuild -p always -d build_dk
west flash -d build_dk --runner jlink    # DK direkt oder über DEBUG OUT → J600 auf die R1A-CU
```

- **Ein Image für DK und R1A.** Die Pin-Map in `boards/` spiegelt die CU-Platine.
- **Auf dem DK** gibt es kein IMU, keinen Pt1000 und keinen PMIC. Dann ist `shp.valid` = false, die Temperaturen sind `null`, LTE, MQTT und GNSS laufen normal.
- **FOTA:** `build_dk/firmware/zephyr/zephyr.signed.bin` (389 KB, Slot 448 KB). Es ist mit dem NCS-Standardschlüssel signiert, wie Pecura. Ein Pecura-Gerät nimmt das Image also per FOTA an.
- **Host-Test** ohne Hardware:
  ```bash
  cc -std=c11 -O2 -Wall -Wextra -Isrc tools/shipment_test.c src/shipment.c -lm -o build_host/shipment_test && build_host/shipment_test
  ```
  Ergebnis: 46/46 Prüfungen bestanden.

## Ereignisse (`src/shipment.c`)

BMI323-FIFO mit 100 Hz und ±16 g. Die Produkttemperatur kommt von Pt1000 **T1/AIN0**.

| Ereignis | Bedingung | Wert |
|---|---|---|
| `shock` | \|a\| > `shock_mg`; Prellen < 50 ms wird zusammengefasst | Spitze g (16 = Endanschlag) |
| `fall` | \|a\| < 0,35 g für ≥ `fall_ms` | Dauer ms (150 ms ≈ 11 cm, 300 ms ≈ 44 cm) |
| `tilt` / `flip` | Lage in Ruhe > `tilt` bzw. > 135° gegen die Referenz | Winkel ° |
| `t_hi` / `t_lo` / `t_ok` | Produkttemperatur verlässt bzw. erreicht wieder [`t_lo`, `t_hi`], nur bei `t_lim` = 1 | °C |

Die Referenz für „diese Seite oben“ ist die erste ruhige Lage nach dem Boot bzw. nach `trip start`.
Bei einem Ereignis und `alarm` = 1 sendet das Gerät frühestens 60 s nach Fensterbeginn.

## MQTT

Topics `versor/data|evt|cmd|cfg/<id>`, Client-ID `<id>-vsr-<rand>`. Broker und User sind dieselben wie bei Pecura (?).

**Konfiguration** über cfg-Topic (retained) oder `{"cmd":"cfg",…}`. Die Werte sind Ganzzahlen, die Grenzen werden geklemmt:

| Key | Default | Bereich |
|---|---|---|
| `shock_mg` | 3000 | 1000–15000 |
| `fall_ms` | 150 | 50–2000 |
| `tilt` | 45 | 10–170 |
| `t_lim` / `t_lo` / `t_hi` | 0 / 2 / 8 | Grenzen aus / °C |
| `alarm` | 1 | 0/1 |

Dazu kommen wie bisher `int`, `mode` usw. aus devcfg.

**Fahrt:** `{"cmd":"trip","a":"start|stop|get"}`. Der Bericht geht als JSON auf `versor/evt/<id>`. Im RTT-Shell: `bu trip [start|stop]`.
Eine laufende Fahrt liegt in NVS (`vsr/trip`). Gespeichert wird bei Start und Stopp und sonst höchstens alle 10 min.
Nach einem Reset läuft sie weiter, `resets` wird hochgezählt.

**Datensatz `versor/data`** enthält zusätzlich zu den Pecura-Feldern (Zelle, GNSS, Akku …):
- nur mit IMU: `shk`, `shk_max`, `fall`, `fall_ms`, `tilt`, `tilt_max`, `orient` (0 aufrecht, 1 gekippt, 2 kopfüber), `ev:[{t,u,e,v}]` (max. 8), `ev_lost`
- immer: `trip`, `trip_s`, `trip_ok`, `t_in`, `t_in_min`, `t_in_max`, `t_out_s`, `t_st`, `t_amb` (T2), `t_aux` (T3), `t_bat`, `t_pmic`

## Grenzen (bekannt, offen)

- **Stöße während des Sendens gehen verloren.** Der FIFO puffert nur ≈ 3,4 s, ein LTE-Sendevorgang dauert länger.
- **Schwellen werden nicht im Gerät gespeichert.** Bis das retained cfg-Topic nach dem Connect ankommt, gelten die Defaults.
- **Nicht auf Hardware getestet.** Getestet ist nur der Build und der Host-Test von `shipment.c`.
- **Keine Feuchte.** Die R1A hat keinen Feuchtesensor.
