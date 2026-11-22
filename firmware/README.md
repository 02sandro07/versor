# CDC-INCO-Bringup-R1A — Inbetriebnahme- und Testfirmware (R1A)

> **Stand 0.2.0:** Testfirmware nach Testplan Kap. 3 — alle Messfelder, Modi
> `normal` / `live` / `drain`, nPM1300 (Laden mit Temperaturfenster, Akku, LEDs),
> RAM-Puffer, Watchdog. Flashen + Geräte-ID über das Cockpit (Lager → „Gerät
> flashen“). Details: [HANDOFF.md](HANDOFF.md).

Eigenständige Testfirmware für die **CU-Platine** von `CDC-INCO-Prototyp-R1A`
(nRF9151). Prüft jedes Subsystem einzeln bei der Erstinbetriebnahme:

- **I2C-Bus** (P0.30 SDA / P0.31 SCL) — Scan
- **IMU BMI323** @0x68 — CHIP_ID (0x43), ±4 g / 100 Hz, Achsen, INT1 (P0.05)
- **3× Pt1000** über SAADC — P0.07 treibt, AIN0–3 (P0.13–0.16) misst, ratiometrisch
- **LTE-M** — Attach, IMEI, RSRP, Modem-Temp/Spannung; TCP-Reach zum HiveMQ-Broker
- **GNSS** — Aquise + Fix (braucht GPS-Antenne + VDD_GNSS + freien Himmel)
- **nPM1300**-Probe @0x6B (erkennt, wann die Energy-Platine/PMIC dran ist)

Konsole **nur über SEGGER RTT** (die CU-Platine hat kein USB/UART-RX). Beim Boot
läuft ein POST + danach ein Monitor-Loop (IMU/Pt1000 alle 5 s). Interaktiv:
`bu status | mode | int | id | i2cscan | imu | imuint | temp | pmic | cellinfo | cell | reach | gnss | gnssstat | ship | reboot`.

Der PMIC fehlt noch → Versorgung extern einspeisen (VSYS 3.7 V + VDD_1V8 1.8 V
[+ VDD_GNSS 1.8 V] an Stecker **P400**). Geflasht wird über den On-Board-J-Link
des nRF9151-DK (DEBUG OUT → J600).

**Vollständige Schritt-für-Schritt-Anleitung** (Verkabelung, Flashen, Tests,
Troubleshooting): siehe `IONOS HiDrive/public/03_Embedded/CDC-INCO-Prototyp-R1A_Inbetriebnahme.md`
(bewusst **nicht** im Git). Der maßgebliche Hardware-Testplan ist
`02_Hardware/CDC-INCO-Testplan-R1A.md` (v2.0).

## Bauen & Flashen (Kurz)

```bash
export PATH=/opt/nordic/ncs/toolchains/322ac893fe/bin:$PATH
export ZEPHYR_BASE=/opt/nordic/ncs/v3.2.1/zephyr
export ZEPHYR_SDK_INSTALL_DIR=/opt/nordic/ncs/toolchains/322ac893fe/opt/zephyr-sdk
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
west build -b nrf9151dk/nrf9151/ns --sysbuild -p always
west flash --runner jlink   # On-Board-J-Link des DK, via DEBUG OUT auf die CU-Platine
```

Pin-Map in `boards/nrf9151dk_nrf9151_ns.overlay` (spiegelt die CU-Platine, nicht
das DK). Baut auf das DK-Target, weil der nRF9151-SiP identisch ist. Die
Pinbelegung stammt aus der ODB++-Netzliste (Testplan v2.0) und der finalen
`shematic-R1A-ordered.pdf`.
