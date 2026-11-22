/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Shared entry points for the CU-board bring-up tests. Each test is a plain,
 * blocking function that prints its result via the log/shell and returns 0 on
 * pass, negative on fail, so it can be driven both by the boot-time POST and by
 * an interactive "bu <test>" shell command.
 */
#ifndef CDC_INCO_BRINGUP_H_
#define CDC_INCO_BRINGUP_H_

#include "shipment.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/util.h>

/* HiveMQ broker TLS security tag (CA provisioned by credentials.c).
 * Matches the DK / production firmware (CONFIG_MQTT_HELPER_SEC_TAG).
 */
#define BROKER_SEC_TAG 16842

/* Application firmware version, reported in the telemetry JSON as "fw" so the
 * dashboard can show which unit runs which build and confirm a FOTA rollout.
 * BUMP THIS every time you build an image you intend to roll out.
 */
#define APP_FW_VERSION "0.1.0"

/* TLS security tag used for the FOTA HTTPS download. By default it reuses the
 * broker's CA (BROKER_SEC_TAG = ISRG Root X1 / Let's Encrypt), which is correct
 * when the firmware file is served from a host with a Let's Encrypt certificate
 * (e.g. Synology Web Station, or Hetzner with certbot).
 *
 * If the FOTA host presents a DIFFERENT chain — most notably a Cloudflare-fronted
 * URL through the cloudflared tunnel, where Cloudflare's edge terminates TLS —
 * point this at a dedicated tag and provision that host's root CA there:
 *   1) find the root:  openssl s_client -connect fw.cadenced.at:443 -showcerts
 *   2) put its PEM in  src/credentials/fota-ca.pem  and enable the block in
 *      credentials.c (FOTA_DEDICATED_CA) so it is written to FOTA_SEC_TAG offline
 *   3) set this to that tag, e.g. 16843.
 * Keeping it separate from BROKER_SEC_TAG means swapping the FOTA host's CA never
 * touches the working HiveMQ CA.
 */
#ifndef FOTA_SEC_TAG
#define FOTA_SEC_TAG BROKER_SEC_TAG
#endif

/* ---- I2C bus / BMI323 IMU (P0.30 SDA, P0.31 SCL, addr 0x68) ------------- */
int bringup_i2c_scan(void);       /* scan the bus, list responders          */
int bringup_imu_id(void);         /* probe BMI323 CHIP_ID (expect 0x43)      */
int bringup_imu_init(void);       /* soft-reset + normal mode, +/-4 g, 100 Hz*/
int bringup_imu_read(void);       /* one accel sample -> g on x/y/z          */
int bringup_imu_int_test(void);   /* DRDY on INT1 (P0.05): count edges/1 s   */
/* One quiet accel sample in g (no logging), for the telemetry averaging loop.
 * Any out pointer may be NULL. Returns 0 on success, negative on I2C error.
 */
int bringup_imu_read_accel(float *gx, float *gy, float *gz);
/* 100 Hz, +/-16 g accel FIFO for the shipment events (shipment.c). */
int  bringup_imu_fifo_start(void);
bool bringup_imu_fifo_on(void);
int  bringup_imu_fifo_read(float *xyz, int max_frames); /* frames (x,y,z g) or <0 */

/* ---- 3x Pt1000 temperature via SAADC (drive P0.07, read AIN0..3) -------- */
int bringup_temp_read(void);      /* one ratiometric read of T1/T2/T3        */
/* High-resolution ratiometric read of temp_in only (IC201 = T2 = AIN1), vs the
 * V_TEMP excitation sense (AIN3). 256x SAADC oversampling + offset calibration,
 * excitation pulsed just for the read. Returns true and *temp_c on success,
 * false if the excitation/divider is out of range (e.g. Pt1000 not fitted).
 */
bool bringup_temp_in_read(double *temp_c);

/* Pt1000 channel indices (schematic net names, Sensors sheet). */
enum pt_chan {
	PT_SKIN = 0, /* IC200 / T1 / AIN0 — COW (skin side)      */
	PT_AMB  = 1, /* IC201 / T2 / AIN1 — AMBIENT (shade)      */
	PT_SUN  = 2, /* IC202 / T3 / AIN2 — AMBIENT_SUN (plate)  */
	PT_COUNT = 3,
};
/* Versor: product temperature = T1/AIN0 (metal-coupled side, faces the goods). */
#define PT_PROD PT_SKIN
/* High-res ratiometric read of all three Pt1000 under ONE excitation pulse
 * (same method as bringup_temp_in_read). valid[i] is false for an open /
 * unfitted / out-of-band channel; temp_c[i] is then untouched.
 */
void bringup_temp_read_all(double temp_c[PT_COUNT], bool valid[PT_COUNT]);

/* ---- Cellular: LTE attach + modem info + broker reachability ------------ */
int bringup_modem_init(void);     /* init nrf_modem_lib + modem_info (once)  */
extern int bringup_modem_init_raw; /* raw nrf_modem_lib_init() return code   */
int bringup_cell_info(void);      /* IMEI, modem temp/voltage (no LTE needed)*/
int bringup_cell_attach(void);    /* attach LTE-M, print operator + RSRP     */
int bringup_cell_reach(void);     /* DNS + TCP-connect to the HiveMQ broker  */
int bringup_cell_lte_off(void);   /* CFUN=20: LTE off, GNSS keeps the antenna */

/* Serving-cell snapshot for the telemetry (AT%XMONITOR + AT+CESQ). */
struct cell_status {
	bool valid;       /* registered and the values below are filled */
	int16_t rsrp;     /* dBm                                        */
	float rsrq;       /* dB  (NAN if unknown)                       */
	float snr;        /* dB  (NAN if unknown)                       */
	uint32_t cell_id; /* E-UTRAN cell id                            */
	uint32_t tac;     /* tracking area code                         */
	uint16_t band;
	char plmn[8];     /* MCC+MNC                                    */
};
/* Read the serving-cell values now. Works in RRC-connected and idle; in PSM the
 * modem returns the last values it measured. Returns 0 if registered.
 */
int bringup_cell_monitor(struct cell_status *cs);
/* Request (enable=true) or cancel LTE Power Saving Mode. rptau_s = requested
 * periodic TAU, rat_s = requested active time. Only re-sent when changed.
 */
int bringup_cell_psm(bool enable, int rptau_s, int rat_s);
/* True if registered (home or roaming) — also true while in PSM. */
bool bringup_cell_registered(void);
/* After a failed attach: leave the radio on so the modem searches on its own. */
void bringup_cell_keep_searching(void);
/* IMEI as a string ("" if the modem could not be read). */
const char *bringup_cell_imei(void);

/* ---- Onboarding: nRF Cloud attestation token (read-only, no key write) --- */
void bringup_print_attest_token(void); /* log ATTESTTOKEN to claim on nrfcloud.com */

/* ---- Onboarding: nRF Cloud device provisioning ("check-in") ------------- */
/* Trigger a one-shot provisioning check with nRF Cloud. Call ONCE the MQTT
 * session is up (so the HiveMQ CA write is done and datetime is valid). If the
 * board is claimed, the service installs nRF Cloud credentials (sec tag
 * 16842753) — the client takes LTE offline for the write, then reboots. If not
 * claimed, it logs the attestation token again. Requires nrf_provisioning built.
 */
void bringup_onboard_begin(void);
/* True while provisioning holds the modem offline for a credential write, so
 * the telemetry loop backs off instead of fighting it for LTE.
 */
bool bringup_onboard_active(void);

/* ---- TLS/MQTT publish (full data path to the NAS/website) --------------- */
int bringup_credentials_provision(void); /* raw CA write to BROKER_SEC_TAG (assumes offline modem) */
int bringup_provision_ca(void);   /* init + offline + write CA (safe, once)  */
int bringup_mqtt_pub(const char *payload); /* one-shot: provision+connect+publish+close; NULL=default JSON */

/* ---- Persistent MQTT session (DK-style: connect once, publish periodically) */
int  bringup_mqtt_session_start(void);            /* provision CA + attach + TLS connect + CONNACK */
int  bringup_mqtt_session_publish(const char *payload); /* publish one QoS1 payload, pump for PUBACK */
int  bringup_mqtt_session_publish_raw(const char *payload); /* same on versor/evt/<id> */
int  bringup_mqtt_session_poll(int timeout_ms);   /* pump input/keepalive while idle */
void bringup_mqtt_session_stop(void);             /* graceful MQTT disconnect         */

/* True while the session is connected. */
bool bringup_mqtt_session_is_up(void);

/* ---- Cellular-only diagnostic (SIM/reg/signal), stays awake for RTT ------ */
void bringup_modem_diag(void);    /* one-shot cell diag, then busy-hold       */

/* ---- GNSS --------------------------------------------------------------- */
int bringup_gnss_start(void);     /* enable GNSS + start acquisition         */
int bringup_gnss_stop(void);
int bringup_gnss_status(void);    /* print sat count / last fix              */

/* Release the GNSS acquisition thread. Called once the first ("online") MQTT
 * message is out, so GNSS gets priority AFTER LTE has done its initial work
 * (matches the DK order: 1) first message, 2) GPS with priority).
 */
void bringup_gnss_begin(void);

/* True once the one-shot A-GNSS assistance fetch has finished (success or error).
 * main() waits on this (bounded) before it takes LTE offline for the exclusive
 * GPS window, so the ephemeris download — which needs LTE — runs first.
 */
bool bringup_gnss_agnss_done(void);

/* Snapshot the latest GNSS state for the telemetry JSON (thread-safe). Any
 * out pointer may be NULL. Returns true if a valid fix has been seen.
 */
bool bringup_gnss_get(double *lat, double *lon, int *sats, bool *fix_valid,
		      float *accuracy, float *speed_kmh);

/* Extra GNSS quality values for the test protocol. cn0 = mean C/N0 of the 4
 * strongest satellites of the latest PVT that tracked >= 4 (dB-Hz, NAN if none
 * yet); ttff_s = time to fix of the latest acquisition (-1 if none yet);
 * fix_age_s = seconds since the last valid fix (-1 if never).
 */
void bringup_gnss_quality(float *cn0, int *ttff_s, int *fix_age_s);

/* Set the GNSS fix interval: 0 = GNSS off, 1 = continuous tracking, >= 10 =
 * periodic fix every N s (the receiver sleeps in between). Applied by the GNSS
 * thread; the first fix after start is always searched continuously.
 */
void bringup_gnss_set_interval(uint32_t interval_s);
/* 32-bit uptime (ms) of the GNSS thread's last loop pass, 0 before it started
 * — watchdog liveness check.
 */
uint32_t bringup_gnss_last_alive_ms(void);

/* ---- nPM1300 probe (Energy board, absent until PMIC arrives) ------------ */
int bringup_pmic_id(void);        /* probe 0x6B; used to detect PMIC present */

/* ---- FOTA (self-hosted HTTP(S) update via MCUboot dual-slot) ------------ */
/* Init the fota_download library + log the MCUboot swap/confirm state. Safe to
 * call once at boot; bringup_fota_start_url() also lazily inits.
 */
int  bringup_fota_init(void);
/* Start downloading "http(s)://host[:port]/path/app_update.bin" into the
 * secondary slot. https reuses the ISRG Root X1 CA on BROKER_SEC_TAG. Async:
 * progress/finish/error arrive as log lines + a queued status (see below).
 * Returns 0 if the download started, negative on bad URL / already running.
 */
int  bringup_fota_start_url(const char *url);
/* Mark the running image permanent (boot_write_img_confirmed) once it is proven
 * healthy, so MCUboot does not roll a freshly swapped FOTA image back. No-op if
 * already confirmed (e.g. a normal JLink flash). Call after a successful publish.
 */
int  bringup_fota_confirm(void);
/* True while a download is in flight (main() backs off / ignores re-triggers). */
bool bringup_fota_busy(void);
/* True once an image is staged and main() should publish a final line + reboot
 * so MCUboot can swap. Kept off the download thread on purpose.
 */
bool bringup_fota_reboot_requested(void);
/* Pop a queued FOTA status word (started/downloaded/error/...) for the main
 * thread to publish over MQTT. Returns true and fills out if one was queued.
 */
bool bringup_fota_dequeue_status(char *out, size_t n);

/* ---- Tiny flat-JSON helpers (no nesting / escapes; enough for commands) -- */
bool json_get_str(const char *json, const char *key, char *out, size_t n);
bool json_get_int(const char *json, const char *key, int32_t *out);

/* ---- Persistent device config + operating modes (devcfg.c) -------------- */
/*
 *  normal : energy saving. Every interval_s: connect, send, check for
 *           commands (~4 s), disconnect, LTE PSM until the next cycle.
 *           Commands therefore arrive with at most interval_s delay.
 *  live   : always-on MQTT (commands instantly), for bench tests (A-5, C-1).
 *  drain  : burn the battery as fast as possible (D-4): always-on MQTT,
 *           short interval, GNSS continuous, all 3 LEDs on. Runs until the
 *           battery is empty and stays selected across resets.
 */
enum app_mode {
	APP_MODE_NORMAL = 0,
	APP_MODE_LIVE   = 1,
	APP_MODE_DRAIN  = 2,
};

struct app_cfg {
	enum app_mode mode;
	uint32_t interval_s; /* telemetry period                          */
	uint32_t gnss_s;     /* 0 = off, 1 = continuous, >= 10 = periodic  */
	uint8_t led;         /* requested LED (normal / live): colour      *
			      * 0 off, 1 green, 2 blue, 3 red, | LED_CFG_BLINK */
};

int  devcfg_init(void);                 /* load settings, count the boot       */
void devcfg_get(struct app_cfg *out);
/* Monotonic counter, bumped on every config change (main() watches it). */
uint32_t devcfg_generation(void);
/* Apply a config JSON {"mode":"normal","int":300,"gnss":300,"led":0}. Missing
 * keys keep their value, except that a mode change resets int/gnss to that
 * mode's defaults unless they are given too. Persists. True if changed.
 */
bool devcfg_apply_json(const char *json);
int  devcfg_set_mode(enum app_mode m);
const char *devcfg_mode_name(enum app_mode m);
bool devcfg_mode_from_name(const char *name, enum app_mode *out);

/* Device ID = serial number (e.g. "P0.1.1"). Sources, first match wins:
 *   1. override in settings: `bu id P0.1.2` (RTT) / {"cmd":"id",...} (MQTT)
 *   2. factory ID in the last flash page, written by the flash station together
 *      with the firmware (see FACTORY_ID_ADDR in devcfg.c)
 *   3. "IMEI<imei>" — only on lab boards flashed without the Cockpit; such IDs
 *      are never assigned to a customer.
 */
const char *devcfg_device_id(void);
int  devcfg_set_device_id(const char *id);
/* Publish ID + IMEI + FW in a RAM block the flash station reads over SWD
 * (halted core) right after flashing. Call once the modem is initialised.
 */
void devcfg_factory_info_update(void);

uint32_t devcfg_next_seq(void);         /* telemetry sequence, survives resets */
uint32_t devcfg_boot_count(void);
uint32_t devcfg_reset_cause(void);      /* hwinfo RESET_* bits of this boot    */
const char *devcfg_reset_name(void);    /* short name: por/pin/sw/wdt/bor/...  */

/* ---- nPM1300 PMIC: charger, fuel gauge, LEDs (pmic.c) ------------------- */
#define LED_RED   BIT(0) /* nPM LED0 -> LED501 */
#define LED_GREEN BIT(1) /* nPM LED1 -> LED500 */
#define LED_BLUE  BIT(2) /* nPM LED2 -> D500   */

/* app_cfg.led / cfg JSON "led": colour the backend asked for (1 = green is
 * also the old "on", so older backends keep working).
 */
#define LED_CFG_OFF   0
#define LED_CFG_GREEN 1
#define LED_CFG_BLUE  2
#define LED_CFG_RED   3
#define LED_CFG_COLOUR(v) ((v) & 0x03)
#define LED_CFG_BLINK 0x04   /* flag: the device blinks the colour itself */
#define LED_CFG_MAX   (LED_CFG_RED | LED_CFG_BLINK)
/* Telemetry "led": LED_RED|LED_GREEN|LED_BLUE as lit + this bit while blinking */
#define LED_TEL_BLINK BIT(3)

struct pmic_status {
	bool present;      /* PMIC answered; all values below valid          */
	float vbat;        /* V                                              */
	float ibat_ma;     /* mA, + = charging, - = discharging              */
	float t_bat;       /* C, battery NTC                                 */
	float t_die;       /* C, nPM1300 die temperature                     */
	float soc;         /* %, 0..100                                      */
	int chg;           /* 0 = not charging, 1 = charging, 2 = full        */
	uint8_t chg_raw;   /* BCHGCHARGESTATUS                               */
	uint8_t chg_err;   /* BCHGERRREASON                                  */
	bool vbus;         /* Qi / USB supply present                        */
	bool reduced;      /* firmware reduced the charge current (warm)     */
};

int  pmic_init(void);                   /* 0 if the PMIC is present            */
bool pmic_present(void);
/* Measure, run the charge-temperature supervision and the SoC estimate. Call
 * regularly (every few seconds); cheap. s may be NULL.
 */
int  pmic_update(struct pmic_status *s);
void pmic_get(struct pmic_status *s);   /* last values, no I2C traffic         */
void pmic_leds(uint8_t mask);           /* LED_RED | LED_GREEN | LED_BLUE      */
uint8_t pmic_leds_get(void);
int  pmic_ship_mode(void);              /* power off; only Qi/VBUS wakes it    */

/* ---- Telemetry records + RAM backlog (telemetry.c) ---------------------- */
struct tel_rec {
	int64_t ts_ms;        /* unix ms, 0 = network time not known yet  */
	uint32_t seq;
	uint32_t uptime_s;
	uint8_t mode;
	uint32_t interval_s;
	/* GNSS */
	double lat, lon;
	int16_t sats;
	bool fix;
	float hacc, speed, cn0;
	int32_t ttff_s, fix_age_s;
	/* IMU, mean over the window, m/s^2; act = mean |a - mean| (activity) */
	float acc_x, acc_y, acc_z, act;
	/* Versor shipment window (shipment.c); shp.valid false -> not sent */
	struct shp_win shp;
	bool trip, trip_ok;   /* trip running / no violation so far        */
	uint32_t trip_s;      /* trip duration s                           */
	/* temperatures, C (NAN = invalid): t_in = product (PT_PROD), t_amb =
	 * T2, t_aux = T3; window min/max of t_in are in shp.t_min/t_max.
	 */
	float t_in, t_amb, t_aux, t_bat, t_pmic;
	/* power */
	struct pmic_status pm;
	/* cellular */
	struct cell_status cell;
	uint8_t leds;
};

/* Push a finished record into the backlog (oldest dropped when full). */
void telemetry_push(const struct tel_rec *r);
/* Number of records waiting to be sent. */
int  telemetry_pending(void);
/* Peek at the oldest / newest pending record (NULL if empty). */
struct tel_rec *telemetry_oldest(void);
struct tel_rec *telemetry_newest(void);
void telemetry_drop_oldest(void);
/* Render a record as the telemetry JSON. Returns length or negative. */
int  telemetry_format(const struct tel_rec *r, char *buf, size_t n);
/* JSON of the newest record (or a minimal one) for `bu pub` / NULL publish. */
int  telemetry_format_latest(char *buf, size_t n);

/* Versor glue (main.c, thread-safe): shipment thresholds from a cfg/cmd JSON
 * (true if changed) and trip control 0 = stop, 1 = start, 2 = report.
 */
bool app_shipment_cfg(const char *json);
void app_trip_cmd(int op);

/* Main-thread reboot request (MQTT "reboot"/"id" commands). */
void app_request_reboot(void);
/* Feed the watchdog from long but bounded waits (MQTT socket pump). Skipped if
 * a monitored thread has stalled, so a hang still ends in a watchdog reset.
 */
void app_watchdog_kick(void);

#endif /* CDC_INCO_BRINGUP_H_ */
