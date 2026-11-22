/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Versor shipment logger on the Pecura hardware (R1A: nRF9151, BMI323,
 * 3x Pt1000, nPM1300) — based on the Pecura test firmware 0.4.0.
 *
 * The BMI323 runs at 100 Hz / +-16 g into its FIFO; shipment.c turns the
 * samples into shock, fall, tilt and flip events, the product temperature
 * (T1/AIN0) into t_hi/t_lo/t_ok events, and sums everything per trip
 * (`bu trip start|stop|show`, or {"cmd":"trip","a":"start"}). The trip summary
 * is kept in flash (settings "vsr/trip") and survives resets.
 *
 * Modes (devcfg.c):
 *   normal  energy saving: sample for interval_s (default 300 s), then connect,
 *           send the record (+ backlog), listen ~4 s for commands/config,
 *           disconnect (PSM). An event (shock, fall, flip, temperature limit)
 *           closes the window early (at most once per ALARM_MIN_WINDOW_MS).
 *   live    always-on MQTT, 30 s.
 *   drain   empty the battery fast: always-on MQTT, 10 s, GNSS continuous.
 *
 * Topics: versor/data|evt|cmd|cfg/<id> (mqtt.c). Thresholds are cfg keys
 * (shock_mg, fall_ms, tilt, t_lim, t_lo, t_hi, alarm — shipment.h).
 *
 * A hardware watchdog resets a hung firmware (reported as rst:"wdt"); a new
 * FOTA image that hangs before its first successful publish is rolled back.
 *
 * Everything else stays available on the `bu` RTT shell (see bottom).
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/watchdog.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#if defined(CONFIG_DATE_TIME)
#include <date_time.h>
#endif

#include "bringup.h"
#include "shipment.h"
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Fast-loop slice: accel FIFO drain + MQTT poll (or sleep) per slice. The
 * BMI323 samples at 100 Hz into its FIFO (~10 frames per slice, room for
 * ~3.4 s); each slice feeds shipment.c. Without FIFO one sample per slice
 * still gives the window mean and "act", but no events.
 */
#define SAMPLE_SLICE_MS 100
#define FIFO_FRAMES_MAX 128

/* An event closes the window early, but not before this (bounds airtime). */
#define ALARM_MIN_WINDOW_MS (60 * 1000)

/* Store the running trip at least this often (counts, duration). */
#define TRIP_SAVE_MS (10 * 60 * 1000)

/* Pt1000 read period. Each read pulses the excitation for ~45 ms (3 nodes + top,
 * 256x oversampled), so 500 ms = ~9 % duty (no measurable self-heating) and
 * 2 s in "normal" to save energy.
 */
#define TEMP_PERIOD_FAST_MS 500
#define TEMP_PERIOD_SLOW_MS 2000

/* PMIC poll period (charge supervision reacts within this time). */
#define PMIC_PERIOD_MS 5000

/* "normal" mode: time to stay connected after sending, so the retained config
 * and any command the backend holds for us (it answers our telemetry) arrive.
 */
#define DOWNLINK_WAIT_MS 4000

/* At most this many backlog records per connection (bounds the airtime after a
 * long dead spot; the rest follows in the next cycles).
 */
#define BACKLOG_PER_CYCLE 24

/* PSM request in "normal" mode: periodic TAU well above the cycle, short active
 * time. The device wakes itself for each cycle (mobile-originated data needs no
 * paging), so the TAU only bounds how often the network expects a sign of life.
 */
#define PSM_RPTAU_MIN_S 3600
#define PSM_RAT_S 4

/* After a failed LTE attach, wait this long before blocking on a new attempt
 * (unless the modem registers on its own earlier).
 */
#define ATTACH_RETRY_MS (10 * 60 * 1000)

/* Watchdog: 5 min. It is fed every sample slice, before/after each send step
 * and from the MQTT socket pump (all of those are deadline-bounded); the
 * longest single block is the LTE attach (LTE_NETWORK_TIMEOUT = 180 s).
 * Feeding stops when the GNSS thread has not passed its loop for
 * THREAD_STALL_MS, so a hung worker thread also ends in a reset.
 */
#define WDT_TIMEOUT_MS  (5U * 60U * 1000U)
#define THREAD_STALL_MS (2U * 60U * 1000U)

/* BMI323 reports acceleration in g; the telemetry JSON carries m/s^2. */
#define G_TO_MS2 9.80665f

/* Serialises sensor-bus access between the main loop and shell commands. */
static K_MUTEX_DEFINE(bus_lock);

#define LOCK()   k_mutex_lock(&bus_lock, K_FOREVER)
#define UNLOCK() k_mutex_unlock(&bus_lock)

static atomic_t reboot_req;

void app_request_reboot(void)
{
	atomic_set(&reboot_req, 1);
}

/* ------------------------------ shipment glue ---------------------------- */

/* shipment.c is plain C: one lock for the main loop, MQTT handler, shell. */
static K_MUTEX_DEFINE(shp_lock);
#define SHP_LOCK()   k_mutex_lock(&shp_lock, K_FOREVER)
#define SHP_UNLOCK() k_mutex_unlock(&shp_lock)

static atomic_t trip_req;      /* bit0 start, bit1 stop, bit2 report */
static atomic_t trip_report;   /* send a trip report on the evt topic */
static struct shp_trip trip_stored;
static bool trip_loaded;
static int64_t trip_saved_ms;

static int64_t unix_now_s(void)
{
#if defined(CONFIG_DATE_TIME)
	int64_t ms;

	if (date_time_now(&ms) == 0) {
		return ms / 1000;
	}
#endif
	return 0;
}

static int vsr_set(const char *name, size_t len, settings_read_cb read_cb,
		   void *cb_arg)
{
	const char *next;

	if (settings_name_steq(name, "trip", &next) && !next &&
	    len == sizeof(trip_stored)) {
		if (read_cb(cb_arg, &trip_stored, sizeof(trip_stored)) ==
		    sizeof(trip_stored)) {
			trip_loaded = true;
		}
	}
	return 0;
}
SETTINGS_STATIC_HANDLER_DEFINE(vsr, "vsr", NULL, vsr_set, NULL, NULL);

/* Write the trip to flash if it changed (or periodically while it runs). */
static void trip_persist(bool force)
{
	struct shp_trip t;

	SHP_LOCK();
	bool dirty = shipment_trip_dirty();

	shipment_trip_get(&t);
	SHP_UNLOCK();
	if (!force && !dirty &&
	    !(t.active && k_uptime_get() - trip_saved_ms >= TRIP_SAVE_MS)) {
		return;
	}
	int err = settings_save_one("vsr/trip", &t, sizeof(t));

	trip_saved_ms = k_uptime_get();
	if (err) {
		LOG_WRN("Trip not stored (%d)", err);
	}
}

bool app_shipment_cfg(const char *json)
{
	SHP_LOCK();
	bool changed = shipment_apply_json(json);
	SHP_UNLOCK();

	if (changed) {
		struct shp_params p;

		shipment_get_params(&p);
		LOG_INF("Shipment cfg: shock %d mg, fall %d ms, tilt %d deg, "
			"t_lim %d [%d..%d C], alarm %d", p.shock_mg, p.fall_ms,
			p.tilt_deg, p.t_lim, p.t_lo, p.t_hi, p.alarm);
	}
	return changed;
}

/* Callable from any thread; carried out by the main loop (flash, MQTT). */
void app_trip_cmd(int op)
{
	atomic_or(&trip_req, op == 1 ? 1 : op == 0 ? 2 : 4);
}

static void trip_requests(void)
{
	atomic_val_t r = atomic_set(&trip_req, 0);

	if (!r) {
		return;
	}
	SHP_LOCK();
	if (r & 2) {
		shipment_trip_stop();
	}
	if (r & 1) {
		shipment_trip_start();
	}
	SHP_UNLOCK();
	if (r & 3) {
		LOG_INF("Trip %s", (r & 1) ? "started" : "stopped");
		trip_persist(true);
	}
	atomic_set(&trip_report, 1);
}

static int trip_format(char *buf, size_t n)
{
	struct shp_trip t;

	SHP_LOCK();
	shipment_trip_get(&t);
	SHP_UNLOCK();
	return snprintk(buf, n,
		"{\"device_id\":\"%s\",\"trip\":\"%s\",\"ok\":%s,"
		"\"start\":%lld,\"stop\":%lld,\"dur_s\":%u,"
		"\"shk\":%u,\"shk_max\":%.1f,\"fall\":%u,\"fall_ms\":%u,"
		"\"tilt\":%u,\"flip\":%u,\"tilt_max\":%.0f,"
		"\"t_viol\":%u,\"t_out_s\":%u,\"t_min\":%.2f,\"t_max\":%.2f,"
		"\"resets\":%u}",
		devcfg_device_id(), t.active ? "running" : (t.start_up || t.start_ts ? "stopped" : "none"),
		shipment_trip_ok(&t) ? "true" : "false",
		(long long)t.start_ts, (long long)t.stop_ts, t.dur_s,
		t.shocks, (double)t.shock_max, t.falls, t.fall_max_ms,
		t.tilts, t.flips, isnan(t.tilt_max) ? -1.0 : (double)t.tilt_max,
		t.t_viol, t.t_out_s,
		isnan(t.t_min) ? -999.0 : (double)t.t_min,
		isnan(t.t_max) ? -999.0 : (double)t.t_max, t.resets);
}

/* ------------------------------ watchdog --------------------------------- */

static const struct device *const wdt = DEVICE_DT_GET_OR_NULL(DT_ALIAS(watchdog0));
static int wdt_channel = -1;

static void wdt_start(void)
{
	struct wdt_timeout_cfg cfg = {
		.window.min = 0,
		.window.max = WDT_TIMEOUT_MS,
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC,
	};

	if (!wdt || !device_is_ready(wdt)) {
		LOG_WRN("Watchdog not available");
		return;
	}
	wdt_channel = wdt_install_timeout(wdt, &cfg);
	if (wdt_channel < 0) {
		LOG_WRN("wdt_install_timeout: %d", wdt_channel);
		return;
	}
	/* Pause while the debugger halts the core (RTT readout procedure). */
	if (wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG) != 0) {
		LOG_WRN("wdt_setup failed");
		wdt_channel = -1;
		return;
	}
	LOG_INF("Watchdog armed (%u min)", WDT_TIMEOUT_MS / 60000U);
}

static void wdt_kick(void)
{
	static bool stall_logged;

	if (wdt_channel < 0) {
		return;
	}
	uint32_t gnss_alive = bringup_gnss_last_alive_ms();

	/* 0 = GNSS thread not started yet (waits for the first connect). */
	if (gnss_alive && k_uptime_get_32() - gnss_alive > THREAD_STALL_MS) {
		if (!stall_logged) {
			LOG_ERR("GNSS thread stalled — letting the watchdog reset");
			stall_logged = true;
		}
		return;
	}
	(void)wdt_feed(wdt, wdt_channel);
}

void app_watchdog_kick(void)
{
	wdt_kick();
}

/* ------------------------------ helpers ---------------------------------- */

static void run_post(void)
{
	LOG_INF("===== POST: power-on self-test =====");

	LOCK();
	bringup_i2c_scan();
	bringup_pmic_id();
	bringup_imu_init();
	bringup_imu_read();
	bringup_imu_int_test();
	bringup_temp_read();
	UNLOCK();

	bringup_cell_info();

	LOG_INF("===== POST done — see results above =====");
}

/* LEDs: drain = all on; otherwise the requested colour (green / blue / red,
 * e.g. "find these cows"), steady or blinking, plus blue (steady) while
 * charging and red on a charger error.
 *
 * Blinking runs on the system work queue, independent of the main loop, so it
 * keeps its rhythm while the main thread is busy attaching/sending (normal
 * mode). The nPM1300 LED driver is a single I2C register write per change;
 * led_lock serialises the main thread and the blink work.
 */
#define LED_BLINK_ON_MS  300
#define LED_BLINK_OFF_MS 700

static K_MUTEX_DEFINE(led_lock);
static uint8_t led_steady;  /* always lit (charging, error, steady request) */
static uint8_t led_blink;   /* blinked on top (0 = no blinking)            */
static bool led_phase;      /* blink currently lit                          */
static void led_blink_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(led_blink_work, led_blink_fn);

static void led_blink_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	k_mutex_lock(&led_lock, K_FOREVER);
	if (led_blink) {
		led_phase = !led_phase;
		pmic_leds(led_steady | (led_phase ? led_blink : 0));
		k_work_reschedule(&led_blink_work,
				  K_MSEC(led_phase ? LED_BLINK_ON_MS : LED_BLINK_OFF_MS));
	}
	k_mutex_unlock(&led_lock);
}

/* What the telemetry reports: lit colours (blinking ones as if steady) plus
 * LED_TEL_BLINK — the instantaneous blink phase would be meaningless.
 */
static uint8_t leds_report(void)
{
	uint8_t m;

	k_mutex_lock(&led_lock, K_FOREVER);
	m = led_steady | led_blink | (led_blink ? LED_TEL_BLINK : 0);
	k_mutex_unlock(&led_lock);
	return m;
}

static void leds_apply(const struct app_cfg *cfg)
{
	static const uint8_t colour_mask[] = {
		[LED_CFG_OFF] = 0, [LED_CFG_GREEN] = LED_GREEN,
		[LED_CFG_BLUE] = LED_BLUE, [LED_CFG_RED] = LED_RED,
	};
	struct pmic_status pm;
	uint8_t steady = 0, blink = 0;

	pmic_get(&pm);
	if (cfg->mode == APP_MODE_DRAIN) {
		steady = LED_RED | LED_GREEN | LED_BLUE;
	} else {
		uint8_t want = colour_mask[LED_CFG_COLOUR(cfg->led)];

		if (cfg->led & LED_CFG_BLINK) {
			blink = want;
		} else {
			steady = want;
		}
		if (pm.present && pm.chg == 1) {
			steady |= LED_BLUE;
		}
		if (pm.present && pm.chg_err) {
			steady |= LED_RED;
		}
		blink &= ~steady; /* a colour that is lit anyway does not blink */
	}

	k_mutex_lock(&led_lock, K_FOREVER);
	bool start = blink && !led_blink;

	led_steady = steady;
	led_blink = blink;
	if (!blink) {
		led_phase = false;
		(void)k_work_cancel_delayable(&led_blink_work);
		if (steady != pmic_leds_get()) {
			pmic_leds(steady);
		}
	} else if (start) {
		led_phase = false; /* the work flips it -> starts lit */
		k_work_reschedule(&led_blink_work, K_NO_WAIT);
	} else {
		/* already blinking: just refresh the steady part right now */
		pmic_leds(steady | (led_phase ? blink : 0));
	}
	k_mutex_unlock(&led_lock);
}

/* Push the mode's radio settings: GNSS interval and PSM. */
static void apply_mode(const struct app_cfg *cfg)
{
	bringup_gnss_set_interval(cfg->gnss_s);

	if (cfg->mode == APP_MODE_NORMAL) {
		int tau = MAX(PSM_RPTAU_MIN_S, (int)cfg->interval_s * 2);

		(void)bringup_cell_psm(true, tau, PSM_RAT_S);
	} else {
		(void)bringup_cell_psm(false, 0, 0);
	}
	leds_apply(cfg);

	LOG_INF("Mode %s: interval %u s, GNSS %s%u s, PSM %s",
		devcfg_mode_name(cfg->mode), cfg->interval_s,
		cfg->gnss_s == 1 ? "continuous/" : "", cfg->gnss_s,
		cfg->mode == APP_MODE_NORMAL ? "on" : "off");
}

/* ------------------------------ telemetry window ------------------------- */

/* Sample for cfg->interval_s (or until the config changes / a reboot is
 * requested), keep the MQTT link alive if it is up, and produce one record.
 */
static void telemetry_window(const struct app_cfg *cfg, struct tel_rec *rec)
{
	/* accel: mean per axis + Welford variance of |a| (activity) */
	double sx = 0, sy = 0, sz = 0;
	double m_mean = 0, m_m2 = 0;
	uint32_t n_acc = 0;
	double st[PT_COUNT] = { 0 };
	uint32_t nt[PT_COUNT] = { 0 };

	uint32_t gen0 = devcfg_generation();
	int temp_period = cfg->mode == APP_MODE_NORMAL ? TEMP_PERIOD_SLOW_MS :
							  TEMP_PERIOD_FAST_MS;
	int64_t t0 = k_uptime_get();
	int64_t t_end = t0 + (int64_t)cfg->interval_s * 1000;
	int64_t next_temp = t0;
	int64_t next_pmic = t0;
	int64_t last_temp = 0;
	static float fifo[FIFO_FRAMES_MAX * 3];

	LOCK();
	if (!bringup_imu_fifo_on()) {
		(void)bringup_imu_fifo_start();
	}
	UNLOCK();

	while (k_uptime_get() < t_end) {
		wdt_kick();

		int nf;

		LOCK();
		if (bringup_imu_fifo_on()) {
			nf = bringup_imu_fifo_read(fifo, FIFO_FRAMES_MAX);
		} else {
			nf = bringup_imu_read_accel(&fifo[0], &fifo[1], &fifo[2]) == 0 ? 1 : -1;
		}
		bool fed = bringup_imu_fifo_on();
		UNLOCK();

		SHP_LOCK();
		shipment_clock(unix_now_s(), (uint32_t)(k_uptime_get() / 1000));
		for (int f = 0; f < nf; f++) {
			float gx = fifo[3 * f], gy = fifo[3 * f + 1], gz = fifo[3 * f + 2];
			double mag = sqrt((double)(gx * gx + gy * gy + gz * gz));
			double d = mag - m_mean;

			n_acc++;
			sx += (double)gx;
			sy += (double)gy;
			sz += (double)gz;
			m_mean += d / n_acc;
			m_m2 += d * (mag - m_mean);
			/* Events need the steady 100 Hz FIFO stream, not single reads. */
			if (fed) {
				shipment_sample(gx, gy, gz);
			}
		}
		SHP_UNLOCK();

		if (k_uptime_get() >= next_temp) {
			double tc[PT_COUNT];
			bool ok[PT_COUNT];
			int64_t now = k_uptime_get();

			LOCK();
			bringup_temp_read_all(tc, ok);
			UNLOCK();
			for (int i = 0; i < PT_COUNT; i++) {
				if (ok[i]) {
					st[i] += tc[i];
					nt[i]++;
				}
			}
			SHP_LOCK();
			shipment_temp(ok[PT_PROD] ? (float)tc[PT_PROD] : NAN,
				      last_temp ? (uint32_t)(now - last_temp) : 0);
			SHP_UNLOCK();
			last_temp = now;
			next_temp += temp_period;
		}

		if (k_uptime_get() >= next_pmic) {
			(void)pmic_update(NULL);
			leds_apply(cfg);
			next_pmic += PMIC_PERIOD_MS;
		}

		/* Keep MQTT alive (live/drain) or just idle (normal: offline). */
		if (bringup_mqtt_session_poll(SAMPLE_SLICE_MS) == -ENOTCONN) {
			k_msleep(SAMPLE_SLICE_MS);
		}

		trip_requests();

		if (devcfg_generation() != gen0 || atomic_get(&reboot_req)) {
			LOG_INF("Config changed — closing the window early");
			break;
		}
		/* Shock / fall / flip / temperature limit: send now instead of at
		 * the end of the interval (at most once per ALARM_MIN_WINDOW_MS).
		 */
		SHP_LOCK();
		bool alarm = shipment_alarm();
		SHP_UNLOCK();
		if (alarm && k_uptime_get() - t0 >= ALARM_MIN_WINDOW_MS) {
			LOG_INF("Shipment event — closing the window early");
			break;
		}
		IF_ENABLED(CONFIG_FOTA_DOWNLOAD,
			   (if (bringup_fota_reboot_requested()) { break; }));
	}

	/* ---- assemble the record ---- */
	memset(rec, 0, sizeof(*rec));

#if defined(CONFIG_DATE_TIME)
	if (date_time_now(&rec->ts_ms) != 0) {
		rec->ts_ms = 0;
	}
#endif
	rec->seq = devcfg_next_seq();
	rec->uptime_s = (uint32_t)(k_uptime_get() / 1000);
	rec->mode = (uint8_t)cfg->mode;
	rec->interval_s = cfg->interval_s;

	int sats;
	bool fix;

	(void)bringup_gnss_get(&rec->lat, &rec->lon, &sats, &fix, &rec->hacc,
			       &rec->speed);
	rec->sats = (int16_t)sats;
	rec->fix = fix;
	bringup_gnss_quality(&rec->cn0, (int *)&rec->ttff_s, (int *)&rec->fix_age_s);
	/* "fix" = a fix within the last GNSS interval (+ margin), not just ever. */
	if (rec->fix_age_s < 0 ||
	    (cfg->gnss_s > 0 &&
	     rec->fix_age_s > (int32_t)MAX(cfg->gnss_s * 2, 60U) + 120)) {
		rec->fix = false;
	}

	if (n_acc) {
		rec->acc_x = (float)(sx / n_acc) * G_TO_MS2;
		rec->acc_y = (float)(sy / n_acc) * G_TO_MS2;
		rec->acc_z = (float)(sz / n_acc) * G_TO_MS2;
		rec->act = n_acc > 1 ?
			(float)sqrt(m_m2 / (n_acc - 1)) * G_TO_MS2 : 0.0f;
	}

	struct shp_trip trip;

	SHP_LOCK();
	shipment_window(&rec->shp);
	shipment_trip_get(&trip);
	SHP_UNLOCK();
	rec->trip = trip.active;
	rec->trip_s = trip.dur_s;
	rec->trip_ok = shipment_trip_ok(&trip);
	trip_persist(false);

	rec->t_in = nt[PT_PROD] ? (float)(st[PT_PROD] / nt[PT_PROD]) : NAN;
	rec->t_amb = nt[PT_AMB] ? (float)(st[PT_AMB] / nt[PT_AMB]) : NAN;
	rec->t_aux = nt[PT_SUN] ? (float)(st[PT_SUN] / nt[PT_SUN]) : NAN;

	(void)pmic_update(&rec->pm);
	rec->t_bat = rec->pm.present ? rec->pm.t_bat : NAN;
	rec->t_pmic = rec->pm.present ? rec->pm.t_die : NAN;

	(void)bringup_cell_monitor(&rec->cell);
	rec->leds = leds_report();

	const struct shp_win *w = &rec->shp;

	LOG_INF("Record seq=%u: acc %.2f/%.2f/%.2f act %.3f (n=%u) | T in/amb/aux "
		"%.2f/%.2f/%.2f | vbat %.3f soc %.0f chg %d | rsrp %d",
		rec->seq, (double)rec->acc_x, (double)rec->acc_y, (double)rec->acc_z,
		(double)rec->act, n_acc, (double)rec->t_in, (double)rec->t_amb,
		(double)rec->t_aux, (double)rec->pm.vbat, (double)rec->pm.soc,
		rec->pm.chg, rec->cell.valid ? rec->cell.rsrp : 0);
	if (w->valid) {
		LOG_INF("Shipment: shock %u (max %.1f g) fall %u (max %u ms) tilt %.0f deg "
			"orient %u | T %.1f..%.1f out %u s | events %u (+%u lost) | trip %s %u s %s",
			w->shocks, (double)w->shock_max, w->falls, w->fall_max_ms,
			(double)w->tilt, w->orient, (double)w->t_min, (double)w->t_max,
			w->t_out_s, w->n_evt, w->evt_lost, rec->trip ? "on" : "off",
			rec->trip_s, rec->trip_ok ? "ok" : "VIOLATED");
	}
}

/* ------------------------------ sending ---------------------------------- */

static bool first_connect_done;
static bool img_confirmed;

/* Publish a small status line on the data topic. */
static int publish_status(const char *fmt, ...)
{
	char buf[256];
	int len = snprintk(buf, sizeof(buf), "{\"device_id\":\"%s\",",
			   devcfg_device_id());
	va_list ap;

	va_start(ap, fmt);
	vsnprintk(buf + len, sizeof(buf) - len, fmt, ap);
	va_end(ap);
	return bringup_mqtt_session_publish(buf);
}

/* Connect if needed. On the very first connection after boot announce the
 * device and release GNSS (DK order: first message, then GPS).
 */
static int ensure_connected(void)
{
	static int64_t attach_fail_ms;

	if (bringup_mqtt_session_is_up()) {
		return 0;
	}
	/* No coverage (Funkloch, freezer): after a failed attach the modem keeps
	 * searching in the background. Do not block the sampling loop with a new
	 * attach attempt until it has registered again or ATTACH_RETRY_MS passed.
	 */
	if (attach_fail_ms && !bringup_cell_registered() &&
	    k_uptime_get() - attach_fail_ms < ATTACH_RETRY_MS) {
		LOG_INF("No cell yet — keeping %d record(s) in the backlog",
			telemetry_pending());
		return -ENETDOWN;
	}

	wdt_kick();
	int err = bringup_mqtt_session_start();

	wdt_kick();
	if (err == -ENETDOWN) {
		attach_fail_ms = k_uptime_get();
		bringup_cell_keep_searching();
	} else if (err == 0) {
		attach_fail_ms = 0;
	}
	if (err) {
		LOG_WRN("MQTT connect failed (%d) — record stays in the backlog (%d)",
			err, telemetry_pending());
		return err;
	}

	if (!first_connect_done) {
		struct app_cfg cfg;

		devcfg_get(&cfg);
		(void)publish_status("\"status\":\"online\",\"fw\":\"%s\",\"imei\":\"%s\","
				     "\"boot\":%u,\"rst\":\"%s\",\"mode\":\"%s\"}",
				     APP_FW_VERSION, bringup_cell_imei(),
				     devcfg_boot_count(), devcfg_reset_name(),
				     devcfg_mode_name(cfg.mode));
		bringup_gnss_begin();
		first_connect_done = true;
	}
	return 0;
}

/* Send the backlog oldest-first; a record is dropped only after its PUBACK.
 * Returns the number of records sent.
 */
static int flush_backlog(void)
{
	static char json[2048];
	int sent = 0;

	while (telemetry_pending() > 0 && sent < BACKLOG_PER_CYCLE) {
		struct tel_rec *r = telemetry_oldest();

		if (telemetry_format(r, json, sizeof(json)) < 0) {
			telemetry_drop_oldest(); /* cannot be rendered — skip it */
			continue;
		}
		int err = bringup_mqtt_session_publish(json);

		wdt_kick();
		if (err) {
			/* No PUBACK / link lost: keep it, retry next time (the
			 * backend files it under its own ts, so a duplicate just
			 * overwrites the same point).
			 */
			break;
		}
		telemetry_drop_oldest();
		sent++;
	}
	return sent;
}

/* Trip report (after start/stop/report) on versor/evt/<id>, retried until
 * it is acked.
 */
static void send_trip_report(void)
{
	static char buf[512];

	if (!atomic_get(&trip_report)) {
		return;
	}
	if (trip_format(buf, sizeof(buf)) < 0) {
		return;
	}
	wdt_kick();
	if (bringup_mqtt_session_publish_raw(buf) == 0) {
		atomic_set(&trip_report, 0);
	}
}

/* FOTA housekeeping on the main thread (status lines, confirm, reboot). */
static void fota_housekeeping(bool published_ok)
{
#if defined(CONFIG_FOTA_DOWNLOAD)
	/* A successful telemetry publish proves LTE + MQTT + sensors healthy:
	 * confirm the running image (no-op after a JLink flash; makes a freshly
	 * swapped FOTA image permanent).
	 */
	if (published_ok && !img_confirmed) {
		bringup_fota_confirm();
		img_confirmed = true;
	}

	char word[24];

	if (bringup_fota_dequeue_status(word, sizeof(word))) {
		(void)publish_status("\"fota\":\"%s\",\"fw\":\"%s\"}", word,
				     APP_FW_VERSION);
	}

	if (bringup_fota_reboot_requested()) {
		LOG_INF("FOTA image staged — rebooting so MCUboot swaps it in");
		(void)publish_status("\"fota\":\"rebooting\"}");
		k_sleep(K_SECONDS(2));
		sys_reboot(SYS_REBOOT_COLD);
	}
#else
	ARG_UNUSED(published_ok);
#endif
}

static void handle_reboot_request(void)
{
	if (atomic_get(&reboot_req)) {
		LOG_INF("Reboot requested");
		if (bringup_mqtt_session_is_up()) {
			(void)publish_status("\"status\":\"rebooting\"}");
			bringup_mqtt_session_stop();
		}
		k_sleep(K_SECONDS(1));
		sys_reboot(SYS_REBOOT_COLD);
	}
}

/* One send cycle after a window. */
static void send_cycle(const struct app_cfg *cfg)
{
	if (ensure_connected() != 0) {
		return;
	}

	/* In normal mode the record was taken offline: refresh the cell values
	 * now that the link is up (RSRP of the serving cell, freshly measured).
	 */
	if (cfg->mode == APP_MODE_NORMAL) {
		struct tel_rec *r = telemetry_newest();

		if (r) {
			(void)bringup_cell_monitor(&r->cell);
		}
	}

	int sent = flush_backlog();

	trip_requests();
	send_trip_report();
	fota_housekeeping(sent > 0);

	if (cfg->mode == APP_MODE_NORMAL) {
		/* Listen for the retained config + commands the backend sends
		 * in reply to our telemetry, then go quiet (PSM).
		 */
		(void)bringup_mqtt_session_poll(DOWNLINK_WAIT_MS);
		fota_housekeeping(false);
		handle_reboot_request();

		struct app_cfg now;

		devcfg_get(&now);
		if (now.mode == APP_MODE_NORMAL) {
			bringup_mqtt_session_stop();
		}
	}
	handle_reboot_request();
}

int main(void)
{
	/* Give the RTT host a moment to attach before the banner. */
	k_sleep(K_SECONDS(1));

	LOG_INF("========================================");
	LOG_INF(" Versor shipment logger %s (Pecura R1A hardware)", APP_FW_VERSION);
	LOG_INF(" board=%s  built %s %s", CONFIG_BOARD, __DATE__, __TIME__);
	LOG_INF(" I2C P0.30/31 | BMI323@0x68 | nPM1300@0x6B | Pt1000 AIN0-3/P0.07");
	LOG_INF("========================================");

	ARG_UNUSED(run_post);

	devcfg_init();
	wdt_start();

	/* Onboarding token (read-only) — also initialises the modem library. */
	bringup_print_attest_token();
	LOG_INF("Device ID %s, IMEI %s", devcfg_device_id(), bringup_cell_imei());
	devcfg_factory_info_update();

	if (bringup_imu_init() != 0) {
		LOG_WRN("BMI323 init failed — accel will read 0 (check I2C / VDD_1V8)");
	}
	(void)pmic_init();
	shipment_init();
	/* devcfg_init() ran settings_subsys_init(); load our own subtree. */
	(void)settings_load_subtree("vsr");
	if (trip_loaded && trip_stored.magic == SHP_TRIP_MAGIC && trip_stored.active) {
		shipment_trip_restore(&trip_stored);
		trip_persist(true);
		LOG_INF("Trip restored after reset (%u s so far)", trip_stored.dur_s);
	}

	/* FOTA: register the download handler and log the MCUboot swap/confirm
	 * state. A freshly swapped-in image starts unconfirmed; the loop confirms
	 * it after the first successful publish.
	 */
	IF_ENABLED(CONFIG_FOTA_DOWNLOAD, (bringup_fota_init();));

	struct app_cfg cfg;
	uint32_t applied_gen;

	devcfg_get(&cfg);
	applied_gen = devcfg_generation();
	apply_mode(&cfg);

	/* Connect right away in every mode: the unit shows up on the dashboard
	 * within a minute of power-on (Serienprüfung S-2) and picks up its
	 * retained config. In normal mode it then goes quiet until the first
	 * window is done.
	 */
	if (ensure_connected() == 0 && cfg.mode == APP_MODE_NORMAL) {
		(void)bringup_mqtt_session_poll(DOWNLINK_WAIT_MS);
		devcfg_get(&cfg);
		if (cfg.mode == APP_MODE_NORMAL) {
			bringup_mqtt_session_stop();
		}
	}
	handle_reboot_request();

	while (1) {
		struct tel_rec rec;

		devcfg_get(&cfg);
		if (devcfg_generation() != applied_gen) {
			applied_gen = devcfg_generation();
			apply_mode(&cfg);
		}

		telemetry_window(&cfg, &rec);
		handle_reboot_request();

		/* The config may have changed during the window: the record keeps
		 * the mode it was taken in, the send cycle uses the new one.
		 */
		telemetry_push(&rec);

		devcfg_get(&cfg);
		if (devcfg_generation() != applied_gen) {
			applied_gen = devcfg_generation();
			apply_mode(&cfg);
		}
		send_cycle(&cfg);
		wdt_kick();
	}
	return 0;
}

/* ------------------------------- shell ---------------------------------- */
#define WRAP(fn) do { LOCK(); int _r = fn; UNLOCK(); return _r; } while (0)

static int cmd_post(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	run_post();
	return 0;
}
static int cmd_i2cscan(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	WRAP(bringup_i2c_scan());
}
static int cmd_imu(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	LOCK();
	bringup_imu_init();
	int r = bringup_imu_read();
	UNLOCK();
	return r;
}
static int cmd_imuint(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	WRAP(bringup_imu_int_test());
}
static int cmd_temp(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	WRAP(bringup_temp_read());
}
static int cmd_pmic(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	WRAP(bringup_pmic_id());
}
static int cmd_cellinfo(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	return bringup_cell_info();
}
static int cmd_cell(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	return bringup_cell_attach();
}
static int cmd_reach(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	return bringup_cell_reach();
}
static int cmd_pub(const struct shell *sh, size_t argc, char **argv)
{
	if (argc >= 2) {
		/* Join argv[1..] with spaces so multi-word payloads work. */
		static char buf[256];
		size_t pos = 0;

		for (size_t i = 1; i < argc && pos < sizeof(buf) - 1; i++) {
			int n = snprintk(buf + pos, sizeof(buf) - pos, "%s%s",
					 argv[i], (i + 1 < argc) ? " " : "");
			if (n < 0) {
				break;
			}
			pos += n;
		}
		shell_print(sh, "Publishing custom payload (%u bytes)...",
			    (unsigned)pos);
		return bringup_mqtt_pub(buf);
	}
	shell_print(sh, "Publishing default JSON telemetry sample...");
	return bringup_mqtt_pub(NULL);
}
static int cmd_provca(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	return bringup_provision_ca();
}
static int cmd_gnss(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	return bringup_gnss_start();
}
static int cmd_gnssstat(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	return bringup_gnss_status();
}
static int cmd_gnssstop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	return bringup_gnss_stop();
}
#if defined(CONFIG_FOTA_DOWNLOAD)
static int cmd_fota(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "usage: bu fota <http(s)://host/path/app_update.bin>");
		return -EINVAL;
	}
	shell_print(sh, "Starting FOTA from %s ...", argv[1]);
	return bringup_fota_start_url(argv[1]);
}
static int cmd_fotaconfirm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	int r = bringup_fota_confirm();

	shell_print(sh, r ? "confirm failed: %d" : "running image confirmed", r);
	return r;
}
#endif
static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	enum app_mode m;

	if (argc < 2 || !devcfg_mode_from_name(argv[1], &m)) {
		shell_print(sh, "usage: bu mode normal|live|drain");
		return -EINVAL;
	}
	return devcfg_set_mode(m);
}
static int cmd_int(const struct shell *sh, size_t argc, char **argv)
{
	char json[48];

	if (argc < 2) {
		shell_print(sh, "usage: bu int <seconds> [gnss_seconds]");
		return -EINVAL;
	}
	if (argc >= 3) {
		snprintk(json, sizeof(json), "{\"int\":%s,\"gnss\":%s}", argv[1], argv[2]);
	} else {
		snprintk(json, sizeof(json), "{\"int\":%s}", argv[1]);
	}
	(void)devcfg_apply_json(json);
	return 0;
}
static int cmd_id(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "device id = %s (IMEI %s)", devcfg_device_id(),
			    bringup_cell_imei());
		return 0;
	}
	int err = devcfg_set_device_id(argv[1]);

	if (err) {
		shell_print(sh, "invalid id (max 23 chars, no + # / \" space)");
		return err;
	}
	shell_print(sh, "id stored — rebooting to use the new topics");
	app_request_reboot();
	return 0;
}
static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	struct app_cfg cfg;
	struct pmic_status pm;
	struct cell_status cs;
	float cn0;
	int ttff, age;

	devcfg_get(&cfg);
	(void)pmic_update(&pm);
	(void)bringup_cell_monitor(&cs);
	bringup_gnss_quality(&cn0, &ttff, &age);

	shell_print(sh, "id %s  fw %s  mode %s  int %u s  gnss %u s  led %d",
		    devcfg_device_id(), APP_FW_VERSION, devcfg_mode_name(cfg.mode),
		    cfg.interval_s, cfg.gnss_s, cfg.led);
	shell_print(sh, "boot %u  rst %s  backlog %d  mqtt %s",
		    devcfg_boot_count(), devcfg_reset_name(), telemetry_pending(),
		    bringup_mqtt_session_is_up() ? "up" : "down");
	if (pm.present) {
		shell_print(sh, "vbat %.3f V  ibat %.1f mA  soc %.0f %%  chg %d (st 0x%02x err 0x%02x)"
			    "  vbus %d  t_bat %.1f C  t_pmic %.1f C  reduced %d",
			    (double)pm.vbat, (double)pm.ibat_ma, (double)pm.soc, pm.chg,
			    pm.chg_raw, pm.chg_err, pm.vbus, (double)pm.t_bat,
			    (double)pm.t_die, pm.reduced);
	} else {
		shell_print(sh, "PMIC not present");
	}
	if (cs.valid) {
		shell_print(sh, "cell %u tac %u band %u plmn %s  rsrp %d dBm  rsrq %.1f dB  snr %.1f dB",
			    cs.cell_id, cs.tac, cs.band, cs.plmn, cs.rsrp,
			    (double)cs.rsrq, (double)cs.snr);
	} else {
		shell_print(sh, "cell: not registered");
	}
	shell_print(sh, "gnss cn0 %.1f dB-Hz  ttff %d s  fix age %d s",
		    (double)cn0, ttff, age);
	return 0;
}
static int cmd_ship(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	shell_print(sh, "Ship mode: device switches OFF, wake it by putting it on the Qi pad.");
	return pmic_ship_mode();
}
static int cmd_trip(const struct shell *sh, size_t argc, char **argv)
{
	static char buf[512];

	if (argc > 1 && !strcmp(argv[1], "start")) {
		app_trip_cmd(1);
		shell_print(sh, "Trip start queued (main loop, within 0.1 s). "
			    "Reference = the next still orientation.");
		return 0;
	}
	if (argc > 1 && !strcmp(argv[1], "stop")) {
		app_trip_cmd(0);
		shell_print(sh, "Trip stop queued.");
		return 0;
	}
	(void)trip_format(buf, sizeof(buf));
	shell_print(sh, "%s", buf);

	struct shp_params p;

	SHP_LOCK();
	shipment_get_params(&p);
	SHP_UNLOCK();
	shell_print(sh, "cfg: shock_mg %d fall_ms %d tilt %d t_lim %d t_lo %d t_hi %d alarm %d",
		    p.shock_mg, p.fall_ms, p.tilt_deg, p.t_lim, p.t_lo, p.t_hi, p.alarm);
	return 0;
}
static int cmd_reboot(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh); ARG_UNUSED(argc); ARG_UNUSED(argv);
	app_request_reboot();
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(bu_sub,
	SHELL_CMD(post,     NULL, "Run full power-on self-test",       cmd_post),
	SHELL_CMD(i2cscan,  NULL, "Scan the I2C bus",                  cmd_i2cscan),
	SHELL_CMD(imu,      NULL, "BMI323: id + init + one sample",    cmd_imu),
	SHELL_CMD(imuint,   NULL, "BMI323: INT1/P0.05 edge test",      cmd_imuint),
	SHELL_CMD(temp,     NULL, "Read the 3 Pt1000 temperatures",    cmd_temp),
	SHELL_CMD(pmic,     NULL, "Probe nPM1300 @0x6B",               cmd_pmic),
	SHELL_CMD(cellinfo, NULL, "Modem IMEI / temp / voltage",       cmd_cellinfo),
	SHELL_CMD(cell,     NULL, "Attach LTE-M + signal",             cmd_cell),
	SHELL_CMD(reach,    NULL, "DNS + TCP-connect to the broker",   cmd_reach),
	SHELL_CMD(provca,   NULL, "Provision broker CA to sec tag (offline)", cmd_provca),
	SHELL_CMD(pub,      NULL, "TLS+MQTT publish [text]|default JSON", cmd_pub),
	SHELL_CMD(gnss,     NULL, "Start GNSS acquisition",            cmd_gnss),
	SHELL_CMD(gnssstat, NULL, "Show GNSS satellites / fix",        cmd_gnssstat),
	SHELL_CMD(gnssstop, NULL, "Stop GNSS",                         cmd_gnssstop),
#if defined(CONFIG_FOTA_DOWNLOAD)
	SHELL_CMD(fota,     NULL, "fota <url> : download+swap app_update.bin", cmd_fota),
	SHELL_CMD(fotaconfirm, NULL, "Confirm running image (no rollback)", cmd_fotaconfirm),
#endif
	SHELL_CMD(mode,     NULL, "mode normal|live|drain",            cmd_mode),
	SHELL_CMD(int,      NULL, "int <s> [gnss_s] : send / GNSS interval", cmd_int),
	SHELL_CMD(id,       NULL, "id [P0.1.x] : show / set device id (reboots)", cmd_id),
	SHELL_CMD(status,   NULL, "Config, battery, cell, GNSS quality", cmd_status),
	SHELL_CMD(ship,     NULL, "PMIC ship mode (off until Qi/VBUS)", cmd_ship),
	SHELL_CMD(trip,     NULL, "trip [start|stop] : shipment trip / show", cmd_trip),
	SHELL_CMD(reboot,   NULL, "Reboot",                            cmd_reboot),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(bu, &bu_sub, "Versor shipment logger (Pecura R1A)", NULL);
