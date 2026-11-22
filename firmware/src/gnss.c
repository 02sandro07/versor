/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * GNSS bring-up, DK-style. Order matches the working nRF9151-DK firmware:
 *   1) LTE attaches and the first ("online") MQTT message goes out.
 *   2) THEN GNSS starts, requests A-GNSS assistance (ephemerides) from nRF
 *      Cloud over REST while LTE is still free, and once that download is done
 *      switches to GNSS priority mode for a fast first fix.
 *   3) After the first fix it switches to the interval of the operating mode
 *      (bringup_gnss_set_interval: off / continuous / periodic N s). Periodic
 *      leaves the shared RF frontend to LTE between fixes. In "drain" mode it
 *      stays continuous: GNSS then only gets the antenna in the gaps between
 *      LTE activity, which is enough to keep tracking outdoors.
 *
 * A-GNSS is best-effort: if the board is not yet onboarded to nRF Cloud the
 * JWT/REST call fails (-77) and GNSS keeps searching standalone (priority mode
 * + network time still help a lot vs. a truly cold start). Needs the GPS
 * antenna fitted, VDD_GNSS (1.8 V) supplied and open sky.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <modem/lte_lc.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#if defined(CONFIG_NRF_CLOUD_AGNSS)
#include <date_time.h>
#include "assistance.h"
#define APP_GNSS_ASSISTANCE 1
#endif

#include <math.h>

#include "bringup.h"

LOG_MODULE_REGISTER(gnss, LOG_LEVEL_INF);

/* Default fix interval (s) until main() applies the mode's value. */
#define GNSS_PERIODIC_INTERVAL_S 120
/* Consecutive LTE-blocked PVT events before we force GNSS priority mode. */
#define BLOCKED_THRESHOLD 5
/* Hard backstop: enable GNSS priority mode this many seconds after GNSS start
 * even if the A-GNSS fetch has not reported done. A-GNSS is best-effort and
 * must NEVER be able to hold GNSS hostage — if assistance stalls, GNSS still
 * gets the antenna and falls back to a (slower) standalone fix.
 */
#define AGNSS_PRIO_TIMEOUT_S 90
/* The first fix is searched continuously, but only this long: a collar
 * switched on indoors (barn) may never see the sky, and an endless continuous
 * search keeps receiver + LNA at 30-50 mA. After that the requested periodic
 * interval applies even without a fix (each search capped at 120 s).
 */
#define FIRST_FIX_TIMEOUT_S 600

static K_SEM_DEFINE(pvt_sem, 0, 1);
static K_MUTEX_DEFINE(fix_lock);

/* Given once the first ("online") MQTT message is out — gates the whole GNSS
 * search so LTE does its initial work first (see file header).
 */
static K_SEM_DEFINE(first_msg_sem, 0, 1);

/* Set once the one-shot A-GNSS download has been attempted (success OR error).
 * BEFORE that the priority mode must NOT be enabled: priority blocks LTE, and
 * the A-GNSS REST call needs LTE. Without assistance built in, true immediately.
 */
static atomic_t agnss_fetch_done = ATOMIC_INIT(0);
static bool running;

#if defined(APP_GNSS_ASSISTANCE)
static K_SEM_DEFINE(agnss_sem, 0, 1);
static struct nrf_modem_gnss_agnss_data_frame agnss_request;
#endif

static struct {
	double lat;
	double lon;
	bool valid;
	int sats;
	float accuracy;
	float speed_kmh;
	float cn0;          /* mean C/N0 of the 4 strongest SVs, dB-Hz     */
	int ttff_s;         /* time to fix of the latest acquisition       */
	int64_t fix_ms;     /* uptime of the last valid fix (0 = never)    */
} last_fix = { .cn0 = NAN, .ttff_s = -1 };

/* Requested fix interval (0 off, 1 continuous, >= 10 periodic) and the one the
 * receiver currently runs with. Changed by main(), applied by gnss_thread.
 */
static atomic_t target_interval = ATOMIC_INIT(GNSS_PERIODIC_INTERVAL_S);
static uint32_t cur_interval; /* 0 while stopped */

/* 32-bit uptime (ms, | 1) of the thread's last loop pass; 0 until it runs. */
static atomic_t last_alive_ms;

uint32_t bringup_gnss_last_alive_ms(void)
{
	return (uint32_t)atomic_get(&last_alive_ms);
}

void bringup_gnss_begin(void)
{
	k_sem_give(&first_msg_sem);
}

/* True once the one-shot A-GNSS fetch has finished (success OR error). main()
 * waits for this (bounded) before taking LTE offline, so the ephemeris download
 * — which needs LTE — gets a chance to complete first. Without assistance built
 * in this is true immediately.
 */
bool bringup_gnss_agnss_done(void)
{
	return atomic_get(&agnss_fetch_done) != 0;
}

bool bringup_gnss_get(double *lat, double *lon, int *sats, bool *fix_valid,
		      float *accuracy, float *speed_kmh)
{
	bool valid;

	k_mutex_lock(&fix_lock, K_FOREVER);
	valid = last_fix.valid;
	if (lat) {
		*lat = last_fix.lat;
	}
	if (lon) {
		*lon = last_fix.lon;
	}
	if (sats) {
		*sats = last_fix.sats;
	}
	if (fix_valid) {
		*fix_valid = last_fix.valid;
	}
	if (accuracy) {
		*accuracy = last_fix.accuracy;
	}
	if (speed_kmh) {
		*speed_kmh = last_fix.valid ? last_fix.speed_kmh : 0.0f;
	}
	k_mutex_unlock(&fix_lock);

	return valid;
}

void bringup_gnss_quality(float *cn0, int *ttff_s, int *fix_age_s)
{
	k_mutex_lock(&fix_lock, K_FOREVER);
	if (cn0) {
		*cn0 = last_fix.cn0;
	}
	if (ttff_s) {
		*ttff_s = last_fix.ttff_s;
	}
	if (fix_age_s) {
		*fix_age_s = last_fix.fix_ms ?
			(int)((k_uptime_get() - last_fix.fix_ms) / 1000) : -1;
	}
	k_mutex_unlock(&fix_lock);
}

void bringup_gnss_set_interval(uint32_t interval_s)
{
	atomic_set(&target_interval, (atomic_val_t)interval_s);
	k_sem_give(&pvt_sem); /* wake the thread so it applies the change */
}

/* Mean C/N0 of the 4 strongest tracked satellites (dB-Hz), NAN if < 4. */
static float cn0_top4(const struct nrf_modem_gnss_pvt_data_frame *pvt)
{
	uint16_t top[4] = { 0 };
	int n = 0;

	for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
		uint16_t c = pvt->sv[i].cn0; /* 0.1 dB-Hz */

		if (pvt->sv[i].sv == 0 || c == 0) {
			continue;
		}
		n++;
		for (int k = 0; k < 4; k++) {
			if (c > top[k]) {
				for (int m = 3; m > k; m--) {
					top[m] = top[m - 1];
				}
				top[k] = c;
				break;
			}
		}
	}
	if (n < 4) {
		return NAN;
	}
	return (float)(top[0] + top[1] + top[2] + top[3]) / 40.0f;
}

static int count_sv(const struct nrf_modem_gnss_pvt_data_frame *pvt)
{
	int n = 0;

	for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
		if (pvt->sv[i].sv != 0 && pvt->sv[i].cn0 > 0) {
			n++;
		}
	}
	return n;
}

/* Called in modem-library context — keep it minimal. */
static void gnss_evt(int event)
{
	switch (event) {
	case NRF_MODEM_GNSS_EVT_PVT:
		k_sem_give(&pvt_sem);
		break;
	case NRF_MODEM_GNSS_EVT_BLOCKED:
		LOG_WRN("GNSS blocked by LTE");
		break;
	case NRF_MODEM_GNSS_EVT_UNBLOCKED:
		LOG_INF("GNSS unblocked, resuming");
		break;
#if defined(APP_GNSS_ASSISTANCE)
	case NRF_MODEM_GNSS_EVT_AGNSS_REQ:
		if (nrf_modem_gnss_read(&agnss_request, sizeof(agnss_request),
					NRF_MODEM_GNSS_DATA_AGNSS_REQ) == 0) {
			k_sem_give(&agnss_sem);
		}
		break;
#endif
	default:
		break;
	}
}

#if defined(APP_GNSS_ASSISTANCE)
/* Assistance fetches must run in thread context (socket / HTTPS REST), not in
 * the GNSS event handler.
 */
static void agnss_thread(void)
{
	if (assistance_init(NULL) != 0) {
		LOG_ERR("Assistance init failed — GNSS runs standalone");
		atomic_set(&agnss_fetch_done, 1);
		return;
	}

	while (1) {
		k_sem_take(&agnss_sem, K_FOREVER);

		LOG_INF("A-GNSS data requested by modem");

		/* The modem asks right at GNSS start, but JWT signing needs a valid
		 * network time and the REST call needs LTE. date_time becomes valid
		 * only once LTE attached — wait for it (bounded).
		 */
		int64_t ts_unused;
		int waited_s = 0;

		while (date_time_now(&ts_unused) != 0) {
			if (waited_s == 0) {
				LOG_INF("Waiting for network time before A-GNSS fetch...");
			}
			k_sleep(K_SECONDS(2));
			waited_s += 2;
			if (waited_s >= 90) {
				LOG_WRN("No network time after %ds, trying A-GNSS anyway",
					waited_s);
				break;
			}
		}

		LOG_INF("Fetching A-GNSS assistance from nRF Cloud...");
		int err = assistance_request(&agnss_request);

		if (err) {
			LOG_WRN("A-GNSS failed (%d) — GNSS continues standalone "
				"(slow cold start; onboard the board to fix this)", err);
		} else {
			LOG_INF("A-GNSS injected — fast fix expected");
		}

		/* Success or fail: priority mode may now run (LTE no longer needed). */
		atomic_set(&agnss_fetch_done, 1);
	}
}

K_THREAD_DEFINE(agnss_thread_id, 6144, agnss_thread, NULL, NULL, NULL, 6, 0, 0);
#endif /* APP_GNSS_ASSISTANCE */

static int gnss_setup(void)
{
	int err;

	/* Enable GNSS on top of LTE (LTE-M + GPS system mode). With the modem in
	 * full functionality (lte_lc_connect -> CFUN=1) GNSS is usually already on;
	 * ACTIVATE_GNSS is best-effort, so a benign error here is not fatal.
	 */
	err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
	if (err) {
		LOG_WRN("ACTIVATE_GNSS func mode returned %d (GNSS may already be on)",
			err);
	}

	/* Route COEX0 to enable the external GNSS LNA in the GPS band (SKY65943
	 * VEN <- COEX0), like the DK.
	 */
	if (nrf_modem_at_printf("AT%%XCOEX0=1,1,1565,1586") != 0) {
		LOG_WRN("AT%%XCOEX0 config rejected — external LNA may stay off");
	}

	err = nrf_modem_gnss_event_handler_set(gnss_evt);
	if (err) {
		LOG_ERR("gnss event handler set: %d", err);
		return err;
	}

	(void)nrf_modem_gnss_fix_interval_set(1); /* continuous until first fix */
	(void)nrf_modem_gnss_fix_retry_set(0);    /* search indefinitely        */

	err = nrf_modem_gnss_start();
	if (err) {
		LOG_ERR("nrf_modem_gnss_start: %d", err);
		return err;
	}

	return 0;
}

/* (Re)start the receiver with the given fix interval (1 or >= 10), or stop it
 * for 0. Must run on the GNSS thread.
 */
static void apply_interval(uint32_t iv)
{
	if (iv == cur_interval) {
		return;
	}
	if (cur_interval != 0) {
		(void)nrf_modem_gnss_stop();
	}
	cur_interval = 0;
	if (iv == 0) {
		LOG_INF("GNSS off");
		return;
	}
	if (nrf_modem_gnss_fix_interval_set(iv) != 0) {
		LOG_WRN("GNSS fix interval %u rejected", iv);
		return;
	}
	/* Periodic: give each search up to 120 s, then sleep until the next slot.
	 * Continuous: search forever.
	 */
	(void)nrf_modem_gnss_fix_retry_set(iv == 1 ? 0 : 120);
	if (nrf_modem_gnss_start() == 0) {
		cur_interval = iv;
		LOG_INF("GNSS %s", iv == 1 ? "continuous" : "periodic");
		if (iv > 1) {
			LOG_INF("GNSS periodic interval %u s", iv);
		}
	}
}

static void gnss_thread(void)
{
	struct nrf_modem_gnss_pvt_data_frame pvt;
	int err;
	bool first_fix_done = false;
	bool first_search_expired = false;
	bool prio_enabled = false;
	bool searching = false;   /* a search without fix is in progress */
	int pvt_count = 0;
	int blocked_count = 0;

	/* Wait until the first ("online") MQTT message is out. */
	LOG_INF("GNSS waiting for first MQTT publish...");
	k_sem_take(&first_msg_sem, K_FOREVER);

	/* Retry setup until the modem accepts GNSS commands (EACCES = not ready). */
	do {
		k_sleep(K_SECONDS(2));
		err = gnss_setup();
		if (err) {
			LOG_WRN("gnss_setup failed (%d), retrying...", err);
		}
	} while (err);

	running = true;
	cur_interval = 1; /* gnss_setup() started it continuous for the first fix */

#if !defined(APP_GNSS_ASSISTANCE)
	atomic_set(&agnss_fetch_done, 1);
#endif

	int64_t gnss_start_ms = k_uptime_get();
	int64_t search_start_ms = gnss_start_ms;

	LOG_INF("GNSS started (continuous until first fix; priority after A-GNSS)");

	/* First fix is always searched continuously (unless GNSS is switched off
	 * entirely); afterwards the requested interval applies.
	 */
	if (atomic_get(&target_interval) == 0) {
		apply_interval(0);
	}

	while (1) {
		bool got_pvt = k_sem_take(&pvt_sem, K_SECONDS(5)) == 0;

		atomic_set(&last_alive_ms, (atomic_val_t)(k_uptime_get_32() | 1U));
		uint32_t want = (uint32_t)atomic_get(&target_interval);

		if (!first_fix_done && !first_search_expired && want > 1 &&
		    (k_uptime_get() - gnss_start_ms) >=
			    ((int64_t)FIRST_FIX_TIMEOUT_S * 1000)) {
			first_search_expired = true;
			prio_enabled = false; /* gnss_stop in apply_interval ends it */
			LOG_WRN("GNSS: no first fix after %d s, going periodic",
				FIRST_FIX_TIMEOUT_S);
		}

		if (want == 0 || first_fix_done || first_search_expired) {
			if (want != cur_interval) {
				apply_interval(want);
				searching = false;
				search_start_ms = k_uptime_get();
			}
		} else if (cur_interval == 0) {
			/* Re-enabled before the first fix: search continuously. */
			apply_interval(1);
			search_start_ms = k_uptime_get();
		}

		if (!got_pvt || cur_interval == 0) {
			continue;
		}
		if (nrf_modem_gnss_read(&pvt, sizeof(pvt),
					NRF_MODEM_GNSS_DATA_PVT) != 0) {
			continue;
		}

		/* Priority mode may run once A-GNSS reported done OR the backstop
		 * timeout elapsed — the latter guarantees a stalled/slow assistance
		 * fetch can never permanently starve GNSS (see AGNSS_PRIO_TIMEOUT_S).
		 */
		bool agnss_ready = atomic_get(&agnss_fetch_done) ||
			(k_uptime_get() - gnss_start_ms) >=
				((int64_t)AGNSS_PRIO_TIMEOUT_S * 1000);

		/* Once A-GNSS is done (or the backstop fired), enable priority mode
		 * once for the first fix. The modem disables it after the fix.
		 */
		if (!prio_enabled && !first_fix_done && !first_search_expired &&
		    agnss_ready) {
			if (nrf_modem_gnss_prio_mode_enable() == 0) {
				prio_enabled = true;
				LOG_INF("GNSS priority mode ENABLED for first fix%s",
					atomic_get(&agnss_fetch_done) ? "" :
						" (A-GNSS backstop timeout)");
			}
		}

		int sv = count_sv(&pvt);
		float cn0 = cn0_top4(&pvt);
		bool fix = (pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) != 0;

		/* A new search starts with the first non-fix PVT after a fix
		 * (periodic wake-up) — that is the reference for ttff.
		 */
		if (!fix && !searching) {
			searching = true;
			if (first_fix_done) {
				search_start_ms = k_uptime_get();
			}
		}

		k_mutex_lock(&fix_lock, K_FOREVER);
		last_fix.sats = sv;
		if (!isnan(cn0)) {
			last_fix.cn0 = cn0;
		}
		k_mutex_unlock(&fix_lock);

		if (++pvt_count % 10 == 0 && !fix) {
			LOG_INF("GNSS searching: %d satellites tracked, flags=0x%02x",
				sv, pvt.flags);
		}

		if (fix) {
			float speed_kmh = pvt.speed * 3.6f;
			int64_t now = k_uptime_get();

			k_mutex_lock(&fix_lock, K_FOREVER);
			last_fix.lat = pvt.latitude;
			last_fix.lon = pvt.longitude;
			last_fix.accuracy = pvt.accuracy;
			last_fix.speed_kmh = speed_kmh;
			last_fix.valid = true;
			if (searching || !first_fix_done) {
				last_fix.ttff_s = (int)((now - search_start_ms) / 1000);
			}
			last_fix.fix_ms = now;
			k_mutex_unlock(&fix_lock);

			blocked_count = 0;
			prio_enabled = false; /* modem cleared it after the fix */

			if (searching || !first_fix_done) {
				LOG_INF("GNSS FIX: lat=%.6f lon=%.6f acc=%.1fm sats=%d "
					"cn0=%.1f ttff=%ds",
					pvt.latitude, pvt.longitude,
					(double)pvt.accuracy, sv, (double)cn0,
					last_fix.ttff_s);
			}
			searching = false;

			if (!first_fix_done) {
				first_fix_done = true;
				apply_interval((uint32_t)atomic_get(&target_interval));
			}
		} else if (pvt.flags & (NRF_MODEM_GNSS_PVT_FLAG_DEADLINE_MISSED |
					NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME)) {
			blocked_count++;
			if (blocked_count >= BLOCKED_THRESHOLD && !prio_enabled &&
			    agnss_ready && !first_fix_done && !first_search_expired) {
				if (nrf_modem_gnss_prio_mode_enable() == 0) {
					prio_enabled = true;
					LOG_INF("GNSS priority mode ENABLED (LTE was blocking)");
				}
			}
		} else {
			blocked_count = 0;
		}
	}
}

K_THREAD_DEFINE(gnss_thread_id, 4096, gnss_thread, NULL, NULL, NULL, 5, 0, 0);

/* ---- Shell / legacy entry points --------------------------------------- */

int bringup_gnss_start(void)
{
	/* Manual start from the shell: just release the acquisition thread. */
	bringup_gnss_begin();
	LOG_INF("GNSS acquisition released (bu gnssstat to poll)");
	return 0;
}

int bringup_gnss_stop(void)
{
	/* Handled by the GNSS thread like any interval change (0 = off). */
	bringup_gnss_set_interval(0);
	LOG_INF("GNSS stop requested");
	return 0;
}

int bringup_gnss_status(void)
{
	double lat, lon;
	int sats;
	bool fix;
	float acc, speed;

	if (!running) {
		LOG_INF("GNSS not running yet (waiting for first MQTT publish)");
		return 0;
	}

	(void)bringup_gnss_get(&lat, &lon, &sats, &fix, &acc, &speed);
	if (fix) {
		LOG_INF("GNSS FIX: lat=%.6f lon=%.6f acc=%.1f m, %d sats, %.1f km/h",
			lat, lon, (double)acc, sats, (double)speed);
	} else {
		LOG_INF("GNSS searching: %d satellite(s) tracked, no fix yet", sats);
	}
	return 0;
}
