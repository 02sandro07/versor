/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Telemetry records, the RAM backlog and the JSON format sent to the NAS.
 *
 * Every telemetry window produces one record. Records go into a RAM ring and
 * are sent oldest-first whenever MQTT is up; a record leaves the ring only after
 * its PUBACK. So a dead spot (Funkloch, freezer, cookie tin) just delays the
 * data: it is sent with its original "ts" once the link is back and the backend
 * files it under that time. RAM only for now — lost on reset. A flash-backed
 * buffer comes later if space allows after FOTA (see HANDOFF).
 *
 * Capacity TEL_RING_LEN records: 8 h at the 5-min "normal" interval, 48 min at
 * 30 s "live". Oldest records are dropped when full.
 *
 * JSON (one flat object; absent/invalid values are null):
 *   device_id, ts (unix s, 0 = no network time yet), seq, fw, mode, int, up,
 *   boot, rst, bl (backlog incl. this one)
 *   lat, lon, sat, fix, hacc, speed, cn0, ttff, fix_age
 *   acc_x, acc_y, acc_z (mean, m/s^2), act (mean |a - mean|, m/s^2)
 *   t_skin, t_amb, t_sun, t_bat, t_pmic (C)
 *   vbat (V), ibat (mA, + charging), soc (%), chg (0 no / 1 charging / 2 full),
 *   chg_st (raw status), chg_err, vbus, chg_red (warm-reduced current)
 *   rsrp (dBm), rsrq (dB), snr (dB), cell, tac, band, plmn
 *   led (bit0 red, bit1 green, bit2 blue)
 * From 0.4.0 (motion.c, docs/PLAN-ERKENNUNG.md 1.5), only when the 50 Hz FIFO ran:
 *   min_graze, min_rum, min_walk, min_run, min_rest, min_other, min_lat
 *   (SECONDS per class in the window, the name is historic), steps, pos_chg,
 *   still_max (s), acc_peak (m/s^2), pitch, roll, pitch_sd (deg),
 *   gait_n, gait_f (Hz), gait_reg, gait_asym, gait_bob (m/s^2)
 *   t_skin_min, t_skin_max, t_skin_sd (C)
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bringup.h"

LOG_MODULE_REGISTER(telemetry, LOG_LEVEL_INF);

#define TEL_RING_LEN 96

static struct tel_rec ring[TEL_RING_LEN];
static int head;  /* next write slot   */
static int count; /* records in ring   */
static uint32_t dropped;
static K_MUTEX_DEFINE(ring_lock);

void telemetry_push(const struct tel_rec *r)
{
	k_mutex_lock(&ring_lock, K_FOREVER);
	if (count == TEL_RING_LEN) {
		/* Full: overwrite the oldest. */
		count--;
		dropped++;
		LOG_WRN("Backlog full — dropped oldest record (%u total)", dropped);
	}
	ring[head] = *r;
	head = (head + 1) % TEL_RING_LEN;
	count++;
	k_mutex_unlock(&ring_lock);
}

int telemetry_pending(void)
{
	return count;
}

struct tel_rec *telemetry_oldest(void)
{
	struct tel_rec *r = NULL;

	k_mutex_lock(&ring_lock, K_FOREVER);
	if (count) {
		r = &ring[(head - count + TEL_RING_LEN) % TEL_RING_LEN];
	}
	k_mutex_unlock(&ring_lock);
	return r;
}

struct tel_rec *telemetry_newest(void)
{
	struct tel_rec *r = NULL;

	k_mutex_lock(&ring_lock, K_FOREVER);
	if (count) {
		r = &ring[(head - 1 + TEL_RING_LEN) % TEL_RING_LEN];
	}
	k_mutex_unlock(&ring_lock);
	return r;
}

void telemetry_drop_oldest(void)
{
	k_mutex_lock(&ring_lock, K_FOREVER);
	if (count) {
		count--;
	}
	k_mutex_unlock(&ring_lock);
}

/* ------------------------------- JSON ------------------------------------ */

struct jbuf {
	char *p;
	size_t n;
	size_t len;
	bool overflow;
};

static void jb_add(struct jbuf *b, const char *fmt, ...)
{
	va_list ap;

	if (b->overflow) {
		return;
	}
	va_start(ap, fmt);
	int w = vsnprintk(b->p + b->len, b->n - b->len, fmt, ap);

	va_end(ap);
	if (w < 0 || (size_t)w >= b->n - b->len) {
		b->overflow = true;
		return;
	}
	b->len += w;
}

/* "key":value with %.Nf, or null for NAN / !valid. */
static void jb_float(struct jbuf *b, const char *key, float v, int prec, bool valid)
{
	if (!valid || isnan(v) || isinf(v)) {
		jb_add(b, ",\"%s\":null", key);
	} else {
		jb_add(b, ",\"%s\":%.*f", key, prec, (double)v);
	}
}

static void jb_int(struct jbuf *b, const char *key, int64_t v, bool valid)
{
	if (!valid) {
		jb_add(b, ",\"%s\":null", key);
	} else {
		jb_add(b, ",\"%s\":%lld", key, (long long)v);
	}
}

int telemetry_format(const struct tel_rec *r, char *buf, size_t n)
{
	struct jbuf b = { .p = buf, .n = n };
	const struct pmic_status *pm = &r->pm;
	const struct cell_status *c = &r->cell;

	jb_add(&b, "{\"device_id\":\"%s\",\"ts\":%lld,\"seq\":%u,\"fw\":\"%s\"",
	       devcfg_device_id(), (long long)(r->ts_ms / 1000), r->seq,
	       APP_FW_VERSION);
	jb_add(&b, ",\"mode\":\"%s\",\"int\":%u,\"up\":%u,\"boot\":%u,\"rst\":\"%s\",\"bl\":%d",
	       devcfg_mode_name((enum app_mode)r->mode), r->interval_s, r->uptime_s,
	       devcfg_boot_count(), devcfg_reset_name(), telemetry_pending());

	/* GNSS (lat/lon keep the last fix; "fix" says if it is current). */
	jb_add(&b, ",\"lat\":%.6f,\"lon\":%.6f,\"sat\":%d,\"fix\":%d",
	       r->lat, r->lon, r->sats, r->fix ? 1 : 0);
	jb_float(&b, "hacc", r->hacc, 1, r->fix);
	jb_float(&b, "speed", r->speed, 1, r->fix);
	jb_float(&b, "cn0", r->cn0, 1, true);
	jb_int(&b, "ttff", r->ttff_s, r->ttff_s >= 0);
	jb_int(&b, "fix_age", r->fix_age_s, r->fix_age_s >= 0);

	/* IMU */
	jb_add(&b, ",\"acc_x\":%.2f,\"acc_y\":%.2f,\"acc_z\":%.2f,\"act\":%.3f",
	       (double)r->acc_x, (double)r->acc_y, (double)r->acc_z, (double)r->act);

	/* Temperatures */
	if (r->mot_ok) {
		static const char *const k[6] = { "min_graze", "min_rum", "min_walk",
						  "min_run", "min_rest", "min_other" };

		for (int i = 0; i < 6; i++) {
			jb_add(&b, ",\"%s\":%u", k[i], r->sec[i]);
		}
		jb_add(&b, ",\"min_lat\":%u,\"steps\":%u,\"pos_chg\":%u,\"still_max\":%u",
		       r->sec_lat, r->steps, r->pos_chg, r->still_max);
		jb_float(&b, "acc_peak", r->acc_peak, 2, true);
		jb_float(&b, "pitch", r->pitch, 1, true);
		jb_float(&b, "roll", r->roll, 1, true);
		jb_float(&b, "pitch_sd", r->pitch_sd, 1, true);
		jb_add(&b, ",\"gait_n\":%u", r->gait_n);
		jb_float(&b, "gait_f", r->gait_f, 2, r->gait_n > 0);
		jb_float(&b, "gait_reg", r->gait_reg, 2, r->gait_n > 0);
		jb_float(&b, "gait_asym", r->gait_asym, 2, r->gait_n > 0);
		jb_float(&b, "gait_bob", r->gait_bob, 2, r->gait_n > 0);
	}

	jb_float(&b, "t_skin", r->t_skin, 2, true);
	jb_float(&b, "t_skin_min", r->t_skin_min, 2, true);
	jb_float(&b, "t_skin_max", r->t_skin_max, 2, true);
	jb_float(&b, "t_skin_sd", r->t_skin_sd, 3, true);
	jb_float(&b, "t_amb", r->t_amb, 2, true);
	jb_float(&b, "t_sun", r->t_sun, 2, true);
	jb_float(&b, "t_bat", r->t_bat, 1, pm->present);
	jb_float(&b, "t_pmic", r->t_pmic, 1, pm->present);

	/* Power */
	jb_float(&b, "vbat", pm->vbat, 3, pm->present);
	jb_float(&b, "ibat", pm->ibat_ma, 1, pm->present);
	jb_float(&b, "soc", pm->soc, 1, pm->present);
	jb_int(&b, "chg", pm->chg, pm->present);
	jb_int(&b, "chg_st", pm->chg_raw, pm->present);
	jb_int(&b, "chg_err", pm->chg_err, pm->present);
	jb_int(&b, "vbus", pm->vbus ? 1 : 0, pm->present);
	jb_int(&b, "chg_red", pm->reduced ? 1 : 0, pm->present);

	/* Cellular */
	jb_int(&b, "rsrp", c->rsrp, c->valid);
	jb_float(&b, "rsrq", c->rsrq, 1, c->valid);
	jb_float(&b, "snr", c->snr, 1, c->valid);
	jb_int(&b, "cell", c->cell_id, c->valid);
	jb_int(&b, "tac", c->tac, c->valid);
	jb_int(&b, "band", c->band, c->valid);
	if (c->valid && c->plmn[0]) {
		jb_add(&b, ",\"plmn\":\"%s\"", c->plmn);
	} else {
		jb_add(&b, ",\"plmn\":null");
	}

	jb_add(&b, ",\"led\":%u}", r->leds);

	if (b.overflow) {
		LOG_ERR("telemetry JSON truncated (buffer %u)", (unsigned)n);
		return -ENOMEM;
	}
	return (int)b.len;
}

int telemetry_format_latest(char *buf, size_t n)
{
	struct tel_rec *r = telemetry_newest();

	if (r) {
		return telemetry_format(r, buf, n);
	}
	return snprintk(buf, n, "{\"device_id\":\"%s\",\"fw\":\"%s\",\"status\":\"no_data_yet\"}",
			devcfg_device_id(), APP_FW_VERSION);
}
