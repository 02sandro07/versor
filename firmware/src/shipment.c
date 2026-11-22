/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Versor shipment events — see shipment.h. Pure C, no Zephyr; called from
 * the main thread only (no locking).
 */
#include "shipment.h"

#include <math.h>
#include <string.h>

/* devcfg.c (firmware) or the host test provides it. */
bool json_get_int(const char *json, const char *key, int32_t *out);

#define SHOCK_GAP_N   (SHP_SHOCK_GAP_MS * SHP_HZ / 1000)
#define MS_PER_SAMPLE (1000 / SHP_HZ)
#define STILL_SD_G    0.05f
#define STILL_MAG_G   0.10f
#define RAD2DEG       57.29578f

static struct shp_params prm;

static int64_t now_ts;
static uint32_t now_up;
static uint32_t last_up;
static bool last_up_ok;

static struct shp_win win;
static bool alarm_flag;

static struct shp_trip trip;
static bool trip_dirty;

/* shock */
static bool in_shock;
static float shock_peak;
static int shock_below;

/* free fall */
static uint32_t fall_n;

/* tilt: 1-s blocks */
static float blk_x, blk_y, blk_z, blk_m, blk_m2;
static int blk_n;
static bool ref_ok;
static float ref[3];
static float tilt_cur = NAN;
static uint8_t orient;

/* temperature */
static int8_t t_state;
static uint32_t t_out_ms_win, t_out_ms_trip;

static const char *const evt_names[SHP_EV_COUNT] = {
	"shock", "fall", "tilt", "flip", "t_hi", "t_lo", "t_ok",
};

const char *shipment_evt_name(uint8_t type)
{
	return type < SHP_EV_COUNT ? evt_names[type] : "?";
}

static int32_t clampi(int32_t v, int32_t lo, int32_t hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

static void win_reset(void)
{
	memset(&win, 0, sizeof(win));
	win.tilt = NAN;
	win.tilt_max = NAN;
	win.t_min = NAN;
	win.t_max = NAN;
	t_out_ms_win = 0;
}

static void trip_reset(void)
{
	memset(&trip, 0, sizeof(trip));
	trip.magic = SHP_TRIP_MAGIC;
	trip.tilt_max = NAN;
	trip.t_min = NAN;
	trip.t_max = NAN;
	t_out_ms_trip = 0;
}

void shipment_init(void)
{
	prm.shock_mg = 3000;
	prm.fall_ms = 150;
	prm.tilt_deg = 45;
	prm.t_lim = 0;
	prm.t_lo = 2;
	prm.t_hi = 8;
	prm.alarm = 1;

	now_ts = 0;
	now_up = 0;
	last_up_ok = false;
	alarm_flag = false;
	trip_dirty = false;
	in_shock = false;
	shock_below = 0;
	fall_n = 0;
	blk_x = blk_y = blk_z = blk_m = blk_m2 = 0.0f;
	blk_n = 0;
	ref_ok = false;
	tilt_cur = NAN;
	orient = 0;
	t_state = 0;
	win_reset();
	trip_reset();
}

void shipment_get_params(struct shp_params *p)
{
	*p = prm;
}

void shipment_set_params(const struct shp_params *p)
{
	prm.shock_mg = clampi(p->shock_mg, 1000, 15000);
	prm.fall_ms = clampi(p->fall_ms, 50, 2000);
	prm.tilt_deg = clampi(p->tilt_deg, 10, 170);
	prm.t_lim = p->t_lim ? 1 : 0;
	prm.t_lo = clampi(p->t_lo, -40, 85);
	prm.t_hi = clampi(p->t_hi, -40, 85);
	if (prm.t_hi <= prm.t_lo) {
		prm.t_hi = prm.t_lo + 1;
	}
	prm.alarm = p->alarm ? 1 : 0;
}

bool shipment_apply_json(const char *json)
{
	struct shp_params p = prm;
	int32_t v;
	bool any = false;

	if (json_get_int(json, "shock_mg", &v)) {
		p.shock_mg = v;
		any = true;
	}
	if (json_get_int(json, "fall_ms", &v)) {
		p.fall_ms = v;
		any = true;
	}
	if (json_get_int(json, "tilt", &v)) {
		p.tilt_deg = v;
		any = true;
	}
	if (json_get_int(json, "t_lim", &v)) {
		p.t_lim = v;
		any = true;
	}
	if (json_get_int(json, "t_lo", &v)) {
		p.t_lo = v;
		any = true;
	}
	if (json_get_int(json, "t_hi", &v)) {
		p.t_hi = v;
		any = true;
	}
	if (json_get_int(json, "alarm", &v)) {
		p.alarm = v;
		any = true;
	}
	if (!any) {
		return false;
	}
	struct shp_params old = prm;

	shipment_set_params(&p);
	return memcmp(&old, &prm, sizeof(prm)) != 0;
}

void shipment_clock(int64_t unix_s, uint32_t up_s)
{
	now_ts = unix_s > 0 ? unix_s : 0;
	now_up = up_s;

	if (trip.active) {
		if (last_up_ok && up_s >= last_up) {
			trip.dur_s += up_s - last_up;
		}
		/* Start before network time: back-date once the clock is known
		 * (same boot only — start_up is from this boot then).
		 */
		if (trip.start_ts == 0 && now_ts > 0 && trip.resets == 0 &&
		    up_s >= trip.start_up) {
			trip.start_ts = now_ts - (int64_t)(up_s - trip.start_up);
			trip_dirty = true;
		}
	}
	last_up = up_s;
	last_up_ok = true;
}

static void emit(uint8_t type, float val)
{
	if (win.n_evt < SHP_WIN_EVT_MAX) {
		struct shp_evt *e = &win.evt[win.n_evt++];

		e->ts = now_ts;
		e->up_s = now_up;
		e->type = type;
		e->val = val;
	} else if (win.evt_lost < UINT16_MAX) {
		win.evt_lost++;
	}
	if (type != SHP_EV_T_OK && prm.alarm) {
		alarm_flag = true;
	}
	if (!trip.active) {
		return;
	}
	switch (type) {
	case SHP_EV_SHOCK:
		trip.shocks++;
		if (val > trip.shock_max) {
			trip.shock_max = val;
		}
		break;
	case SHP_EV_FALL:
		trip.falls++;
		if (val > trip.fall_max_ms) {
			trip.fall_max_ms = (uint16_t)val;
		}
		break;
	case SHP_EV_TILT:
		trip.tilts++;
		break;
	case SHP_EV_FLIP:
		trip.flips++;
		break;
	case SHP_EV_T_HI:
	case SHP_EV_T_LO:
		trip.t_viol++;
		break;
	default:
		break;
	}
	trip_dirty = true;
}

static void tilt_block(void)
{
	float n = (float)blk_n;
	float mx = blk_x / n, my = blk_y / n, mz = blk_z / n;
	float mm = blk_m / n;
	float var = blk_m2 / n - mm * mm;
	float sd = var > 0.0f ? sqrtf(var) : 0.0f;

	if (sd >= STILL_SD_G || fabsf(mm - 1.0f) >= STILL_MAG_G) {
		return; /* moving — no orientation reading this second */
	}
	float len = sqrtf(mx * mx + my * my + mz * mz);

	if (len < 0.5f) {
		return;
	}
	float d[3] = { mx / len, my / len, mz / len };

	if (!ref_ok) {
		memcpy(ref, d, sizeof(ref));
		ref_ok = true;
		if (trip.active) {
			memcpy(trip.ref, d, sizeof(trip.ref));
			trip.ref_ok = 1;
			trip_dirty = true;
		}
	}
	float dot = d[0] * ref[0] + d[1] * ref[1] + d[2] * ref[2];

	dot = dot > 1.0f ? 1.0f : (dot < -1.0f ? -1.0f : dot);
	float ang = acosf(dot) * RAD2DEG;

	tilt_cur = ang;
	win.tilt = ang;
	if (isnan(win.tilt_max) || ang > win.tilt_max) {
		win.tilt_max = ang;
	}
	if (trip.active && (isnan(trip.tilt_max) || ang > trip.tilt_max)) {
		trip.tilt_max = ang;
		trip_dirty = true;
	}

	float lim = (float)prm.tilt_deg;

	switch (orient) {
	case 0:
		if (ang > SHP_FLIP_DEG) {
			orient = 2;
			emit(SHP_EV_FLIP, ang);
		} else if (ang > lim) {
			orient = 1;
			emit(SHP_EV_TILT, ang);
		}
		break;
	case 1:
		if (ang > SHP_FLIP_DEG) {
			orient = 2;
			emit(SHP_EV_FLIP, ang);
		} else if (ang < lim - SHP_TILT_HYST_DEG) {
			orient = 0;
		}
		break;
	default:
		if (ang < SHP_FLIP_DEG - SHP_TILT_HYST_DEG) {
			orient = ang > lim ? 1 : 0;
		}
		break;
	}
}

void shipment_sample(float gx, float gy, float gz)
{
	float m = sqrtf(gx * gx + gy * gy + gz * gz);
	float th = (float)prm.shock_mg / 1000.0f;

	win.valid = true;
	win.n++;

	/* shock episode */
	if (m > th) {
		if (!in_shock || m > shock_peak) {
			shock_peak = m;
		}
		in_shock = true;
		shock_below = 0;
	} else if (in_shock && ++shock_below >= SHOCK_GAP_N) {
		in_shock = false;
		win.shocks++;
		if (shock_peak > win.shock_max) {
			win.shock_max = shock_peak;
		}
		emit(SHP_EV_SHOCK, shock_peak);
	}

	/* free fall */
	if (m < SHP_FALL_G) {
		fall_n++;
	} else {
		uint32_t ms = fall_n * MS_PER_SAMPLE;

		if (fall_n && ms >= (uint32_t)prm.fall_ms) {
			win.falls++;
			if (ms > win.fall_max_ms) {
				win.fall_max_ms = ms > UINT16_MAX ? UINT16_MAX : (uint16_t)ms;
			}
			emit(SHP_EV_FALL, (float)ms);
		}
		fall_n = 0;
	}

	/* orientation, 1-s blocks */
	blk_x += gx;
	blk_y += gy;
	blk_z += gz;
	blk_m += m;
	blk_m2 += m * m;
	if (++blk_n >= SHP_HZ) {
		tilt_block();
		blk_x = blk_y = blk_z = blk_m = blk_m2 = 0.0f;
		blk_n = 0;
	}
}

void shipment_temp(float t_c, uint32_t dt_ms)
{
	if (isnan(t_c)) {
		return;
	}
	if (isnan(win.t_min) || t_c < win.t_min) {
		win.t_min = t_c;
	}
	if (isnan(win.t_max) || t_c > win.t_max) {
		win.t_max = t_c;
	}
	if (trip.active) {
		if (isnan(trip.t_min) || t_c < trip.t_min) {
			trip.t_min = t_c;
		}
		if (isnan(trip.t_max) || t_c > trip.t_max) {
			trip.t_max = t_c;
		}
	}
	if (!prm.t_lim) {
		t_state = 0;
		return;
	}
	/* Time out of range counts the interval that led to this reading. */
	if (t_state != 0) {
		t_out_ms_win += dt_ms;
		if (trip.active) {
			t_out_ms_trip += dt_ms;
			trip.t_out_s += t_out_ms_trip / 1000;
			t_out_ms_trip %= 1000;
		}
	}
	float lo = (float)prm.t_lo, hi = (float)prm.t_hi;
	int8_t ns = t_state;

	if (t_c > hi) {
		ns = 1;
	} else if (t_c < lo) {
		ns = -1;
	} else if ((t_state == 1 && t_c < hi - SHP_T_HYST_C) ||
		   (t_state == -1 && t_c > lo + SHP_T_HYST_C)) {
		ns = 0;
	}
	if (ns != t_state) {
		t_state = ns;
		emit(ns > 0 ? SHP_EV_T_HI : (ns < 0 ? SHP_EV_T_LO : SHP_EV_T_OK), t_c);
	}
}

bool shipment_alarm(void)
{
	return alarm_flag;
}

void shipment_window(struct shp_win *w)
{
	win.tilt = tilt_cur;
	win.orient = orient;
	win.t_state = t_state;
	win.t_out_s = t_out_ms_win / 1000;
	*w = win;
	win_reset();
	alarm_flag = false;
}

void shipment_trip_start(void)
{
	trip_reset();
	trip.active = 1;
	trip.start_ts = now_ts;
	trip.start_up = now_up;
	last_up = now_up;
	last_up_ok = true;
	/* New "this side up": the next still second becomes the reference. */
	ref_ok = false;
	tilt_cur = NAN;
	orient = 0;
	t_state = 0;
	trip_dirty = true;
}

void shipment_trip_stop(void)
{
	if (!trip.active) {
		return;
	}
	trip.active = 0;
	trip.stop_ts = now_ts;
	trip.stop_up = now_up;
	trip_dirty = true;
}

void shipment_trip_get(struct shp_trip *t)
{
	*t = trip;
}

void shipment_trip_restore(const struct shp_trip *t)
{
	if (t->magic != SHP_TRIP_MAGIC) {
		return;
	}
	trip = *t;
	t_out_ms_trip = 0;
	if (!trip.active) {
		return;
	}
	if (trip.resets < UINT16_MAX) {
		trip.resets++;
	}
	if (trip.ref_ok) {
		memcpy(ref, trip.ref, sizeof(ref));
		ref_ok = true;
	}
	last_up_ok = false; /* uptime restarted: continue dur_s from the next clock */
	trip_dirty = true;
}

bool shipment_trip_dirty(void)
{
	bool d = trip_dirty;

	trip_dirty = false;
	return d;
}

bool shipment_trip_ok(const struct shp_trip *t)
{
	return t->shocks == 0 && t->falls == 0 && t->flips == 0 &&
	       t->t_viol == 0 && t->t_out_s == 0;
}
