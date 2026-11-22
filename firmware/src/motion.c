/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Motion features, behaviour classes and gait (see motion.h).
 *
 * Per sample (50 Hz, g):
 *   gravity  g_lp  first-order low pass, tau 1 s
 *   dynamic  d = a - g_lp
 *   step band   biquad band pass 1.2 Hz (Q 0.8) on the vertical part of d
 *   chew band   biquad band pass 1.25 Hz (Q 2.5) on each axis of d
 * Per 2-s epoch: rms of d / step band / chew band, peak |d|, mean gravity ->
 * device pitch and roll, angles relative to the rest position (server g0 +
 * pitch axis if given, otherwise a slow average over rest epochs), class:
 *
 *   run    step rms >= m_run
 *   walk   step rms >= m_walk and at least one step in the epoch
 *   graze  head down (pitch below rest by m_down) and moving (dyn >= m_still)
 *   rum    chew rms >= m_chew, chew band carries >= half of the motion, slow
 *   rest   dyn rms < m_still
 *   other  everything else
 *
 * Lateral lying (roll off rest by m_lat) is counted as a feature, no class.
 * Steps: up-crossings of the step band with hysteresis (0.7 x m_walk).
 * Gait: per walking phase with >= 6 steps -> step frequency, regularity
 * (1 - CV of the step intervals), asymmetry |amp n - amp n+1| / mean amp
 * (a nodding head when lame), nod amplitude. These are first rule-based
 * versions; the thresholds come per retained config so they can be tuned
 * without FOTA (PLAN-ERKENNUNG 1.3).
 */
#include <math.h>
#include <string.h>

#include "motion.h"

#define G_MS2    9.80665f
#define RAD2DEG  57.29578f
#define LP_ALPHA (1.0f / (MOTION_HZ * 1.0f))   /* tau 1 s */
#define REF_ALPHA 0.05f                         /* rest reference, per rest epoch */
#define POS_CHG_DEG 30.0f
#define EPOCH_STEPS_MAX 16
#define PHASE_STEPS_MAX 256
#define GAIT_MIN_STEPS 6

struct biquad {
	float b0, b1, b2, a1, a2;
	float x1, x2, y1, y2;
};

static void bq_bandpass(struct biquad *q, float f0, float Q)
{
	float w0 = 2.0f * 3.14159265f * f0 / MOTION_HZ;
	float al = sinf(w0) / (2.0f * Q);
	float a0 = 1.0f + al;

	memset(q, 0, sizeof(*q));
	q->b0 = al / a0;
	q->b1 = 0.0f;
	q->b2 = -al / a0;
	q->a1 = -2.0f * cosf(w0) / a0;
	q->a2 = (1.0f - al) / a0;
}

static float bq_run(struct biquad *q, float x)
{
	float y = q->b0 * x + q->b1 * q->x1 + q->b2 * q->x2 - q->a1 * q->y1 - q->a2 * q->y2;

	q->x2 = q->x1;
	q->x1 = x;
	q->y2 = q->y1;
	q->y1 = y;
	return y;
}

static struct motion_params P;
static bool started;
static float g_lp[3];
static struct biquad bq_step, bq_chew[3];
static uint32_t sample_idx;

/* epoch */
static int e_n;
static float e_dyn2, e_step2, e_chew2, e_peak, e_g[3];
static uint32_t e_step_t[EPOCH_STEPS_MAX];
static float e_step_a[EPOCH_STEPS_MAX];
static int e_steps;
static enum motion_class last_class = MC_OTHER;

/* step detector */
static bool st_pos;
static float st_crest, st_trough;
static int st_pending = -1;     /* epoch index of the step whose amplitude is open */
static int ph_pending = -1;     /* same, once the step moved into the phase arrays */

/* rest reference (fallback without server g0) */
static bool ref_ok;
static float ref_g[3];
static bool still_ok;
static float still_g[3];
static int rest_run;

/* walking phase */
static uint32_t ph_t[PHASE_STEPS_MAX];
static float ph_a[PHASE_STEPS_MAX];
static int ph_n;

/* window */
static struct {
	int epochs;
	uint32_t sec[MC_COUNT];
	uint32_t sec_lat, steps, pos_chg, still_max;
	float peak;
	double p_sum, r_sum, p_mean, p_m2;
	int gait_n;
	double gf, greg, gasym, gbob;
} W;

void motion_defaults(struct motion_params *p)
{
	memset(p, 0, sizeof(*p));
	p->still_mg = 15;
	p->walk_mg = 40;
	p->run_mg = 250;
	p->chew_mg = 6;
	p->down_deg = 25;
	p->lat_deg = 60;
}

void motion_init(void)
{
	motion_defaults(&P);
	started = false;
	bq_bandpass(&bq_step, 1.2f, 0.8f);
	for (int i = 0; i < 3; i++) {
		bq_bandpass(&bq_chew[i], 1.25f, 2.5f);
	}
	sample_idx = 0;
	e_n = 0;
	e_steps = 0;
	ref_ok = still_ok = false;
	rest_run = 0;
	ph_n = 0;
	st_pos = false;
	st_crest = st_trough = 0.0f;
	st_pending = ph_pending = -1;
	memset(&W, 0, sizeof(W));
	motion_raw_discard();
}

void motion_get_params(struct motion_params *p)
{
	*p = P;
}

void motion_set_params(const struct motion_params *p)
{
	P = *p;
}

static bool get_i16(const char *json, const char *key, int16_t *v, int lo, int hi)
{
	int32_t x;

	if (!json_get_int(json, key, &x) || x < lo || x > hi || x == *v) {
		return false;
	}
	*v = (int16_t)x;
	return true;
}

bool motion_apply_json(const char *json)
{
	static const char *const gk[3] = { "g0x", "g0y", "g0z" };
	static const char *const ak[3] = { "axx", "axy", "axz" };
	bool ch = false;

	ch |= get_i16(json, "m_still", &P.still_mg, 1, 2000);
	ch |= get_i16(json, "m_walk", &P.walk_mg, 1, 4000);
	ch |= get_i16(json, "m_run", &P.run_mg, 1, 8000);
	ch |= get_i16(json, "m_chew", &P.chew_mg, 1, 2000);
	ch |= get_i16(json, "m_down", &P.down_deg, 1, 90);
	ch |= get_i16(json, "m_lat", &P.lat_deg, 1, 180);
	ch |= get_i16(json, "raw_every", &P.raw_every_min, 0, 1440);
	for (int i = 0; i < 3; i++) {
		ch |= get_i16(json, gk[i], &P.g0[i], -4000, 4000);
		ch |= get_i16(json, ak[i], &P.ax[i], -1000, 1000);
	}
	return ch;
}

/* ------------------------------ vectors ---------------------------------- */

static float norm3(const float v[3])
{
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static bool unit3(const float in[3], float out[3])
{
	float n = norm3(in);

	if (n < 1e-6f) {
		return false;
	}
	for (int i = 0; i < 3; i++) {
		out[i] = in[i] / n;
	}
	return true;
}

static float dot3(const float a[3], const float b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void cross3(const float a[3], const float b[3], float o[3])
{
	o[0] = a[1] * b[2] - a[2] * b[1];
	o[1] = a[2] * b[0] - a[0] * b[2];
	o[2] = a[0] * b[1] - a[1] * b[0];
}

static float angle_deg(const float a[3], const float b[3])
{
	float c = dot3(a, b);

	c = c > 1.0f ? 1.0f : c < -1.0f ? -1.0f : c;
	return acosf(c) * RAD2DEG;
}

/* Signed rotation (deg) from r to g around axis k (all unit vectors). */
static float rot_about(const float r[3], const float g[3], const float k[3])
{
	float c[3];

	cross3(r, g, c);
	return atan2f(dot3(c, k), dot3(r, g)) * RAD2DEG;
}

/* --------------------------------- gait ----------------------------------- */

static void phase_close(void)
{
	if (ph_n >= GAIT_MIN_STEPS) {
		int m = ph_n - 1;
		double di = 0, di2 = 0, am = 0, asy = 0;

		for (int i = 0; i < m; i++) {
			double d = (double)(ph_t[i + 1] - ph_t[i]);

			di += d;
			di2 += d * d;
		}
		di /= m;
		double sd = sqrt(fmax(0.0, di2 / m - di * di));

		int na = 0, np = 0;

		/* a step whose down-crossing never came has no amplitude: skip it */
		for (int i = 0; i < ph_n; i++) {
			if (ph_a[i] > 0.0f) {
				am += (double)ph_a[i];
				na++;
			}
		}
		am = na ? am / na : 0.0;
		for (int i = 0; i < m; i++) {
			if (ph_a[i] > 0.0f && ph_a[i + 1] > 0.0f) {
				asy += fabs((double)ph_a[i + 1] - (double)ph_a[i]);
				np++;
			}
		}
		asy = np ? asy / np : 0.0;

		W.gait_n++;
		W.gf += di > 0 ? MOTION_HZ / di : 0;
		W.greg += di > 0 ? fmax(0.0, 1.0 - sd / di) : 0;
		W.gasym += am > 1e-6 ? asy / am : 0;
		W.gbob += am / 2.0 * (double)G_MS2;
	}
	ph_n = 0;
	ph_pending = -1;
}

/* -------------------------------- epochs ---------------------------------- */

static void epoch_end(void)
{
	float n = (float)e_n;
	float dyn = sqrtf(e_dyn2 / n) * 1000.0f;   /* mg */
	float stp = sqrtf(e_step2 / n) * 1000.0f;
	float chw = sqrtf(e_chew2 / n) * 1000.0f;
	float gm[3] = { e_g[0] / n, e_g[1] / n, e_g[2] / n }, gn[3];

	if (!unit3(gm, gn)) {
		gn[0] = 0;
		gn[1] = 0;
		gn[2] = 1;
	}
	float pitch = atan2f(gm[0], sqrtf(gm[1] * gm[1] + gm[2] * gm[2])) * RAD2DEG;
	float roll = atan2f(gm[1], gm[2]) * RAD2DEG;

	/* angles relative to the rest position */
	float pr, rr;
	bool down, lat;
	float g0[3] = { P.g0[0], P.g0[1], P.g0[2] }, ax[3] = { P.ax[0], P.ax[1], P.ax[2] };
	float g0n[3], axn[3];

	if (unit3(g0, g0n) && unit3(ax, axn)) {
		float fwd[3], fwdn[3];

		cross3(axn, g0n, fwd);
		if (!unit3(fwd, fwdn)) {
			fwdn[0] = 1;
			fwdn[1] = 0;
			fwdn[2] = 0;
		}
		pr = rot_about(g0n, gn, axn);           /* + = head down (server sign) */
		rr = rot_about(g0n, gn, fwdn);
		down = pr >= P.down_deg;
		lat = fabsf(rr) >= P.lat_deg;
	} else {
		if (!ref_ok) {
			memcpy(ref_g, gn, sizeof(ref_g));
			ref_ok = true;
		}
		float rp = atan2f(ref_g[0], sqrtf(ref_g[1] * ref_g[1] + ref_g[2] * ref_g[2])) * RAD2DEG;
		float rl = atan2f(ref_g[1], ref_g[2]) * RAD2DEG;
		float dl = roll - rl;

		dl = dl > 180.0f ? dl - 360.0f : dl < -180.0f ? dl + 360.0f : dl;
		pr = pitch - rp;
		rr = dl;
		lat = fabsf(rr) >= P.lat_deg;
		down = !lat && angle_deg(gn, ref_g) >= P.down_deg;
	}

	enum motion_class c;

	if (stp >= P.run_mg) {
		c = MC_RUN;
	} else if (stp >= P.walk_mg && e_steps > 0) {
		c = MC_WALK;
	} else if (down && dyn >= P.still_mg) {
		c = MC_GRAZE;
	} else if (chw >= P.chew_mg && chw >= 0.5f * dyn && dyn < P.walk_mg) {
		c = MC_RUM;
	} else if (dyn < P.still_mg) {
		c = MC_REST;
	} else {
		c = MC_OTHER;
	}
	last_class = c;

	/* window bookkeeping */
	int sec = (e_n + MOTION_HZ / 2) / MOTION_HZ;

	W.epochs++;
	W.sec[c] += sec;
	if (lat) {
		W.sec_lat += sec;
	}
	if (e_peak > W.peak) {
		W.peak = e_peak;
	}
	W.p_sum += (double)pitch;
	W.r_sum += (double)roll;
	double dp = (double)pitch - W.p_mean;

	W.p_mean += dp / W.epochs;
	W.p_m2 += dp * ((double)pitch - W.p_mean);

	if (c == MC_REST) {
		rest_run += sec;
		if ((uint32_t)rest_run > W.still_max) {
			W.still_max = rest_run;
		}
		if (still_ok && angle_deg(gn, still_g) >= POS_CHG_DEG) {
			W.pos_chg++;
		}
		memcpy(still_g, gn, sizeof(still_g));
		still_ok = true;
		if (ref_ok && !lat) {
			float t[3];

			for (int i = 0; i < 3; i++) {
				t[i] = ref_g[i] + REF_ALPHA * (gn[i] - ref_g[i]);
			}
			(void)unit3(t, ref_g);
		}
	} else {
		rest_run = 0;
	}

	if (c == MC_WALK || c == MC_RUN) {
		W.steps += e_steps;
		for (int i = 0; i < e_steps && ph_n < PHASE_STEPS_MAX; i++) {
			if (i == st_pending) {
				ph_pending = ph_n;
			}
			ph_t[ph_n] = e_step_t[i];
			ph_a[ph_n] = e_step_a[i];
			ph_n++;
		}
	} else {
		phase_close();
	}

	e_n = 0;
	e_dyn2 = e_step2 = e_chew2 = e_peak = 0.0f;
	e_g[0] = e_g[1] = e_g[2] = 0.0f;
	e_steps = 0;
	st_pending = -1;     /* an open amplitude now lives in ph_a[ph_pending] */
}

/* ------------------------------- raw burst -------------------------------- */

static int16_t raw_buf[MOTION_RAW_MAX_S * MOTION_HZ * 3];
static int raw_want, raw_have;           /* samples */
static uint8_t raw_state;                /* 0 idle, 1 recording, 2 ready */
static uint32_t raw_sent_mask;
static int64_t raw_ts;

static int16_t to_mg(float g)
{
	float v = g * 1000.0f;

	return (int16_t)(v > 32767.0f ? 32767 : v < -32768.0f ? -32768 : lrintf(v));
}

void motion_raw_start(int s)
{
	if (s < 1) {
		s = 1;
	}
	if (s > MOTION_RAW_MAX_S) {
		s = MOTION_RAW_MAX_S;
	}
	raw_want = s * MOTION_HZ;
	raw_have = 0;
	raw_sent_mask = 0;
	raw_ts = 0;
	raw_state = 1;
}

bool motion_raw_recording(void)
{
	return raw_state == 1;
}

bool motion_raw_ready(void)
{
	return raw_state == 2;
}

void motion_raw_set_start(int64_t unix_s)
{
	raw_ts = unix_s;
}

int64_t motion_raw_start_ts(void)
{
	return raw_ts;
}

int motion_raw_blocks(void)
{
	return (raw_have + MOTION_EPOCH_N - 1) / MOTION_EPOCH_N;
}

const int16_t *motion_raw_block(int k, int *len)
{
	int n = motion_raw_blocks();

	if (raw_state != 2 || k < 0 || k >= n) {
		*len = 0;
		return NULL;
	}
	int first = k * MOTION_EPOCH_N;
	int cnt = raw_have - first < MOTION_EPOCH_N ? raw_have - first : MOTION_EPOCH_N;

	*len = cnt;
	return &raw_buf[first * 3];
}

/* At most MOTION_RAW_MAX_S * MOTION_HZ / MOTION_EPOCH_N = 30 blocks -> one mask word. */
void motion_raw_sent(int k)
{
	int n = motion_raw_blocks();

	if (k >= 0 && k < n) {
		raw_sent_mask |= 1u << k;
	}
	if (raw_sent_mask == (1u << n) - 1u) {
		raw_state = 0;
	}
}

bool motion_raw_block_sent(int k)
{
	return k >= 0 && k < 32 && (raw_sent_mask & (1u << k));
}

void motion_raw_discard(void)
{
	raw_state = 0;
	raw_have = raw_want = 0;
	raw_sent_mask = 0;
}

/* -------------------------------- samples --------------------------------- */

void motion_sample(float gx, float gy, float gz)
{
	float a[3] = { gx, gy, gz };

	if (raw_state == 1) {
		raw_buf[raw_have * 3 + 0] = to_mg(gx);
		raw_buf[raw_have * 3 + 1] = to_mg(gy);
		raw_buf[raw_have * 3 + 2] = to_mg(gz);
		if (++raw_have >= raw_want) {
			raw_state = 2;
		}
	}

	if (!started) {
		memcpy(g_lp, a, sizeof(g_lp));
		started = true;
	}
	for (int i = 0; i < 3; i++) {
		g_lp[i] += LP_ALPHA * (a[i] - g_lp[i]);
	}
	float d[3] = { a[0] - g_lp[0], a[1] - g_lp[1], a[2] - g_lp[2] };
	float gu[3];

	if (!unit3(g_lp, gu)) {
		gu[0] = 0;
		gu[1] = 0;
		gu[2] = 1;
	}
	float v = dot3(d, gu);             /* vertical part of the motion */
	float s = bq_run(&bq_step, v);
	float c2 = 0.0f;

	for (int i = 0; i < 3; i++) {
		float c = bq_run(&bq_chew[i], d[i]);

		c2 += c * c;
	}
	float dm2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
	float dm = sqrtf(dm2) * G_MS2;

	e_n++;
	e_dyn2 += dm2;
	e_step2 += s * s;
	e_chew2 += c2;
	if (dm > e_peak) {
		e_peak = dm;
	}
	for (int i = 0; i < 3; i++) {
		e_g[i] += g_lp[i];
	}

	/* steps: up-crossing of +h after the signal was below -h */
	float h = 0.7f * P.walk_mg / 1000.0f;

	if (!st_pos && s > h) {
		st_pos = true;
		st_crest = s;
		if (e_steps < EPOCH_STEPS_MAX) {
			e_step_t[e_steps] = sample_idx;
			e_step_a[e_steps] = 0.0f;
			st_pending = e_steps;
			ph_pending = -1;
			e_steps++;
		}
	} else if (st_pos) {
		if (s > st_crest) {
			st_crest = s;
		}
		if (s < -h) {
			st_pos = false;
			if (st_pending >= 0) {
				e_step_a[st_pending] = st_crest - st_trough;
				st_pending = -1;
			} else if (ph_pending >= 0) {
				ph_a[ph_pending] = st_crest - st_trough;
				ph_pending = -1;
			}
			st_trough = s;
		}
	} else if (s < st_trough) {
		st_trough = s;
	}
	sample_idx++;

	if (e_n >= MOTION_EPOCH_N) {
		epoch_end();
	}
}

void motion_window(struct motion_out *o)
{
	memset(o, 0, sizeof(*o));
	o->valid = W.epochs > 0;
	for (int i = 0; i < MC_COUNT; i++) {
		o->sec[i] = (uint16_t)(W.sec[i] > 65535 ? 65535 : W.sec[i]);
	}
	o->sec_lat = (uint16_t)W.sec_lat;
	o->steps = (uint16_t)(W.steps > 65535 ? 65535 : W.steps);
	o->pos_chg = (uint16_t)W.pos_chg;
	o->still_max = (uint16_t)(W.still_max > 65535 ? 65535 : W.still_max);
	o->acc_peak = W.peak;
	if (W.epochs) {
		o->pitch = (float)(W.p_sum / W.epochs);
		o->roll = (float)(W.r_sum / W.epochs);
		o->pitch_sd = W.epochs > 1 ? (float)sqrt(W.p_m2 / (W.epochs - 1)) : 0.0f;
	}
	o->gait_n = (uint16_t)W.gait_n;
	if (W.gait_n) {
		o->gait_f = (float)(W.gf / W.gait_n);
		o->gait_reg = (float)(W.greg / W.gait_n);
		o->gait_asym = (float)(W.gasym / W.gait_n);
		o->gait_bob = (float)(W.gbob / W.gait_n);
	}
	/* an ongoing rest bout and walking phase continue into the next window */
	memset(&W, 0, sizeof(W));
}

enum motion_class motion_last_class(void)
{
	return last_class;
}

const char *motion_class_name(enum motion_class c)
{
	static const char *const n[MC_COUNT] = { "graze", "rum", "walk", "run", "rest", "other" };

	return c < MC_COUNT ? n[c] : "?";
}
