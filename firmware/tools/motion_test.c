/*
 * Host test for src/motion.c (no Zephyr):
 *   cc -std=c11 -O2 -Wall -Wextra -Isrc tools/motion_test.c src/motion.c -lm -o build_host/motion_test
 *   ./build_host/motion_test
 *
 * Synthetic collar signals at 50 Hz (device frame, head up = gravity on +z):
 * rest, grazing (head down 40 deg), rumination (1.2 Hz, 12 mg), walking
 * (1 Hz steps), running (2 Hz), lateral lying, lame walking (alternating step
 * amplitude). Checks the dominant class, step counts, posture changes and that
 * the lame gait has a clearly higher asymmetry than the normal one.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion.h"

/* Same flat parser as devcfg.c (only what motion_apply_json needs). */
bool json_get_int(const char *json, const char *key, int32_t *out)
{
	char pat[24];

	snprintf(pat, sizeof(pat), "\"%s\"", key);
	const char *p = strstr(json, pat);

	if (!p || !(p = strchr(p + strlen(pat), ':'))) {
		return false;
	}
	char *end;
	long v = strtol(p + 1, &end, 10);

	if (end == p + 1) {
		return false;
	}
	*out = (int32_t)v;
	return true;
}

static unsigned rng = 12345;
static float noise(float sd)                /* ~gaussian, Irwin-Hall */
{
	float s = 0;

	for (int i = 0; i < 12; i++) {
		rng = rng * 1103515245u + 12345u;
		s += (float)((rng >> 8) & 0xFFFF) / 65535.0f;
	}
	return (s - 6.0f) * sd;
}

#define PI 3.14159265f
enum scen { REST, GRAZE, RUM, WALK, RUN, LAT, LAME };
static const char *const sname[] = { "rest", "graze", "rum", "walk", "run", "lateral", "lame" };

static void gen(enum scen s, int seconds)
{
	for (int i = 0; i < seconds * MOTION_HZ; i++) {
		float t = (float)i / MOTION_HZ;
		float g[3] = { 0, 0, 1 }, d[3] = { 0, 0, 0 };
		float n = 0.003f;

		switch (s) {
		case GRAZE: {
			float p = 40.0f * PI / 180.0f;

			g[0] = sinf(p);
			g[2] = cosf(p);
			n = 0.030f;
			break;
		}
		case RUM:
			d[0] = 0.012f * sinf(2 * PI * 1.2f * t);
			n = 0.002f;
			break;
		case WALK:
			d[2] = 0.12f * sinf(2 * PI * 1.0f * t);
			n = 0.010f;
			break;
		case LAME: {
			int step = (int)(t * 1.0f);
			float amp = (step % 2) ? 0.06f : 0.18f;

			d[2] = amp * sinf(2 * PI * 1.0f * t);
			n = 0.010f;
			break;
		}
		case RUN:
			d[2] = 0.6f * sinf(2 * PI * 2.0f * t);
			n = 0.050f;
			break;
		case LAT:
			g[1] = 1;
			g[2] = 0;
			break;
		default:
			break;
		}
		motion_sample(g[0] + d[0] + noise(n), g[1] + d[1] + noise(n), g[2] + d[2] + noise(n));
	}
}

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static struct motion_out run_scen(enum scen s, int seconds)
{
	struct motion_out o;

	gen(s, seconds);
	motion_window(&o);
	printf("%-8s graze %3u rum %3u walk %3u run %3u rest %3u other %3u | lat %3u steps %3u pos %u still_max %3u "
	       "peak %.2f pitch %+6.1f roll %+6.1f sd %4.1f | gait n %u f %.2f reg %.2f asym %.2f bob %.2f\n",
	       sname[s], o.sec[MC_GRAZE], o.sec[MC_RUM], o.sec[MC_WALK], o.sec[MC_RUN], o.sec[MC_REST],
	       o.sec[MC_OTHER], o.sec_lat, o.steps, o.pos_chg, o.still_max, (double)o.acc_peak, (double)o.pitch,
	       (double)o.roll, (double)o.pitch_sd, o.gait_n, (double)o.gait_f, (double)o.gait_reg,
	       (double)o.gait_asym, (double)o.gait_bob);
	return o;
}

static int total(const struct motion_out *o)
{
	int t = 0;

	for (int i = 0; i < MC_COUNT; i++) {
		t += o->sec[i];
	}
	return t;
}

int main(void)
{
	motion_init();

	/* warm-up so the low pass settles, then each scenario 60 s + 4 s settle */
	gen(REST, 10);
	struct motion_out o;

	motion_window(&o);

	o = run_scen(REST, 60);
	CHECK(o.sec[MC_REST] >= 56, "rest: %u s rest", o.sec[MC_REST]);
	CHECK(o.steps == 0, "rest: %u steps", o.steps);

	gen(GRAZE, 4);
	motion_window(&o);
	o = run_scen(GRAZE, 60);
	CHECK(o.sec[MC_GRAZE] >= 50, "graze: %u s graze", o.sec[MC_GRAZE]);

	gen(REST, 6);
	motion_window(&o);
	o = run_scen(RUM, 60);
	CHECK(o.sec[MC_RUM] >= 50, "rum: %u s rum", o.sec[MC_RUM]);

	gen(WALK, 4);
	motion_window(&o);
	o = run_scen(WALK, 60);
	CHECK(o.sec[MC_WALK] >= 54, "walk: %u s walk", o.sec[MC_WALK]);
	CHECK(o.steps >= 55 && o.steps <= 62, "walk: %u steps (60 expected)", o.steps);
	struct motion_out walk = o;

	gen(REST, 6);           /* closes the walking phase -> gait values */
	motion_window(&o);
	CHECK(o.gait_n == 1, "walk: gait phases %u", o.gait_n);
	CHECK(fabsf(o.gait_f - 1.0f) < 0.05f, "walk: step frequency %.2f", (double)o.gait_f);
	CHECK(o.gait_reg > 0.9f, "walk: regularity %.2f", (double)o.gait_reg);
	float asym_ok = o.gait_asym;

	o = run_scen(RUN, 60);
	CHECK(o.sec[MC_RUN] >= 54, "run: %u s run", o.sec[MC_RUN]);
	CHECK(o.steps >= 110 && o.steps <= 124, "run: %u steps (120 expected)", o.steps);

	gen(REST, 10);
	motion_window(&o);
	o = run_scen(LAT, 60);
	CHECK(o.sec_lat >= 56, "lateral: %u s lateral", o.sec_lat);
	CHECK(o.sec[MC_REST] >= 54, "lateral: %u s rest", o.sec[MC_REST]);
	CHECK(o.pos_chg >= 1, "lateral: posture change %u", o.pos_chg);

	gen(REST, 10);
	motion_window(&o);
	run_scen(LAME, 60);
	gen(REST, 6);
	motion_window(&o);
	CHECK(o.gait_n == 1, "lame: gait phases %u", o.gait_n);
	CHECK(o.gait_asym > 3 * asym_ok && o.gait_asym > 0.3f, "lame: asym %.2f vs normal %.2f",
	      (double)o.gait_asym, (double)asym_ok);

	/* every window accounts for its whole duration */
	CHECK(total(&walk) == 60, "walk window %d s", total(&walk));

	/* server rest position + pitch axis: head down is +pitch about y */
	CHECK(motion_apply_json("{\"mode\":\"normal\",\"int\":60,\"g0x\":0,\"g0y\":0,\"g0z\":1000,"
				"\"axx\":0,\"axy\":1000,\"axz\":0,\"m_down\":20}"), "apply json");
	struct motion_params p;

	motion_get_params(&p);
	CHECK(p.g0[2] == 1000 && p.ax[1] == 1000 && p.down_deg == 20, "params %d %d %d", p.g0[2], p.ax[1], p.down_deg);
	CHECK(!motion_apply_json("{\"m_down\":20}"), "unchanged json reported as change");
	gen(GRAZE, 4);
	motion_window(&o);
	o = run_scen(GRAZE, 60);
	CHECK(o.sec[MC_GRAZE] >= 50, "graze with g0: %u s graze", o.sec[MC_GRAZE]);

	/* raw burst: 3 s -> 2 blocks (100 + 50 samples) */
	motion_raw_start(3);
	gen(WALK, 4);
	CHECK(motion_raw_ready(), "raw not ready");
	CHECK(motion_raw_blocks() == 2, "raw blocks %d", motion_raw_blocks());
	int len;
	const int16_t *b = motion_raw_block(1, &len);

	CHECK(b && len == 50, "raw block 1 len %d", len);
	motion_raw_sent(0);
	CHECK(motion_raw_ready(), "raw freed too early");
	motion_raw_sent(1);
	CHECK(!motion_raw_ready(), "raw not freed");

	printf(fails ? "\n%d FAILED\n" : "\nall motion tests passed\n", fails);
	return fails ? 1 : 0;
}
