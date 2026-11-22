/*
 * Host test for src/shipment.c (no hardware, no Zephyr).
 *
 *   cc -std=c11 -O2 -Wall -Wextra -Isrc tools/shipment_test.c src/shipment.c \
 *      -lm -o build_host/shipment_test && build_host/shipment_test
 */
#include "shipment.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Same behaviour as devcfg.c (300, "300", true/false). */
bool json_get_int(const char *json, const char *key, int32_t *out)
{
	char pat[40];

	snprintf(pat, sizeof(pat), "\"%s\"", key);
	const char *p = strstr(json, pat);

	if (!p) {
		return false;
	}
	p = strchr(p + strlen(pat), ':');
	if (!p) {
		return false;
	}
	p++;
	while (*p == ' ') {
		p++;
	}
	if (*p == '"') {
		p++;
	}
	if (!strncmp(p, "true", 4)) {
		*out = 1;
		return true;
	}
	if (!strncmp(p, "false", 5)) {
		*out = 0;
		return true;
	}
	char *end;
	long v = strtol(p, &end, 10);

	if (end == p) {
		return false;
	}
	*out = (int32_t)v;
	return true;
}

static int fails, checks;

#define CHECK(c, ...)                                                          \
	do {                                                                   \
		checks++;                                                      \
		if (!(c)) {                                                    \
			fails++;                                               \
			printf("FAIL %s:%d: ", __FILE__, __LINE__);            \
			printf(__VA_ARGS__);                                   \
			printf("\n");                                          \
		}                                                              \
	} while (0)

static uint32_t up_ms;
static int64_t unix0;
static uint32_t rng = 12345;

static float noise(void)
{
	rng = rng * 1103515245u + 12345u;
	return ((float)((rng >> 16) & 0x7fff) / 32767.0f - 0.5f) * 0.02f; /* +-0.01 g */
}

/* Feed n samples of a constant vector (+noise), clock every 100 ms. */
static void feed(float x, float y, float z, int n)
{
	for (int i = 0; i < n; i++) {
		if (up_ms % 100 == 0) {
			shipment_clock(unix0 ? unix0 + up_ms / 1000 : 0, up_ms / 1000);
		}
		shipment_sample(x + noise(), y + noise(), z + noise());
		up_ms += 10;
	}
}

/* Rest at an angle (deg) from +z, rotated about x. */
static void rest_angle(float deg, int n)
{
	float r = deg / 57.29578f;

	feed(0.0f, sinf(r), cosf(r), n);
}

static int count(const struct shp_win *w, uint8_t type)
{
	int c = 0;

	for (int i = 0; i < w->n_evt; i++) {
		c += w->evt[i].type == type;
	}
	return c;
}

static void reset(void)
{
	shipment_init();
	up_ms = 0;
	unix0 = 1763800000; /* 2025-11-22, any fixed value */
}

static void test_rest(void)
{
	struct shp_win w;

	reset();
	rest_angle(0, 500);
	shipment_window(&w);
	CHECK(w.valid && w.n == 500, "n=%u", w.n);
	CHECK(w.n_evt == 0, "rest: %u events", w.n_evt);
	CHECK(!isnan(w.tilt) && w.tilt < 3.0f, "tilt %.1f", w.tilt);
	CHECK(!shipment_alarm(), "alarm after window reset");
}

static void test_shock(void)
{
	struct shp_win w;

	reset();
	rest_angle(0, 200);
	feed(0, 0, 5.0f, 2);
	feed(0, 0, 7.5f, 1);
	rest_angle(0, 100);
	CHECK(shipment_alarm(), "shock should raise alarm");
	shipment_window(&w);
	CHECK(w.shocks == 1 && count(&w, SHP_EV_SHOCK) == 1, "shocks=%u", w.shocks);
	CHECK(fabsf(w.shock_max - 7.5f) < 0.05f, "peak %.2f", w.shock_max);

	/* bounce within 50 ms = one shock; after > 50 ms = two */
	feed(0, 0, 6, 2);
	rest_angle(0, 3);
	feed(0, 0, 4, 2);
	rest_angle(0, 100);
	feed(0, 0, 4, 2);
	rest_angle(0, 100);
	shipment_window(&w);
	CHECK(w.shocks == 2, "bounce merge: shocks=%u", w.shocks);

	/* 2.5 g stays below the 3 g default */
	feed(0, 0, 2.5f, 5);
	rest_angle(0, 100);
	shipment_window(&w);
	CHECK(w.shocks == 0, "2.5 g counted");

	/* clipped at full scale */
	feed(16.0f, 0, 0, 3);
	rest_angle(0, 100);
	shipment_window(&w);
	CHECK(w.shock_max >= 15.9f, "clip peak %.1f", w.shock_max);
}

static void test_fall(void)
{
	struct shp_win w;

	reset();
	rest_angle(0, 200);
	feed(0, 0, 0.02f, 25); /* 250 ms free fall ~ 31 cm */
	feed(0, 0, 9.0f, 2);   /* impact */
	rest_angle(0, 100);
	shipment_window(&w);
	CHECK(w.falls == 1 && w.fall_max_ms == 250, "falls=%u ms=%u", w.falls,
	      w.fall_max_ms);
	CHECK(w.shocks == 1, "impact shock=%u", w.shocks);

	feed(0, 0, 0.02f, 10); /* 100 ms < 150 ms */
	rest_angle(0, 100);
	shipment_window(&w);
	CHECK(w.falls == 0, "short fall counted");
}

static void test_tilt(void)
{
	struct shp_win w;

	reset();
	rest_angle(0, 300);
	shipment_window(&w);
	CHECK(w.n_evt == 0, "upright events");

	/* carried (moving) at 90 deg: no reading while moving */
	for (int i = 0; i < 100; i++) {
		feed(0, 1.0f + 0.3f * sinf(i * 0.7f), 0, 1);
	}
	shipment_window(&w);
	CHECK(count(&w, SHP_EV_TILT) == 0, "tilt while moving");

	rest_angle(60, 300);
	shipment_window(&w);
	CHECK(count(&w, SHP_EV_TILT) == 1 && w.orient == 1, "tilt 60: n=%d o=%u",
	      count(&w, SHP_EV_TILT), w.orient);
	CHECK(fabsf(w.tilt - 60) < 3, "tilt %.1f", w.tilt);

	rest_angle(40, 300); /* inside hysteresis (45-10) -> stays tilted */
	shipment_window(&w);
	CHECK(w.orient == 1 && w.n_evt == 0, "hyst o=%u", w.orient);

	rest_angle(180, 300);
	shipment_window(&w);
	CHECK(count(&w, SHP_EV_FLIP) == 1 && w.orient == 2, "flip");

	rest_angle(0, 300);
	shipment_window(&w);
	CHECK(w.orient == 0 && w.n_evt == 0, "back upright o=%u", w.orient);
}

static void test_temp(void)
{
	struct shp_win w;
	struct shp_trip t;

	reset();
	CHECK(shipment_apply_json("{\"t_lim\":1,\"t_lo\":2,\"t_hi\":8}"), "cfg");
	shipment_trip_start();
	shipment_temp(5.0f, 10000);
	shipment_temp(9.0f, 10000);  /* -> high */
	shipment_temp(9.5f, 10000);  /* +10 s out */
	shipment_temp(7.8f, 10000);  /* +10 s, still high (hysteresis) */
	shipment_temp(7.4f, 10000);  /* +10 s, -> ok */
	shipment_temp(1.0f, 10000);  /* -> low */
	shipment_temp(NAN, 10000);   /* skipped */
	shipment_window(&w);
	CHECK(count(&w, SHP_EV_T_HI) == 1 && count(&w, SHP_EV_T_OK) == 1 &&
	      count(&w, SHP_EV_T_LO) == 1, "temp events %u", w.n_evt);
	CHECK(w.t_out_s == 30, "t_out %u", w.t_out_s);
	CHECK(w.t_min == 1.0f && w.t_max == 9.5f, "min/max");
	CHECK(w.t_state == -1, "state %d", w.t_state);
	shipment_trip_get(&t);
	CHECK(t.t_viol == 2 && t.t_out_s == 30 && !shipment_trip_ok(&t),
	      "trip viol=%u out=%u", t.t_viol, t.t_out_s);

	/* limits off: no events */
	reset();
	shipment_temp(30.0f, 1000);
	shipment_window(&w);
	CHECK(w.n_evt == 0 && w.t_max == 30.0f, "t_lim off");
}

static void test_trip(void)
{
	struct shp_win w;
	struct shp_trip t, saved;

	reset();
	unix0 = 0; /* no network time yet */
	rest_angle(0, 100);
	shipment_trip_start();
	CHECK(shipment_trip_dirty() && !shipment_trip_dirty(), "dirty once");
	rest_angle(0, 1000); /* 10 s */
	unix0 = 1763800000;
	rest_angle(0, 1000);
	shipment_trip_get(&t);
	CHECK(t.active && t.start_ts == 1763800000, "back-dated start %lld",
	      (long long)t.start_ts);
	CHECK(t.dur_s >= 19 && t.dur_s <= 20, "dur %u", t.dur_s);
	CHECK(shipment_trip_ok(&t) && t.ref_ok, "clean trip");

	feed(0, 0, 0.02f, 30);
	feed(0, 0, 8, 2);
	rest_angle(0, 100);
	shipment_trip_get(&saved);
	CHECK(saved.shocks == 1 && saved.falls == 1 && !shipment_trip_ok(&saved),
	      "trip counts");

	/* reset: restore, reference kept, tilt still detected */
	shipment_init();
	up_ms = 0;
	shipment_trip_restore(&saved);
	shipment_trip_get(&t);
	CHECK(t.active && t.resets == 1 && t.shocks == 1, "restore");
	rest_angle(170, 300);
	shipment_window(&w);
	CHECK(count(&w, SHP_EV_FLIP) == 1, "flip vs restored reference");
	shipment_trip_stop();
	shipment_trip_get(&t);
	CHECK(!t.active && t.stop_ts > 0 && t.flips == 1, "stop");
	CHECK(t.dur_s >= 22 && t.dur_s <= 24, "dur after reset %u", t.dur_s);

	/* events outside a trip don't touch it */
	feed(0, 0, 9, 2);
	rest_angle(170, 100);
	shipment_trip_get(&saved);
	CHECK(saved.shocks == t.shocks, "shock after stop counted");

	struct shp_trip bad = { 0 };

	shipment_trip_restore(&bad); /* wrong magic ignored */
	shipment_trip_get(&saved);
	CHECK(saved.magic == SHP_TRIP_MAGIC, "bad magic");
}

static void test_overflow_cfg(void)
{
	struct shp_win w;
	struct shp_params p;

	reset();
	rest_angle(0, 100);
	for (int i = 0; i < 12; i++) {
		feed(0, 0, 5, 2);
		rest_angle(0, 20);
	}
	shipment_window(&w);
	CHECK(w.shocks == 12 && w.n_evt == SHP_WIN_EVT_MAX && w.evt_lost == 4,
	      "overflow shocks=%u n=%u lost=%u", w.shocks, w.n_evt, w.evt_lost);
	CHECK(w.evt[0].ts > 0, "event ts");

	CHECK(shipment_apply_json("{\"shock_mg\":\"500\",\"tilt\":300,\"alarm\":false}"),
	      "apply");
	shipment_get_params(&p);
	CHECK(p.shock_mg == 1000 && p.tilt_deg == 170 && p.alarm == 0,
	      "clamp %d %d %d", p.shock_mg, p.tilt_deg, p.alarm);
	CHECK(!shipment_apply_json("{\"shock_mg\":1000}"), "no change");
	CHECK(!shipment_apply_json("{\"int\":60}"), "foreign key");
	feed(0, 0, 5, 2);
	rest_angle(0, 20);
	CHECK(!shipment_alarm(), "alarm off");
	CHECK(!strcmp(shipment_evt_name(SHP_EV_FLIP), "flip"), "name");
}

int main(void)
{
	test_rest();
	test_shock();
	test_fall();
	test_tilt();
	test_temp();
	test_trip();
	test_overflow_cfg();
	printf("%d/%d checks passed\n", checks - fails, checks);
	return fails ? 1 : 0;
}
