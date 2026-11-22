/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Versor shipment events — pure C (no Zephyr), host-testable
 * (tools/shipment_test.c).
 *
 * Input: accel frames in g at SHP_HZ (BMI323 FIFO, +/-16 g) and the product
 * temperature (Pt1000 T1/AIN0) every few seconds. Output per telemetry window
 * and per trip (the "Bescheinigung"):
 *
 *   shock   |a| > shock threshold; one event per impact (bounces within
 *           SHP_SHOCK_GAP_MS merged), value = peak g (>= 16 = clipped)
 *   fall    |a| < SHP_FALL_G for >= fall_ms; value = duration ms
 *           (drop height h = g t^2 / 2: 150 ms ~ 11 cm, 300 ms ~ 44 cm)
 *   tilt    still gravity vector vs. the reference > tilt deg; value = angle
 *   flip    same > SHP_FLIP_DEG (upside down); value = angle
 *   t_hi / t_lo / t_ok   product temperature leaves / re-enters [t_lo, t_hi]
 *           (only with t_lim = 1), value = temperature C
 *
 * The tilt reference is the first still orientation after boot or after
 * `trip start` — i.e. "this side up" is however the parcel stood when the
 * trip was started.
 */
#ifndef SHIPMENT_H_
#define SHIPMENT_H_

#include <stdbool.h>
#include <stdint.h>

#define SHP_HZ            100   /* accel sample rate the caller feeds      */
#define SHP_SHOCK_GAP_MS  50    /* below threshold this long ends a shock  */
#define SHP_FALL_G        0.35f /* |a| below this = free fall              */
#define SHP_FLIP_DEG      135.0f
#define SHP_TILT_HYST_DEG 10.0f
#define SHP_T_HYST_C      0.5f
#define SHP_WIN_EVT_MAX   8     /* events carried per telemetry record     */

enum shp_evt_type {
	SHP_EV_SHOCK = 0,
	SHP_EV_FALL,
	SHP_EV_TILT,
	SHP_EV_FLIP,
	SHP_EV_T_HI,
	SHP_EV_T_LO,
	SHP_EV_T_OK,
	SHP_EV_COUNT,
};

struct shp_evt {
	int64_t ts;     /* unix s, 0 = network time not known yet */
	uint32_t up_s;  /* uptime s (always valid)                */
	uint8_t type;   /* enum shp_evt_type                      */
	float val;
};

/* Thresholds (cfg keys in brackets, all integers). */
struct shp_params {
	int32_t shock_mg; /* "shock_mg"  default 3000 (3 g), 1000..15000      */
	int32_t fall_ms;  /* "fall_ms"   default 150, 50..2000                 */
	int32_t tilt_deg; /* "tilt"      default 45, 10..170                   */
	int32_t t_lim;    /* "t_lim"     0 = temperature limits off (default)  */
	int32_t t_lo;     /* "t_lo"      C, default 2                          */
	int32_t t_hi;     /* "t_hi"      C, default 8                          */
	int32_t alarm;    /* "alarm"     1 = an event closes the window early  */
};

/* Summary of one telemetry window (reset by shipment_window()). */
struct shp_win {
	bool valid;         /* accel samples arrived (FIFO ran)            */
	uint32_t n;         /* accel samples in the window                 */
	uint16_t shocks;
	float shock_max;    /* g, 0 = none                                 */
	uint16_t falls;
	uint16_t fall_max_ms;
	float tilt;         /* deg, current (last still second), NAN = unknown */
	float tilt_max;     /* deg, max over still seconds, NAN = unknown  */
	uint8_t orient;     /* 0 upright, 1 tilted, 2 upside down          */
	float t_min, t_max; /* product temperature C, NAN = no reading     */
	uint32_t t_out_s;   /* seconds outside [t_lo, t_hi]                */
	int8_t t_state;     /* -1 low, 0 ok, +1 high                       */
	uint8_t n_evt;
	uint16_t evt_lost;  /* events beyond SHP_WIN_EVT_MAX               */
	struct shp_evt evt[SHP_WIN_EVT_MAX];
};

/* Whole-trip summary; plain data so it can be stored as one settings blob. */
#define SHP_TRIP_MAGIC 0x56535231u /* "VSR1" */
struct shp_trip {
	uint32_t magic;
	uint8_t active;
	int64_t start_ts, stop_ts;   /* unix s, 0 = unknown            */
	uint32_t start_up, stop_up;  /* uptime s at start/stop          */
	uint32_t dur_s;              /* sampled duration (s)            */
	uint16_t shocks, falls, tilts, flips, t_viol;
	float shock_max;
	uint16_t fall_max_ms;
	float tilt_max;              /* NAN = no still reading yet       */
	float t_min, t_max;          /* NAN = no reading                 */
	uint32_t t_out_s;
	uint16_t resets;             /* device restarts during the trip  */
	uint8_t ref_ok;              /* tilt reference survives a reset  */
	float ref[3];
};

void shipment_init(void);
void shipment_get_params(struct shp_params *p);
void shipment_set_params(const struct shp_params *p); /* clamped */
/* Apply flat cfg keys (see struct shp_params). True if anything changed. */
bool shipment_apply_json(const char *json);

/* Clock for event stamps; call once per sample slice. */
void shipment_clock(int64_t unix_s, uint32_t up_s);
void shipment_sample(float gx, float gy, float gz);
/* Product temperature reading, dt_ms since the previous call. NAN = skip. */
void shipment_temp(float t_c, uint32_t dt_ms);

/* An event worth sending at once happened in this window (and alarm = 1). */
bool shipment_alarm(void);
void shipment_window(struct shp_win *w);

void shipment_trip_start(void);
void shipment_trip_stop(void);
void shipment_trip_get(struct shp_trip *t);
/* Restore a stored trip after a reset (counts the restart). */
void shipment_trip_restore(const struct shp_trip *t);
/* True once after the trip changed (store it). */
bool shipment_trip_dirty(void);
/* No violation at all: no shock, fall, flip and no time out of range. */
bool shipment_trip_ok(const struct shp_trip *t);

const char *shipment_evt_name(uint8_t type);

#endif /* SHIPMENT_H_ */
