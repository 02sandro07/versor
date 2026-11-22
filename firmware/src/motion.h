/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Motion features (docs/PLAN-ERKENNUNG.md, Phase 1): BMI323 samples at 50 Hz ->
 * per 2-s epoch gravity direction, dynamic acceleration, band energies and a
 * behaviour class; per telemetry window seconds per class, steps, posture
 * changes, gait features of walking phases. Plus the raw-data burst buffer.
 *
 * Pure C (no Zephyr), so tools/motion_test.c compiles it on the host.
 * Called from the main thread only (sampling loop and MQTT callbacks both run
 * there), so no locking.
 */
#ifndef CDC_INCO_MOTION_H_
#define CDC_INCO_MOTION_H_

#include <stdbool.h>
#include <stdint.h>

#define MOTION_HZ        50
#define MOTION_EPOCH_N   (2 * MOTION_HZ)     /* 2-s epoch */
#define MOTION_RAW_MAX_S 60                  /* 60 s x 50 Hz x 3 x 2 B = 18 KB, 30 blocks */

enum motion_class {
	MC_GRAZE = 0, /* grasen / fressen: head down, moving                */
	MC_RUM,       /* wiederkauen: head not down, rhythmic 1-1.5 Hz      */
	MC_WALK,      /* gehen                                              */
	MC_RUN,       /* laufen (traben, galopp)                            */
	MC_REST,      /* ruhen: nearly no dynamic acceleration              */
	MC_OTHER,
	MC_COUNT,
};

/* Thresholds, settable from the retained config (flat keys, integers). */
struct motion_params {
	int16_t still_mg;  /* "m_still": dyn. sd below -> rest            (15)  */
	int16_t walk_mg;   /* "m_walk":  step-band rms above -> walk      (40)  */
	int16_t run_mg;    /* "m_run":   step-band rms above -> run       (250) */
	int16_t chew_mg;   /* "m_chew":  chew-band rms above -> rumination (6)  */
	int16_t down_deg;  /* "m_down":  pitch below rest -> head down    (25)  */
	int16_t lat_deg;   /* "m_lat":   roll off rest -> lateral lying   (60)  */
	/* Rest position + pitch axis learned by the server (Phase 3.1); 0 = not set.
	 * "g0x/g0y/g0z" gravity at rest in mg, "axx/axy/axz" pitch axis x1000.
	 */
	int16_t g0[3];
	int16_t ax[3];
	int16_t raw_every_min; /* "raw_every": raw burst every N min, 0 = off   */
};

/* Per telemetry window. */
struct motion_out {
	bool valid;              /* at least one epoch                          */
	uint16_t sec[MC_COUNT];  /* seconds per class                           */
	uint16_t sec_lat;        /* seconds in lateral lying (feature, no class)*/
	uint16_t steps;
	uint16_t pos_chg;        /* posture changes between still epochs        */
	uint16_t still_max;      /* longest rest bout, s                        */
	float acc_peak;          /* max |dynamic acceleration|, m/s^2           */
	float pitch, roll;       /* device angles of gravity, window mean, deg  */
	float pitch_sd;          /* sd of the epoch pitch, deg                  */
	/* gait: mean over walking phases with >= 6 steps */
	uint16_t gait_n;         /* number of such phases                       */
	float gait_f;            /* step frequency, Hz                          */
	float gait_reg;          /* regularity 0..1 (1 - CV of step intervals)  */
	float gait_asym;         /* |amp n - amp n+1| / mean amp (head bob)     */
	float gait_bob;          /* mean nod amplitude, m/s^2                   */
};

void motion_init(void);
void motion_defaults(struct motion_params *p);
void motion_get_params(struct motion_params *p);
void motion_set_params(const struct motion_params *p);
/* Apply flat cfg keys (m_still, ..., g0x, ..., axz); others ignored. True if changed. */
bool motion_apply_json(const char *json);

/* One accelerometer sample in g (device frame), at MOTION_HZ. */
void motion_sample(float gx, float gy, float gz);
/* Close the window: fill out and start a new one. */
void motion_window(struct motion_out *out);
/* Class of the last finished epoch (debug / shell). */
enum motion_class motion_last_class(void);
const char *motion_class_name(enum motion_class c);

/* Raw-data burst: record the next s seconds of samples (mg, int16). */
void motion_raw_start(int s);
bool motion_raw_recording(void);
/* True once a recording is complete and not yet fully sent. */
bool motion_raw_ready(void);
/* Burst start (unix s, set by the caller via motion_raw_set_start), blocks of
 * MOTION_EPOCH_N samples: n blocks, block k has *len samples at d (x,y,z...).
 */
void motion_raw_set_start(int64_t unix_s);
int64_t motion_raw_start_ts(void);
int  motion_raw_blocks(void);
const int16_t *motion_raw_block(int k, int *len);
/* Block k is sent (PUBACK); the burst is freed when all are. */
void motion_raw_sent(int k);
bool motion_raw_block_sent(int k);
void motion_raw_discard(void);

/* Implemented elsewhere (devcfg.c on target, the test on the host). */
bool json_get_int(const char *json, const char *key, int32_t *out);

#endif /* CDC_INCO_MOTION_H_ */
