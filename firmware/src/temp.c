/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * 3x Pt1000 temperature bring-up (ratiometric), final schematic:
 *   P0.07 drives the divider top rail (V_TEMP) high (~1.8 V) only during a read
 *   (pulsed -> no self-heating). AIN3/P0.16 measures the actual top voltage,
 *   AIN0/1/2 (P0.13/14/15) measure the three divider mid-nodes:
 *      top --[ R_series 1 kOhm 0.1% (R200..202) ]-- node(AINx) --[ Pt1000 ]-- GND
 *
 * Pt1000 = platinum RTD, POSITIVE tempco: R0 = 1000 Ohm at 0 C, ~1078 Ohm at
 * 20 C. Heating a sensor -> its node voltage RISES (opposite to an NTC).
 * Temperature from Callendar-Van Dusen (valid >= 0 C; slight error below 0 C).
 *
 * The C values are BEST-EFFORT / UNCALIBRATED — for bring-up trust the mV:
 * touch/heat one sensor and watch its channel rise. Real calibration = bath,
 * Testplan S-1. Divider constants confirmed against the netlist (R200-202 =
 * 1 kOhm 0.1 %, IC200-202 = Pt1000 IST P1K0.232.6W).
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

#include "bringup.h"

LOG_MODULE_REGISTER(temp, LOG_LEVEL_INF);

/* Divider / Pt1000 constants */
#define RSERIES_OHM   1000.0    /* R200..R202, 0.1 %                        */
#define PT1000_R0     1000.0    /* Pt1000 resistance at 0 C                 */
/* Callendar-Van Dusen coefficients (IEC 60751, Pt platinum) */
#define CVD_A         3.9083e-3
#define CVD_B        (-5.775e-7)

/* Optional 1-point field calibration for temp_in: (reference_true - reading), in
 * C, added to the final value. The ratiometric divider already cancels the ADC's
 * ref/gain, so the residual absolute error is the Pt1000 element tolerance
 * (~+/-0.3 C, class B) — a single reference reading removes most of it. Leave 0
 * until such a reading is taken, then set this and reflash.
 */
#define TEMP_IN_OFFSET_C 0.0

/* Plausible temp_in band; anything outside is rejected as open/short/rail. With
 * gain 1/2 an OPEN Pt1000 rails the node at ~1.2 V (< V_TEMP), which would slip
 * past the v_node<v_top guard and read ~256 C, so this band is the real guard.
 */
#define TEMP_IN_MIN_C  (-50.0)
#define TEMP_IN_MAX_C  (150.0)

/* Channel indices into ch[] (see io-channels in the overlay). The schematic nets
 * (shematic-R1A-ordered.pdf, Sensors sheet) route IC200->T1->AIN0, IC201->T2->
 * AIN1, IC202->T3->AIN2, V_TEMP->AIN3. "temp_in" is the Pt1000 IC201 = T2 = AIN1.
 */
#define TEMP_IN_IDX   1   /* IC201 / T2 / AIN0+1 -> AIN1 (P0.14) */
#define VTEMP_IDX     3   /* V_TEMP excitation sense (AIN3 / P0.16) */

/* SAADC hardware oversampling for the high-res read: 2^8 = 256 samples averaged
 * in hardware per conversion (the schematic asks for "Oversampling 16-256x").
 */
#define TEMP_OVERSAMPLE 8

static const struct gpio_dt_spec exc =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), temp_exc_gpios);

static const struct adc_dt_spec ch[4] = {
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0), /* T1 (COW)         */
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1), /* T2 (AMBIENT)     */
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2), /* T3 (AMBIENT_SUN) */
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3), /* V_TEMP top       */
};

static const char *const label[3] = { "T1 COW    ", "T2 AMBIENT", "T3 SUN    " };

static bool inited;

/* Pt1000 resistance -> temperature via Callendar-Van Dusen (T >= 0 C branch). */
static double pt1000_temp_c(double r_ohm)
{
	double disc = CVD_A * CVD_A - 4.0 * CVD_B * (1.0 - r_ohm / PT1000_R0);

	if (disc < 0.0) {
		return NAN;
	}
	return (-CVD_A + sqrt(disc)) / (2.0 * CVD_B);
}

static int temp_setup(void)
{
	if (inited) {
		return 0;
	}
	if (!gpio_is_ready_dt(&exc)) {
		LOG_ERR("Pt1000 excitation GPIO (P0.07) not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&exc, GPIO_OUTPUT_INACTIVE);

	for (int i = 0; i < 4; i++) {
		if (!adc_is_ready_dt(&ch[i])) {
			LOG_ERR("SAADC channel %d not ready", i);
			return -ENODEV;
		}
		int err = adc_channel_setup_dt(&ch[i]);

		if (err) {
			LOG_ERR("adc_channel_setup_dt(%d): %d", i, err);
			return err;
		}
	}
	inited = true;
	return 0;
}

static int read_mv(const struct adc_dt_spec *spec, int32_t *mv)
{
	int16_t raw;
	struct adc_sequence seq = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};
	int err = adc_sequence_init_dt(spec, &seq);

	if (err) {
		return err;
	}
	err = adc_read_dt(spec, &seq);
	if (err) {
		return err;
	}
	*mv = raw;
	return adc_raw_to_millivolts_dt(spec, mv);
}

int bringup_temp_read(void)
{
	int32_t mv[4];
	int err = temp_setup();

	if (err) {
		return err;
	}

	/* Pulse the excitation rail, let the 100 nF nodes settle (>=450 us per the
	 * schematic; 2 ms is comfortable), then sample all four channels.
	 */
	gpio_pin_set_dt(&exc, 1);
	k_msleep(2);

	for (int i = 0; i < 4; i++) {
		err = read_mv(&ch[i], &mv[i]);
		if (err) {
			gpio_pin_set_dt(&exc, 0);
			LOG_ERR("SAADC read ch%d failed: %d", i, err);
			return err;
		}
	}
	gpio_pin_set_dt(&exc, 0);

	int32_t vtop = mv[3];

	LOG_INF("Pt1000 read  (V_TEMP top = %d mV):", vtop);

	if (vtop < 200) {
		LOG_ERR("  V_TEMP top only %d mV -> P0.07 excitation not reaching "
			"the divider (check drive/VDD_1V8)", vtop);
		return -EIO;
	}

	for (int i = 0; i < 3; i++) {
		int32_t vn = mv[i];

		if (vn > 0 && vn < vtop) {
			double rpt = RSERIES_OHM * (double)vn / (double)(vtop - vn);
			double t_c = pt1000_temp_c(rpt);

			LOG_INF("  %s  %4d mV  Rpt=%6.1f ohm  ~%.1f C (uncal.)",
				label[i], vn, rpt, t_c);
		} else {
			LOG_WRN("  %s  %4d mV  out of range (open Pt1000 / no divider?)",
				label[i], vn);
		}
	}
	LOG_INF("  (Pt1000: heating a sensor RAISES its mV; C uncalibrated - Testplan S-1)");
	return 0;
}

/* One oversampled SAADC read (256x HW average) -> millivolts, like read_mv() but
 * with the oversampling + optional offset calibration the schematic asks for.
 */
static int read_mv_hires(const struct adc_dt_spec *spec, int32_t *mv, bool calibrate)
{
	int16_t raw;
	struct adc_sequence seq = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};
	int err = adc_sequence_init_dt(spec, &seq);

	if (err) {
		return err;
	}
	/* Override after init_dt (which copies the channel's dt oversampling = 0). */
	seq.oversampling = TEMP_OVERSAMPLE;
	seq.calibrate = calibrate;

	err = adc_read_dt(spec, &seq);
	if (err) {
		return err;
	}
	*mv = raw;
	return adc_raw_to_millivolts_dt(spec, mv);
}

/* High-resolution ratiometric read of temp_in (IC201 = T2 = AIN1). Pulse the
 * excitation, let the node settle (tau = (Rseries||Rpt)*C ~ 50 us; >=9 tau), then
 * sample the sensor node AND the V_TEMP top under the SAME pulse so Vref, Vexc and
 * the GPIO Ron all cancel in the ratio. Resolution comes from three stacked tricks:
 *   1) the node channel runs at gain 1/2 (FS 1.2 V) so a code is worth ~0.18 C,
 *      half of the 0.37 C at gain 1/4 (the overlay sets this; V_TEMP stays 1/4);
 *   2) 256x HW oversampling + an offset calibration on the first conversion;
 *   3) the caller averages ~120 of these over the 30 s send window.
 */
bool bringup_temp_in_read(double *temp_c)
{
	int32_t v_node = 0, v_top = 0;

	if (temp_setup() != 0) {
		return false;
	}

	gpio_pin_set_dt(&exc, 1);
	k_msleep(2); /* >= 450 us settle; 2 ms also lets the SAADC ref settle */

	int e1 = read_mv_hires(&ch[TEMP_IN_IDX], &v_node, true);  /* + calibrate */
	int e2 = read_mv_hires(&ch[VTEMP_IDX], &v_top, false);

	gpio_pin_set_dt(&exc, 0);

	if (e1 || e2) {
		return false;
	}
	/* Guard: excitation must reach the divider, and the node must sit between
	 * GND and V_TEMP. An open / unfitted Pt1000 pulls the node up to ~V_TEMP.
	 */
	if (v_top < 200 || v_node <= 0 || v_node >= v_top) {
		return false;
	}

	double rpt = RSERIES_OHM * (double)v_node / (double)(v_top - v_node);
	double t = pt1000_temp_c(rpt);

	if (isnan(t) || t < TEMP_IN_MIN_C || t > TEMP_IN_MAX_C) {
		return false; /* open / short / railed node -> reject */
	}
	t += TEMP_IN_OFFSET_C;
	if (temp_c) {
		*temp_c = t;
	}
	return true;
}

/* Node + top voltage -> temperature with the same guards as temp_in. */
static bool node_to_temp(int32_t v_node, int32_t v_top, double *t_out)
{
	if (v_top < 200 || v_node <= 0 || v_node >= v_top) {
		return false;
	}
	double rpt = RSERIES_OHM * (double)v_node / (double)(v_top - v_node);
	double t = pt1000_temp_c(rpt);

	if (isnan(t) || t < TEMP_IN_MIN_C || t > TEMP_IN_MAX_C) {
		return false;
	}
	*t_out = t;
	return true;
}

/* All three Pt1000 under one excitation pulse: calibrate once, oversample each
 * node, then the V_TEMP top (same ratiometric method as bringup_temp_in_read).
 * Per-channel field offset: TEMP_IN_OFFSET_C applies to the AMBIENT channel
 * only (the one it was introduced for); the others are uncalibrated (the
 * Testplan relies on trends, A-2 checks the spread once in ice water).
 *
 * Below 0 C the quadratic Callendar-Van Dusen branch is used as well; the
 * missing C-term is < 0.01 K down to -40 C, so freezer tests are fine.
 */
void bringup_temp_read_all(double temp_c[PT_COUNT], bool valid[PT_COUNT])
{
	int32_t v_node[PT_COUNT] = { 0 };
	int32_t v_top = 0;
	int err = 0;

	for (int i = 0; i < PT_COUNT; i++) {
		valid[i] = false;
	}
	if (temp_setup() != 0) {
		return;
	}

	gpio_pin_set_dt(&exc, 1);
	k_msleep(2);
	for (int i = 0; i < PT_COUNT; i++) {
		err |= read_mv_hires(&ch[i], &v_node[i], i == 0);
	}
	err |= read_mv_hires(&ch[VTEMP_IDX], &v_top, false);
	gpio_pin_set_dt(&exc, 0);

	if (err) {
		return;
	}
	for (int i = 0; i < PT_COUNT; i++) {
		double t;

		if (node_to_temp(v_node[i], v_top, &t)) {
			if (i == TEMP_IN_IDX) {
				t += TEMP_IN_OFFSET_C;
			}
			temp_c[i] = t;
			valid[i] = true;
		}
	}
}
