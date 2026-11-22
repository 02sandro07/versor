/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * nPM1300 (Power board, U500): battery measurement, charge supervision, state
 * of charge and the three LEDs. The static configuration (rails, charger, NTC
 * JEITA thresholds, LED modes) lives in the devicetree overlay; this file adds
 * what the hardware cannot do on its own:
 *
 *   Charge temperature window (JOB 2026-09-29):
 *     < 0 C      no charging      -> nPM1300 hardware (NTC "cold")
 *     0..10 C    reduced current  -> hardware halves ISET in the "cool" region
 *     10..35 C   full 800 mA
 *     35..45 C   reduced current  -> hardware only lowers VTERM in "warm", so
 *                                    the FIRMWARE halves ISET here
 *     > 45 C     no charging      -> hardware (NTC "hot")
 *   Belt and braces: the firmware also disables the charger itself outside
 *   0..45 C (with 1 K hysteresis), so a wrong NTC setting cannot silently
 *   charge a frozen cell.
 *
 *   State of charge: until a measured battery model of the PL405070 exists
 *   (nPM PowerUP profiling), soc is estimated from the open-circuit voltage
 *   (LiPo OCV table, IR-compensated, smoothed). Rough (+/-10 %), but capacity-
 *   independent. Drop a profiled `battery_model.inc` into src/ and enable
 *   CONFIG_NRF_FUEL_GAUGE to switch to Nordic's fuel gauge instead.
 *
 * Absent PMIC (CU board bring-up without the Power board): every device here is
 * "not ready", pmic_init() returns -ENODEV and the rest of the firmware sends
 * null for the power fields.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/drivers/mfd/npm13xx.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/regulator.h>
#include <math.h>
#include <string.h>

#include "bringup.h"

#if defined(CONFIG_NRF_FUEL_GAUGE) && __has_include("battery_model.inc")
#include <nrf_fuel_gauge.h>
#define USE_FUEL_GAUGE 1
#endif

LOG_MODULE_REGISTER(pmic, LOG_LEVEL_INF);

#define PMIC_NODE DT_NODELABEL(npm1300_pmic)

#if DT_NODE_EXISTS(PMIC_NODE)

static const struct device *const mfd = DEVICE_DT_GET(PMIC_NODE);
static const struct device *const charger = DEVICE_DT_GET(DT_NODELABEL(npm1300_charger));
static const struct device *const leds = DEVICE_DT_GET(DT_NODELABEL(npm1300_leds));
static const struct device *const regulators =
	DEVICE_DT_GET(DT_NODELABEL(npm1300_regulators));

/* Charger register block (nPM1300 PS, "CHARGER") */
#define CHGR_BASE        0x03U
#define CHGR_EN_SET      0x04U
#define CHGR_EN_CLR      0x05U
#define CHGR_ISET        0x08U /* MSB = ISET[8:1], next reg LSB = ISET[0] */
#define CHGR_ERR_CLR     0x00U

/* BCHGCHARGESTATUS bits */
#define ST_BATTERY       BIT(0)
#define ST_COMPLETED     BIT(1)
#define ST_TRICKLE       BIT(2)
#define ST_CC            BIT(3)
#define ST_CV            BIT(4)

/* Charge currents. FULL must match current-microamp in the overlay (the driver
 * scales its current reading with that value).
 */
#define ICHG_FULL_MA     800
#define ICHG_REDUCED_MA  400

/* Firmware charge-temperature window with hysteresis (C). */
#define T_CHG_MIN        0.0f
#define T_CHG_MAX        45.0f
#define T_WARM           35.0f
#define T_HYST           1.0f

/* Internal resistance used to correct the loaded voltage towards OCV. */
#define R_INT_OHM        0.15f

static bool present;
static bool fw_chg_disabled;
static bool reduced;
static uint8_t led_mask;
static struct pmic_status last;
static K_MUTEX_DEFINE(pm_lock);

#if defined(USE_FUEL_GAUGE)
static const struct battery_model battery_model = {
#include "battery_model.inc"
};
static bool fg_ready;
static int64_t fg_ref_time;
#endif

/* LiPo open-circuit voltage -> SoC (typical curve, 0.2 C, 25 C). */
static const struct { float v; float soc; } ocv_table[] = {
	{ 3.27f,   0 }, { 3.61f,   5 }, { 3.69f,  10 }, { 3.71f,  15 },
	{ 3.73f,  20 }, { 3.75f,  25 }, { 3.77f,  30 }, { 3.79f,  35 },
	{ 3.80f,  40 }, { 3.82f,  45 }, { 3.84f,  50 }, { 3.85f,  55 },
	{ 3.87f,  60 }, { 3.91f,  65 }, { 3.95f,  70 }, { 3.98f,  75 },
	{ 4.02f,  80 }, { 4.08f,  85 }, { 4.11f,  90 }, { 4.15f,  95 },
	{ 4.20f, 100 },
};

static float soc_from_ocv(float v)
{
	if (v <= ocv_table[0].v) {
		return 0.0f;
	}
	for (int i = 1; i < ARRAY_SIZE(ocv_table); i++) {
		if (v <= ocv_table[i].v) {
			float f = (v - ocv_table[i - 1].v) /
				  (ocv_table[i].v - ocv_table[i - 1].v);

			return ocv_table[i - 1].soc +
			       f * (ocv_table[i].soc - ocv_table[i - 1].soc);
		}
	}
	return 100.0f;
}

static float sv_to_f(const struct sensor_value *v)
{
	return (float)sensor_value_to_double(v);
}

static int set_iset_ma(int ma)
{
	uint16_t idx = (uint16_t)(ma / 2); /* 2 mA steps */
	int err;

	/* Change ISET with the charger paused, then resume. */
	err = mfd_npm13xx_reg_write(mfd, CHGR_BASE, CHGR_EN_CLR, 1U);
	err |= mfd_npm13xx_reg_write2(mfd, CHGR_BASE, CHGR_ISET, idx / 2U, idx & 1U);
	if (!fw_chg_disabled) {
		err |= mfd_npm13xx_reg_write(mfd, CHGR_BASE, CHGR_ERR_CLR, 1U);
		err |= mfd_npm13xx_reg_write(mfd, CHGR_BASE, CHGR_EN_SET, 1U);
	}
	return err ? -EIO : 0;
}

static void charger_enable(bool en)
{
	struct sensor_value v = { .val1 = en ? 1 : 0 };

	(void)sensor_attr_set(charger, SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT,
			      SENSOR_ATTR_CONFIGURATION, &v);
}

/* Firmware part of the charge-temperature window (see file header). */
static void supervise_charging(float t_bat)
{
	if (!fw_chg_disabled &&
	    (t_bat < T_CHG_MIN || t_bat >= T_CHG_MAX)) {
		fw_chg_disabled = true;
		charger_enable(false);
		LOG_WRN("Charging disabled by firmware: t_bat %.1f C outside %.0f..%.0f C",
			(double)t_bat, (double)T_CHG_MIN, (double)T_CHG_MAX);
	} else if (fw_chg_disabled &&
		   t_bat >= T_CHG_MIN + T_HYST && t_bat < T_CHG_MAX - T_HYST) {
		fw_chg_disabled = false;
		charger_enable(true);
		LOG_INF("Charging re-enabled: t_bat %.1f C", (double)t_bat);
	}

	if (!reduced && t_bat >= T_WARM) {
		if (set_iset_ma(ICHG_REDUCED_MA) == 0) {
			reduced = true;
			LOG_INF("t_bat %.1f C >= %.0f C -> charge current %d mA",
				(double)t_bat, (double)T_WARM, ICHG_REDUCED_MA);
		}
	} else if (reduced && t_bat < T_WARM - T_HYST) {
		if (set_iset_ma(ICHG_FULL_MA) == 0) {
			reduced = false;
			LOG_INF("t_bat %.1f C -> charge current back to %d mA",
				(double)t_bat, ICHG_FULL_MA);
		}
	}
}

#if defined(USE_FUEL_GAUGE)
static void fg_init(float v, float i_ma, float t)
{
	struct nrf_fuel_gauge_init_parameters p = {
		.model = &battery_model,
		.v0 = v,
		.i0 = -i_ma / 1000.0f, /* library: + = discharging */
		.t0 = t,
	};

	if (nrf_fuel_gauge_init(&p, NULL) == 0) {
		fg_ready = true;
		fg_ref_time = k_uptime_get();
		LOG_INF("nRF Fuel Gauge %s active (profiled battery model)",
			nrf_fuel_gauge_version);
	}
}
#endif

int pmic_init(void)
{
	if (!device_is_ready(mfd) || !device_is_ready(charger)) {
		LOG_WRN("nPM1300 not present (Power board not attached?) — "
			"vbat/soc/chg will be null");
		present = false;
		return -ENODEV;
	}
	present = true;
	LOG_INF("nPM1300 ready: charger %d mA, NTC window 0/10/35/45 C, LEDs host",
		ICHG_FULL_MA);

	if (!device_is_ready(leds)) {
		LOG_WRN("nPM1300 LED driver not ready");
	}
	pmic_leds(0);
	(void)pmic_update(NULL);
	return 0;
}

bool pmic_present(void)
{
	return present;
}

int pmic_update(struct pmic_status *out)
{
	struct pmic_status s = { 0 };
	struct sensor_value v;
	int err;

	if (!present) {
		if (out) {
			memset(out, 0, sizeof(*out));
		}
		return -ENODEV;
	}

	err = sensor_sample_fetch(charger);
	if (err) {
		LOG_WRN("charger fetch failed: %d", err);
		return err;
	}

	s.present = true;
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &v);
	s.vbat = sv_to_f(&v);
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &v);
	s.ibat_ma = sv_to_f(&v) * 1000.0f;
	if (sensor_channel_get(charger, SENSOR_CHAN_GAUGE_TEMP, &v) == 0) {
		s.t_bat = sv_to_f(&v);
	} else {
		s.t_bat = NAN;
	}
	sensor_channel_get(charger, SENSOR_CHAN_DIE_TEMP, &v);
	s.t_die = sv_to_f(&v);
	sensor_channel_get(charger, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_STATUS, &v);
	s.chg_raw = (uint8_t)v.val1;
	sensor_channel_get(charger, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_ERROR, &v);
	s.chg_err = (uint8_t)v.val1;
	if (sensor_attr_get(charger,
			    (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS,
			    (enum sensor_attribute)SENSOR_ATTR_NPM13XX_CHARGER_VBUS_PRESENT,
			    &v) == 0) {
		s.vbus = v.val1 != 0;
	}

	if (s.chg_raw & ST_COMPLETED) {
		s.chg = 2;
	} else if (s.chg_raw & (ST_TRICKLE | ST_CC | ST_CV)) {
		s.chg = 1;
	} else {
		s.chg = 0;
	}

	k_mutex_lock(&pm_lock, K_FOREVER);

	if (!isnan(s.t_bat)) {
		supervise_charging(s.t_bat);
	}
	/* The driver scales the charge current with the overlay ISET (800 mA);
	 * while we run the reduced ISET the real current is proportionally lower.
	 */
	if (reduced && s.ibat_ma > 0.0f) {
		s.ibat_ma *= (float)ICHG_REDUCED_MA / (float)ICHG_FULL_MA;
	}
	s.reduced = reduced;

#if defined(USE_FUEL_GAUGE)
	if (!fg_ready) {
		fg_init(s.vbat, s.ibat_ma, isnan(s.t_bat) ? 20.0f : s.t_bat);
	}
	if (fg_ready) {
		float dt = (float)k_uptime_delta(&fg_ref_time) / 1000.0f;

		s.soc = nrf_fuel_gauge_process(s.vbat, -s.ibat_ma / 1000.0f,
					       isnan(s.t_bat) ? 20.0f : s.t_bat,
					       dt, NULL);
	}
#else
	{
		float ocv = s.vbat - (s.ibat_ma / 1000.0f) * R_INT_OHM;
		float soc = soc_from_ocv(ocv);

		if (s.chg == 2) {
			soc = 100.0f;
		}
		/* Smooth: LTE TX bursts sag the voltage for a moment. */
		if (last.present) {
			soc = last.soc + 0.2f * (soc - last.soc);
		}
		s.soc = CLAMP(soc, 0.0f, 100.0f);
	}
#endif

	last = s;
	k_mutex_unlock(&pm_lock);

	if (out) {
		*out = s;
	}
	return 0;
}

void pmic_get(struct pmic_status *s)
{
	k_mutex_lock(&pm_lock, K_FOREVER);
	*s = last;
	k_mutex_unlock(&pm_lock);
}

void pmic_leds(uint8_t mask)
{
	if (!present || !device_is_ready(leds)) {
		led_mask = mask;
		return;
	}
	for (int i = 0; i < 3; i++) {
		bool on = (mask & BIT(i)) != 0;

		if (on != ((led_mask & BIT(i)) != 0) || mask == 0) {
			(void)(on ? led_on(leds, i) : led_off(leds, i));
		}
	}
	led_mask = mask;
}

uint8_t pmic_leds_get(void)
{
	return led_mask;
}

int pmic_ship_mode(void)
{
	if (!present) {
		return -ENODEV;
	}
	LOG_WRN("Entering ship mode — only a Qi pad / VBUS wakes the device again");
	k_sleep(K_MSEC(200));
	return regulator_parent_ship_mode(regulators);
}

#else /* no PMIC node in the devicetree */

int pmic_init(void) { return -ENODEV; }
bool pmic_present(void) { return false; }
int pmic_update(struct pmic_status *s) { if (s) { memset(s, 0, sizeof(*s)); } return -ENODEV; }
void pmic_get(struct pmic_status *s) { memset(s, 0, sizeof(*s)); }
void pmic_leds(uint8_t mask) { ARG_UNUSED(mask); }
uint8_t pmic_leds_get(void) { return 0; }
int pmic_ship_mode(void) { return -ENODEV; }

#endif
