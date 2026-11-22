/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * I2C bus scan + Bosch BMI323 IMU bring-up (register-level, no external sensor
 * driver, so the bench log shows exactly what the chip returns).
 *
 * Final schematic (shematic-R1A-ordered.pdf, Sensors sheet, IC203):
 *   I2C on TWIM2 P0.30 SDA / P0.31 SCL, BMI323 @0x68
 *   (CSB pin12 -> VDD_1V8 = I2C mode, SDO pin1 -> GND = addr 0x68),
 *   INT1 pin4 -> P0.05, INT2 pin9 -> P0.06.
 *
 * BMI323 I2C quirk: every read burst returns 2 dummy bytes BEFORE the data.
 * Registers are 16-bit, little-endian.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

#include "bringup.h"

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

#define BMI323_ADDR   0x68
#define NPM1300_ADDR  0x6B

/* BMI323 registers (16-bit) */
#define REG_CHIP_ID   0x00
#define REG_STATUS    0x02
#define REG_ACC_DATA  0x03  /* X, then Y (0x04), Z (0x05) */
#define REG_FIFO_FILL 0x15  /* fill level in 16-bit words, bits 10:0 */
#define REG_FIFO_DATA 0x16
#define REG_ACC_CONF  0x20
#define REG_FIFO_CONF 0x36  /* bit9 acc_en, bit0 stop_on_full */
#define REG_FIFO_CTRL 0x37  /* bit0 flush */
#define REG_IO_INT_CTRL 0x38
#define REG_INT_MAP2  0x3B
#define REG_CMD       0x7E

#define BMI323_CHIP_ID   0x43   /* low byte of reg 0x00 */
#define CMD_SOFTRESET    0xDEAF

/* ACC_CONF: acc_mode=normal(0x4)<<12 | acc_range=+/-4g(0x1)<<4 | acc_odr=100Hz(0x8) */
#define ACC_CONF_NORMAL_4G_100HZ  0x4018
/* Versor shipment logging: normal, +/-16 g (0x3<<4), 100 Hz — shocks from
 * drops reach 10+ g, +/-4 g would clip them.
 */
#define ACC_CONF_NORMAL_16G_100HZ 0x4038
/* FIFO with accel frames only (x, y, z = 3 words); oldest overwritten when full */
#define FIFO_CONF_ACC             0x0200
#define FIFO_DUMMY_X              0x7F01  /* frame without new data */
/* 16-bit: +/-4 g -> 8192 LSB/g, +/-16 g -> 2048 LSB/g */
#define LSB_PER_G_4G              8192.0f
#define LSB_PER_G_16G             2048.0f

static float lsb_per_g = LSB_PER_G_4G; /* follows the range last written */

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c2));
static bool fifo_on;  /* FIFO configured (a soft reset, e.g. `bu imu`, clears it) */

static const struct gpio_dt_spec int1 =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), imu_int1_gpios);

static bool i2c_ready(void)
{
	if (!device_is_ready(i2c)) {
		LOG_ERR("I2C bus (i2c2) not ready");
		return false;
	}
	return true;
}

/* BMI323 16-bit read: [reg] -> [dummy, dummy, LSB, MSB, ...]. */
static int bmi_read(uint8_t reg, uint16_t *vals, int n)
{
	uint8_t buf[2 + 2 * 4];

	if (n < 1 || n > 4) {
		return -EINVAL;
	}
	int err = i2c_write_read(i2c, BMI323_ADDR, &reg, 1, buf, 2 + 2 * n);

	if (err) {
		return err;
	}
	for (int i = 0; i < n; i++) {
		vals[i] = buf[2 + 2 * i] | (buf[3 + 2 * i] << 8);
	}
	return 0;
}

static int bmi_write(uint8_t reg, uint16_t val)
{
	uint8_t buf[3] = { reg, val & 0xFF, val >> 8 };

	return i2c_write(i2c, buf, sizeof(buf), BMI323_ADDR);
}

/* Zephyr's canonical bus-scan probe: a zero-length write; ACK == device there. */
static bool addr_present(uint16_t addr)
{
	uint8_t dummy = 0;
	struct i2c_msg msg = {
		.buf = &dummy,
		.len = 0U,
		.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
	};

	return i2c_transfer(i2c, &msg, 1, addr) == 0;
}

int bringup_i2c_scan(void)
{
	int found = 0;

	if (!i2c_ready()) {
		return -ENODEV;
	}

	LOG_INF("I2C scan on TWIM2 (P0.30 SDA / P0.31 SCL):");
	for (uint16_t a = 0x08; a <= 0x77; a++) {
		if (addr_present(a)) {
			const char *who =
				(a == BMI323_ADDR)  ? " (BMI323 IMU)" :
				(a == NPM1300_ADDR) ? " (nPM1300 PMIC)" : "";
			LOG_INF("  found 0x%02x%s", a, who);
			found++;
		}
	}
	LOG_INF("I2C scan done: %d device(s). Expected: 0x68 (BMI323). "
		"0x6B (nPM1300) only once the Energy board/PMIC is attached.",
		found);
	return found > 0 ? 0 : -ENODEV;
}

int bringup_pmic_id(void)
{
	if (!i2c_ready()) {
		return -ENODEV;
	}
	if (addr_present(NPM1300_ADDR)) {
		LOG_INF("nPM1300 ACKs at 0x6B -> PMIC/Energy board present");
		return 0;
	}
	LOG_WRN("nPM1300 (0x6B) not responding -> expected while PMIC is missing");
	return -ENODEV;
}

int bringup_imu_id(void)
{
	uint16_t id = 0;
	int err;

	if (!i2c_ready()) {
		return -ENODEV;
	}

	err = bmi_read(REG_CHIP_ID, &id, 1);
	if (err) {
		LOG_ERR("BMI323: no ACK at 0x68 (err %d). Check VDD_1V8, "
			"pull-ups, SDA/SCL wiring, CSB=1V8.", err);
		return err;
	}

	if ((id & 0xFF) == BMI323_CHIP_ID) {
		LOG_INF("BMI323 CHIP_ID = 0x%02x  OK (reg 0x00 = 0x%04x)",
			id & 0xFF, id);
		return 0;
	}

	LOG_ERR("BMI323 CHIP_ID = 0x%02x (expected 0x43) -> wrong part or bus fault",
		id & 0xFF);
	return -EIO;
}

int bringup_imu_init(void)
{
	int err;

	fifo_on = false;
	if (bringup_imu_id() != 0) {
		return -EIO;
	}

	err = bmi_write(REG_CMD, CMD_SOFTRESET);
	if (err) {
		LOG_ERR("BMI323 softreset failed: %d", err);
		return err;
	}
	k_msleep(5); /* datasheet: >=2 ms after soft reset */

	/* Re-check ID after reset (I2C mode must re-latch). */
	if (bringup_imu_id() != 0) {
		return -EIO;
	}

	err = bmi_write(REG_ACC_CONF, ACC_CONF_NORMAL_4G_100HZ);
	if (err) {
		LOG_ERR("BMI323 ACC_CONF write failed: %d", err);
		return err;
	}
	lsb_per_g = LSB_PER_G_4G;
	k_msleep(10); /* let the accelerometer power up */

	LOG_INF("BMI323 initialised: normal mode, +/-4 g, 100 Hz");
	return 0;
}

int bringup_imu_read(void)
{
	uint16_t d[3];
	int err = bmi_read(REG_ACC_DATA, d, 3);

	if (err) {
		LOG_ERR("BMI323 accel read failed: %d", err);
		return err;
	}

	int16_t raw[3];

	for (int i = 0; i < 3; i++) {
		raw[i] = (int16_t)d[i];
	}

	double gx = raw[0] / (double)lsb_per_g;
	double gy = raw[1] / (double)lsb_per_g;
	double gz = raw[2] / (double)lsb_per_g;
	double mag = sqrt(gx * gx + gy * gy + gz * gz);

	LOG_INF("BMI323  x=%+.3f g  y=%+.3f g  z=%+.3f g  |a|=%.3f g "
		"(raw %d/%d/%d)", gx, gy, gz, mag, raw[0], raw[1], raw[2]);

	if (mag < 0.7 || mag > 1.3) {
		LOG_WRN("  |a| off 1 g at rest -> move the board; if static, suspect config");
	}
	return 0;
}

/* Quiet single accel sample in g, for the telemetry averaging loop (no logging,
 * no |a| sanity check — the caller averages many of these over the send window).
 */
int bringup_imu_read_accel(float *gx, float *gy, float *gz)
{
	uint16_t d[3];
	int err = bmi_read(REG_ACC_DATA, d, 3);

	if (err) {
		return err;
	}
	if (gx) {
		*gx = (float)((int16_t)d[0]) / lsb_per_g;
	}
	if (gy) {
		*gy = (float)((int16_t)d[1]) / lsb_per_g;
	}
	if (gz) {
		*gz = (float)((int16_t)d[2]) / lsb_per_g;
	}
	return 0;
}

/* --- FIFO at 100 Hz, +/-16 g (shipment events, shipment.c) ------------- */

int bringup_imu_fifo_start(void)
{
	int err;

	fifo_on = false;
	if (bringup_imu_init() != 0) {
		return -EIO;
	}
	err = bmi_write(REG_ACC_CONF, ACC_CONF_NORMAL_16G_100HZ);
	lsb_per_g = LSB_PER_G_16G;
	err |= bmi_write(REG_FIFO_CONF, FIFO_CONF_ACC);
	err |= bmi_write(REG_FIFO_CTRL, 0x0001);
	if (err) {
		LOG_ERR("BMI323 FIFO config failed: %d", err);
		return -EIO;
	}
	k_msleep(10);
	fifo_on = true;
	LOG_INF("BMI323 FIFO: accel 100 Hz, +/-16 g");
	return 0;
}

bool bringup_imu_fifo_on(void)
{
	return fifo_on;
}

/* Read up to max frames (x, y, z in g) from the FIFO. Returns frames or <0.
 * The FIFO holds 1024 words = 341 frames = 3.4 s at 100 Hz; the caller reads
 * every 100 ms, so it only fills while the main loop sends (LTE attach, TLS):
 * then the oldest samples are overwritten and a shock in that gap is missed.
 */
int bringup_imu_fifo_read(float *xyz, int max)
{
	enum { CHUNK = 20 };
	uint8_t buf[2 + CHUNK * 6];
	uint16_t lvl;
	int got = 0;

	if (!fifo_on) {
		return -ENODEV;
	}
	int err = bmi_read(REG_FIFO_FILL, &lvl, 1);

	if (err) {
		return err;
	}
	int frames = (lvl & 0x07FF) / 3;

	if (frames > max) {
		frames = max;
	}
	while (frames > 0) {
		int n = frames > CHUNK ? CHUNK : frames;
		uint8_t reg = REG_FIFO_DATA;

		err = i2c_write_read(i2c, BMI323_ADDR, &reg, 1, buf, 2 + 6 * n);
		if (err) {
			return got ? got : err;
		}
		for (int i = 0; i < n; i++) {
			const uint8_t *f = &buf[2 + 6 * i];
			uint16_t x = f[0] | (f[1] << 8);

			if (x == FIFO_DUMMY_X) {
				continue;
			}
			for (int k = 0; k < 3; k++) {
				int16_t v = (int16_t)(f[2 * k] | (f[2 * k + 1] << 8));

				xyz[3 * got + k] = (float)v / lsb_per_g;
			}
			got++;
		}
		frames -= n;
	}
	return got;
}

/* --- INT1 (P0.05) data-ready edge test ---------------------------------- */
static struct gpio_callback int1_cb;
static volatile uint32_t int1_edges;

static void int1_handler(const struct device *dev, struct gpio_callback *cb,
			 uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	int1_edges++;
}

int bringup_imu_int_test(void)
{
	int err;

	if (bringup_imu_init() != 0) {
		return -EIO;
	}
	if (!gpio_is_ready_dt(&int1)) {
		LOG_ERR("INT1 GPIO (P0.05) not ready");
		return -ENODEV;
	}

	/* INT1 pin: push-pull, active high, output enabled (IO_INT_CTRL).
	 * Map accel data-ready to INT1 (INT_MAP2 acc_drdy = 0b01 -> INT1).
	 * Values per BMI323 datasheet; verify if no edges are seen.
	 */
	err = bmi_write(REG_IO_INT_CTRL, 0x0005);
	err |= bmi_write(REG_INT_MAP2, 0x0400);
	if (err) {
		LOG_ERR("BMI323 INT config failed: %d", err);
		return err;
	}

	int1_edges = 0;
	gpio_pin_configure_dt(&int1, GPIO_INPUT);
	gpio_init_callback(&int1_cb, int1_handler, BIT(int1.pin));
	gpio_add_callback(int1.port, &int1_cb);
	gpio_pin_interrupt_configure_dt(&int1, GPIO_INT_EDGE_BOTH);

	LOG_INF("BMI323 INT1 (P0.05): counting data-ready edges for 1 s...");
	k_msleep(1000);

	gpio_pin_interrupt_configure_dt(&int1, GPIO_INT_DISABLE);
	gpio_remove_callback(int1.port, &int1_cb);

	uint32_t edges = int1_edges;

	(void)bmi_write(REG_INT_MAP2, 0x0000); /* unmap the interrupt again */

	LOG_INF("BMI323 INT1 edges in 1 s: %u (expect ~200 at 100 Hz, both edges)",
		edges);
	if (edges < 50) {
		LOG_ERR("  too few edges -> INT1/P0.05 trace or BMI323 INT map");
		return -EIO;
	}
	return 0;
}
