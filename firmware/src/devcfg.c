/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Persistent device configuration (Zephyr settings on the settings_storage
 * partition): operating mode, telemetry interval, GNSS interval, LED request,
 * device ID, telemetry sequence and boot counter. Plus the reset cause of this
 * boot and two tiny flat-JSON helpers used by the MQTT command handler.
 *
 * Survives resets and FOTA updates. A JLink flash of this board mass-erases the
 * chip (it boots secured), which also wipes these settings. The device ID
 * therefore comes from the flash station: the Cockpit ("Lager → Gerät flashen")
 * writes it into the last flash page (0x000FF000) in the same J-Link session
 * as the firmware, and reads ID + IMEI back from the factory-info RAM block below.
 *
 * The backend is the source of truth for mode/interval/GNSS/LED: it keeps a
 * RETAINED config message on projekt/cfg/<id>, which the device receives on every
 * (re)connect and applies here. Local shell changes are overwritten by it.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/hwinfo.h>
#include <stdlib.h>
#include <string.h>

#include "bringup.h"

LOG_MODULE_REGISTER(devcfg, LOG_LEVEL_INF);

/* Telemetry sequence is persisted only every SEQ_STEP records (flash wear); at
 * boot it continues at the next multiple, so a value is never reused. After a
 * power loss seq therefore jumps forward by < SEQ_STEP — together with "boot"
 * and "rst" in the same message that is easy to tell apart from lost messages.
 */
#define SEQ_STEP 16

/* Per-mode defaults (interval_s, gnss_s). */
static const struct {
	const char *name;
	uint32_t interval_s;
	uint32_t gnss_s;
} mode_defaults[] = {
	[APP_MODE_NORMAL] = { "normal", 300, 300 },
	[APP_MODE_LIVE]   = { "live",    30, 120 },
	[APP_MODE_DRAIN]  = { "drain",   10,   1 },
};

#define INTERVAL_MIN_S 10
#define INTERVAL_MAX_S 86400

static K_MUTEX_DEFINE(cfg_lock);

static struct app_cfg cfg = {
	.mode = APP_MODE_NORMAL,
	.interval_s = 300,
	.gnss_s = 300,
	.led = LED_CFG_OFF,
};
/* Factory ID in the last flash page (0x000FF000, unused EMPTY_1 area behind
 * settings_storage, same 32 KB SPU region -> non-secure readable), written by
 * the flash station: "PID1" + NUL-terminated ID. FOTA never touches it; a mass
 * erase clears it and the station writes it again with every flash.
 * Not UICR any more: reading UICR via TF-M (soc_secure_mem_read) on the nRF9151
 * broke nrf_modem_lib_init (-116) and corrupted the kernel timeout list
 * (R1A, 2026-10-05).
 */
#define FACTORY_ID_ADDR  0x000FF000
#define FACTORY_ID_MAGIC "PID1"

/* RAM block the flash station finds by its magic in a SWD dump of the halted
 * core (no ELF / fixed address needed). Filled at runtime only, so the magic is
 * never in RAM before the content is valid.
 */
static struct {
	char magic[8];   /* "PECUFAC1" */
	char id[24];
	char imei[24];
	char fw[16];
	char end[8];     /* "PECUEND1" */
} __used factory_info;

static char dev_id[24];      /* settings override */
static char factory_id[24];  /* from the factory-ID flash page */
static uint32_t seq_next;
static uint32_t seq_saved_upto;
static uint32_t boot_count;
static uint32_t reset_cause;
static atomic_t generation;

/* ------------------------------ JSON helpers ----------------------------- */

static const char *json_find_value(const char *json, const char *key)
{
	char pat[24];

	snprintk(pat, sizeof(pat), "\"%s\"", key);
	const char *p = strstr(json, pat);

	if (!p) {
		return NULL;
	}
	p = strchr(p + strlen(pat), ':');
	if (!p) {
		return NULL;
	}
	p++;
	while (*p == ' ' || *p == '\t') {
		p++;
	}
	return p;
}

bool json_get_str(const char *json, const char *key, char *out, size_t n)
{
	const char *p = json_find_value(json, key);

	if (!p || *p != '"') {
		return false;
	}
	p++;
	size_t i = 0;

	while (*p && *p != '"' && i < n - 1) {
		out[i++] = *p++;
	}
	out[i] = '\0';
	return *p == '"';
}

bool json_get_int(const char *json, const char *key, int32_t *out)
{
	const char *p = json_find_value(json, key);
	char *end;

	if (!p) {
		return false;
	}
	/* Accept both 300 and "300", and true/false for flags. */
	if (*p == '"') {
		p++;
	}
	if (strncmp(p, "true", 4) == 0) {
		*out = 1;
		return true;
	}
	if (strncmp(p, "false", 5) == 0) {
		*out = 0;
		return true;
	}
	long v = strtol(p, &end, 10);

	if (end == p) {
		return false;
	}
	*out = (int32_t)v;
	return true;
}

/* ------------------------------- settings -------------------------------- */

static int cfg_set(const char *name, size_t len, settings_read_cb read_cb,
		   void *cb_arg)
{
	const char *next;
	int rc = 0;

	if (settings_name_steq(name, "id", &next) && !next) {
		memset(dev_id, 0, sizeof(dev_id));
		rc = read_cb(cb_arg, dev_id, MIN(len, sizeof(dev_id) - 1));
	} else if (settings_name_steq(name, "mode", &next) && !next) {
		uint8_t m;

		rc = read_cb(cb_arg, &m, sizeof(m));
		if (rc > 0 && m <= APP_MODE_DRAIN) {
			cfg.mode = (enum app_mode)m;
		}
	} else if (settings_name_steq(name, "int", &next) && !next) {
		rc = read_cb(cb_arg, &cfg.interval_s, sizeof(cfg.interval_s));
	} else if (settings_name_steq(name, "gnss", &next) && !next) {
		rc = read_cb(cb_arg, &cfg.gnss_s, sizeof(cfg.gnss_s));
	} else if (settings_name_steq(name, "led", &next) && !next) {
		uint8_t l;

		rc = read_cb(cb_arg, &l, sizeof(l));
		if (rc > 0) {
			cfg.led = l <= LED_CFG_MAX ? l : LED_CFG_GREEN;
		}
	} else if (settings_name_steq(name, "seq", &next) && !next) {
		rc = read_cb(cb_arg, &seq_saved_upto, sizeof(seq_saved_upto));
	} else if (settings_name_steq(name, "boot", &next) && !next) {
		rc = read_cb(cb_arg, &boot_count, sizeof(boot_count));
	}
	return rc < 0 ? rc : 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(app_cfg, "app", NULL, cfg_set, NULL, NULL);

static void save(const char *key, const void *val, size_t len)
{
	char path[16];

	snprintk(path, sizeof(path), "app/%s", key);
	int err = settings_save_one(path, val, len);

	if (err) {
		LOG_WRN("settings_save_one(%s) failed: %d", path, err);
	}
}

static void save_cfg_locked(void)
{
	uint8_t m = (uint8_t)cfg.mode;
	uint8_t l = cfg.led;

	save("mode", &m, sizeof(m));
	save("int", &cfg.interval_s, sizeof(cfg.interval_s));
	save("gnss", &cfg.gnss_s, sizeof(cfg.gnss_s));
	save("led", &l, sizeof(l));
}

static uint32_t clamp_interval(int32_t v)
{
	return (uint32_t)CLAMP(v, INTERVAL_MIN_S, INTERVAL_MAX_S);
}

static uint32_t clamp_gnss(int32_t v)
{
	if (v <= 0) {
		return 0;
	}
	if (v < 10) {
		return 1; /* the modem only does continuous (1) or periodic >= 10 s */
	}
	return (uint32_t)MIN(v, INTERVAL_MAX_S);
}

static void read_reset_cause(void)
{
	uint32_t cause = 0;

	if (hwinfo_get_reset_cause(&cause) == 0) {
		reset_cause = cause;
		(void)hwinfo_clear_reset_cause();
	}
}

static void read_factory_id(void)
{
	uint8_t otp[32];

	memcpy(otp, (const void *)FACTORY_ID_ADDR, sizeof(otp));
	if (memcmp(otp, FACTORY_ID_MAGIC, 4) != 0) {
		return;
	}
	const char *id = (const char *)&otp[4];
	size_t len = strnlen(id, sizeof(otp) - 4);

	if (len == 0 || len >= sizeof(factory_id) || len == sizeof(otp) - 4) {
		return;
	}
	for (size_t i = 0; i < len; i++) {
		if (id[i] < '!' || id[i] > '~' || strchr("+#/\"", id[i])) {
			return;
		}
	}
	memcpy(factory_id, id, len);
	factory_id[len] = '\0';
}

void devcfg_factory_info_update(void)
{
	strncpy(factory_info.id, devcfg_device_id(), sizeof(factory_info.id) - 1);
	strncpy(factory_info.imei, bringup_cell_imei(), sizeof(factory_info.imei) - 1);
	strncpy(factory_info.fw, APP_FW_VERSION, sizeof(factory_info.fw) - 1);
	memcpy(factory_info.end, "PECUEND1", 8);
	compiler_barrier();
	memcpy(factory_info.magic, "PECUFAC1", 8);
}

int devcfg_init(void)
{
	int err;

	read_reset_cause();
	read_factory_id();

	err = settings_subsys_init();
	if (err) {
		LOG_ERR("settings_subsys_init: %d", err);
	}
	err = settings_load_subtree("app");
	if (err) {
		LOG_WRN("settings_load_subtree(app): %d", err);
	}

	if (cfg.interval_s < INTERVAL_MIN_S || cfg.interval_s > INTERVAL_MAX_S) {
		cfg.interval_s = mode_defaults[cfg.mode].interval_s;
	}
	cfg.gnss_s = clamp_gnss((int32_t)cfg.gnss_s);

	boot_count++;
	save("boot", &boot_count, sizeof(boot_count));

	/* Continue after the last persisted block so no seq value repeats. */
	seq_next = seq_saved_upto;
	seq_saved_upto = seq_next + SEQ_STEP;
	save("seq", &seq_saved_upto, sizeof(seq_saved_upto));

	LOG_INF("Config: id=%s mode=%s int=%us gnss=%us led=%d boot=%u seq=%u rst=%s(0x%x)",
		devcfg_device_id(), devcfg_mode_name(cfg.mode), cfg.interval_s,
		cfg.gnss_s, cfg.led, boot_count, seq_next, devcfg_reset_name(),
		reset_cause);
	return 0;
}

void devcfg_get(struct app_cfg *out)
{
	k_mutex_lock(&cfg_lock, K_FOREVER);
	*out = cfg;
	k_mutex_unlock(&cfg_lock);
}

uint32_t devcfg_generation(void)
{
	return (uint32_t)atomic_get(&generation);
}

const char *devcfg_mode_name(enum app_mode m)
{
	return (m <= APP_MODE_DRAIN) ? mode_defaults[m].name : "?";
}

bool devcfg_mode_from_name(const char *name, enum app_mode *out)
{
	for (int m = 0; m < ARRAY_SIZE(mode_defaults); m++) {
		if (strcmp(name, mode_defaults[m].name) == 0) {
			*out = (enum app_mode)m;
			return true;
		}
	}
	return false;
}

int devcfg_set_mode(enum app_mode m)
{
	if (m > APP_MODE_DRAIN) {
		return -EINVAL;
	}
	k_mutex_lock(&cfg_lock, K_FOREVER);
	cfg.mode = m;
	cfg.interval_s = mode_defaults[m].interval_s;
	cfg.gnss_s = mode_defaults[m].gnss_s;
	save_cfg_locked();
	k_mutex_unlock(&cfg_lock);
	atomic_inc(&generation);
	LOG_INF("Mode -> %s (int %us, gnss %us)", devcfg_mode_name(m),
		mode_defaults[m].interval_s, mode_defaults[m].gnss_s);
	return 0;
}

bool devcfg_apply_json(const char *json)
{
	struct app_cfg n;
	char mode[12];
	int32_t v;

	k_mutex_lock(&cfg_lock, K_FOREVER);
	n = cfg;

	if (json_get_str(json, "mode", mode, sizeof(mode))) {
		enum app_mode m;

		if (devcfg_mode_from_name(mode, &m)) {
			if (m != n.mode) {
				n.interval_s = mode_defaults[m].interval_s;
				n.gnss_s = mode_defaults[m].gnss_s;
			}
			n.mode = m;
		} else {
			LOG_WRN("cfg: unknown mode \"%s\" ignored", mode);
		}
	}
	if (json_get_int(json, "int", &v)) {
		n.interval_s = clamp_interval(v);
	}
	if (json_get_int(json, "gnss", &v)) {
		n.gnss_s = clamp_gnss(v);
	}
	if (json_get_int(json, "led", &v)) {
		n.led = (v >= LED_CFG_OFF && v <= LED_CFG_MAX) ? (uint8_t)v : LED_CFG_GREEN;
		if (LED_CFG_COLOUR(n.led) == LED_CFG_OFF) {
			n.led = LED_CFG_OFF; /* "blinking off" = off */
		}
	}

	bool changed = memcmp(&n, &cfg, sizeof(n)) != 0;

	if (changed) {
		cfg = n;
		save_cfg_locked();
	}
	k_mutex_unlock(&cfg_lock);

	if (changed) {
		atomic_inc(&generation);
		LOG_INF("cfg applied: mode=%s int=%us gnss=%us led=%d",
			devcfg_mode_name(n.mode), n.interval_s, n.gnss_s, n.led);
	}
	return changed;
}

const char *devcfg_device_id(void)
{
	static char imei_id[24];

	if (dev_id[0]) {
		return dev_id;
	}
	if (factory_id[0]) {
		return factory_id;
	}
	/* Lab board flashed without the Cockpit: fall back to the IMEI. */
	if (!imei_id[0]) {
		const char *imei = bringup_cell_imei();

		if (imei[0]) {
			snprintk(imei_id, sizeof(imei_id), "IMEI%s", imei);
		}
	}
	return imei_id[0] ? imei_id : "UNSET";
}

int devcfg_set_device_id(const char *id)
{
	size_t len = strlen(id);

	if (len == 0 || len >= sizeof(dev_id)) {
		return -EINVAL;
	}
	/* Keep it topic-safe: no MQTT wildcards / separators / quotes. */
	for (size_t i = 0; i < len; i++) {
		if (strchr("+#/\" ", id[i])) {
			return -EINVAL;
		}
	}
	k_mutex_lock(&cfg_lock, K_FOREVER);
	memset(dev_id, 0, sizeof(dev_id));
	memcpy(dev_id, id, len);
	save("id", dev_id, len);
	k_mutex_unlock(&cfg_lock);
	LOG_INF("Device ID set to %s (takes effect after reboot)", dev_id);
	return 0;
}

uint32_t devcfg_next_seq(void)
{
	uint32_t s = seq_next++;

	if (seq_next >= seq_saved_upto) {
		seq_saved_upto = seq_next + SEQ_STEP;
		save("seq", &seq_saved_upto, sizeof(seq_saved_upto));
	}
	return s;
}

uint32_t devcfg_boot_count(void)
{
	return boot_count;
}

uint32_t devcfg_reset_cause(void)
{
	return reset_cause;
}

const char *devcfg_reset_name(void)
{
	/* Most telling cause first (several bits can be set at once). */
	if (reset_cause & RESET_WATCHDOG) {
		return "wdt";
	}
	if (reset_cause & RESET_CPU_LOCKUP) {
		return "lockup";
	}
	if (reset_cause & RESET_BROWNOUT) {
		return "bor";
	}
	if (reset_cause & RESET_SOFTWARE) {
		return "sw";
	}
	if (reset_cause & RESET_DEBUG) {
		return "dbg";
	}
	if (reset_cause & RESET_PIN) {
		return "pin";
	}
	if (reset_cause & RESET_POR || reset_cause == 0) {
		return "por";
	}
	return "other";
}
