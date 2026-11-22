/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Cellular bring-up: init the modem, read modem info (IMEI, temperature,
 * supply voltage), attach LTE-M, and prove the end-to-end data path by
 * resolving + TCP-connecting to the production HiveMQ broker. No TLS here on
 * purpose — reaching the broker's TCP port already proves SIM + LTE + antenna
 * + routing. The full TLS/MQTT publish to the website is the job of the
 * production firmware (CDC-INCO-Thingy9151-R1A) — see the guide.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>

#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#include <modem/modem_attest_token.h>
#include <modem/modem_jwt.h>
#include <nrf_modem_at.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "bringup.h"

LOG_MODULE_REGISTER(cell, LOG_LEVEL_INF);

/* Same broker as the production firmware's prj.conf. */
#define BROKER_HOST "9d3a3c61e89e4384ab8e326039d9f567.s1.eu.hivemq.cloud"
#define BROKER_PORT "8883"

static bool modem_ready;
static bool attached;
static bool ca_provisioned;

/* Raw return of the last nrf_modem_lib_init() — exposed for the diagnostic.
 * >0 = modem firmware / DFU state, <0 = init error, 0 = OK.
 */
int bringup_modem_init_raw = -999;

/* nrf_modem_lib_init() is not reentrant, and here it also runs the credentials
 * provisioning hook (modem_key_mgmt_write). run_post() (main thread) and
 * net_thread both init the modem at boot, so serialise them: the first caller
 * initialises fully — including the key write — while the other waits, instead
 * of issuing a second concurrent init that crashes the modem.
 */
static K_MUTEX_DEFINE(modem_init_lock);

int bringup_modem_init(void)
{
	int err;

	k_mutex_lock(&modem_init_lock, K_FOREVER);

	if (modem_ready) {
		k_mutex_unlock(&modem_init_lock);
		return 0;
	}

	err = nrf_modem_lib_init();
	bringup_modem_init_raw = err;
	if (err < 0) {
		LOG_ERR("nrf_modem_lib_init failed: %d", err);
		k_mutex_unlock(&modem_init_lock);
		return err;
	}
	if (err > 0) {
		LOG_WRN("modem returned %d at init (modem DFU state) — continuing", err);
	}

	err = modem_info_init();
	if (err) {
		LOG_WRN("modem_info_init failed: %d (temp/voltage unavailable)", err);
	}

	modem_ready = true;
	LOG_INF("Modem library initialised");
	k_mutex_unlock(&modem_init_lock);
	return 0;
}

int bringup_cell_info(void)
{
	char imei[24] = {0};
	int temp = 0, batt_mv = 0;
	int err;

	if (bringup_modem_init() != 0) {
		return -EIO;
	}

	err = nrf_modem_at_scanf("AT+CGSN=1", "+CGSN: \"%23[^\"]\"", imei);
	if (err == 1) {
		LOG_INF("Modem IMEI: %s", imei);
	} else {
		LOG_WRN("IMEI read failed (%d)", err);
	}

	if (modem_info_get_temperature(&temp) == 0) {
		LOG_INF("Modem temperature: %d C", temp);
	}
	if (modem_info_get_batt_voltage(&batt_mv) == 0) {
		LOG_INF("Modem supply (VSYS via %%XVBAT): %d mV", batt_mv);
	}
	return 0;
}

/* Print the nRF Cloud onboarding info to RTT: the modem UUID (= the nRF Cloud
 * client-ID with CLIENT_ID_SRC_INTERNAL_UUID) and the attestation token. This
 * is READ-ONLY (AT%ATTESTTOKEN + a UUID read) — no key write, so no modem-brick
 * risk. Copy the "ATTESTTOKEN: attest.cose" line from the RTT log into nRF
 * Cloud -> Add New Devices -> "Input Device Credentials" to claim the board.
 * Once claimed, the A-GNSS REST call gets real ephemerides (no more -77).
 */
void bringup_print_attest_token(void)
{
	struct nrf_attestation_token token = {0};
	struct nrf_device_uuid uuid = {0};
	int err;

	if (bringup_modem_init() != 0) {
		LOG_ERR("attest token: modem not ready");
		return;
	}

	if (modem_jwt_get_uuids(&uuid, NULL) == 0) {
		LOG_INF("nRF Cloud device UUID (client-ID): %s", uuid.str);
	}

	err = modem_attest_token_get(&token);
	if (err) {
		LOG_ERR("modem_attest_token_get failed: %d", err);
		return;
	}

	/* nRF Cloud expects the token as "<attest>.<cose>". */
	LOG_INF("===== nRF Cloud onboarding =====");
	LOG_INF("ATTESTTOKEN: %.*s.%.*s",
		token.attest_sz, token.attest, token.cose_sz, token.cose);
	LOG_INF("Paste the line above into nRF Cloud -> Add New Devices.");
	LOG_INF("================================");

	modem_attest_token_free(&token);
}

int bringup_provision_ca(void)
{
	int err;

	if (bringup_modem_init() != 0) {
		return -EIO;
	}

	if (ca_provisioned) {
		return 0;
	}

	if (attached) {
		LOG_WRN("Already attached (online) — CA write may be rejected; "
			"provision before attaching for reliability");
	}

	/* Write keys only while the radio is off: AT+CFUN=4 keeps the AT
	 * interface up but the modem offline, which is the state modem_key_mgmt
	 * accepts. Done as its own deliberate step (not during nrf_modem_lib
	 * init and not concurrently with the attach) — writing keys too early or
	 * while other modem AT traffic is in flight crashed the modem.
	 */
	err = nrf_modem_at_printf("AT+CFUN=4");
	if (err) {
		LOG_WRN("AT+CFUN=4 (offline) returned %d — provisioning anyway", err);
	}

	if (bringup_credentials_provision() != 0) {
		LOG_ERR("CA provisioning failed — TLS publish will not work");
		return -EIO;
	}

	ca_provisioned = true;
	return 0;
}

int bringup_cell_attach(void)
{
	int err;
	int16_t rxlev, ber, rscp, ecno, rsrq, rsrp;
	char cops[64] = {0};

	if (bringup_modem_init() != 0) {
		return -EIO;
	}

	LOG_INF("Attaching to LTE-M (APN 'wsim')... this can take up to ~60 s");

	err = lte_lc_connect();
	if (err) {
		LOG_ERR("lte_lc_connect failed: %d (SIM? coverage? antenna?)", err);
		return err;
	}
	attached = true;
	LOG_INF("LTE attached");

	if (nrf_modem_at_cmd(cops, sizeof(cops), "AT+COPS?") == 0) {
		/* strip trailing OK/newlines for a tidy line */
		char *nl = strchr(cops, '\r');

		if (nl) {
			*nl = '\0';
		}
		LOG_INF("Operator: %s", cops);
	}

	err = nrf_modem_at_scanf("AT+CESQ",
		"+CESQ: %hd,%hd,%hd,%hd,%hd,%hd",
		&rxlev, &ber, &rscp, &ecno, &rsrq, &rsrp);
	if (err == 6 && rsrp != 255) {
		LOG_INF("Signal: RSRP = %d dBm", rsrp - 140);
	} else {
		LOG_WRN("RSRP not available yet (AT+CESQ -> %d)", err);
	}
	return 0;
}

/* Deactivate LTE only (AT+CFUN=20) while keeping the modem + GNSS alive, so the
 * single shared RF frontend belongs entirely to GPS for a fast cold-start fix.
 * This is a deliberate acquisition window, NOT a power-save mode — LTE is brought
 * back explicitly (bringup_mqtt_session_start -> lte_lc_connect) once GPS has its
 * fix. Do not confuse with eDRX/PSM, which would silently drop the always-on MQTT
 * link. GNSS is not touched here; it stays in whatever state gnss.c set it to.
 */
int bringup_cell_lte_off(void)
{
	int err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);

	if (err) {
		LOG_WRN("CFUN=20 (deactivate LTE) returned %d", err);
	} else {
		LOG_INF("LTE deactivated (CFUN=20) — GPS owns the antenna now");
	}
	attached = false;
	return err;
}

/* ---- Serving-cell monitor ------------------------------------------------ */

/* Split an AT response line into comma-separated fields, honouring quotes.
 * Quotes are stripped. Returns the number of fields.
 */
static int split_fields(char *s, char *fields[], int max)
{
	int n = 0;
	bool inq = false;
	char *start = s;

	for (char *p = s;; p++) {
		if (*p == '"') {
			inq = !inq;
		} else if ((*p == ',' && !inq) || *p == '\0' || *p == '\r' || *p == '\n') {
			bool end = (*p == '\0' || *p == '\r' || *p == '\n');

			*p = '\0';
			if (n < max) {
				/* strip surrounding quotes */
				if (*start == '"') {
					start++;
					size_t l = strlen(start);

					if (l && start[l - 1] == '"') {
						start[l - 1] = '\0';
					}
				}
				fields[n++] = start;
			}
			if (end) {
				break;
			}
			start = p + 1;
		}
	}
	return n;
}

int bringup_cell_monitor(struct cell_status *cs)
{
	char buf[256];
	char *f[20];
	int16_t rxlev, ber, rscp, ecno, rsrq, rsrp;

	memset(cs, 0, sizeof(*cs));
	cs->rsrq = NAN;
	cs->snr = NAN;

	if (!modem_ready) {
		return -ENODEV;
	}
	if (nrf_modem_at_cmd(buf, sizeof(buf), "AT%%XMONITOR") != 0) {
		return -EIO;
	}
	char *p = strstr(buf, "%XMONITOR: ");

	if (!p) {
		return -EIO;
	}
	p += strlen("%XMONITOR: ");

	/* <reg_status>,<full_name>,<short_name>,<plmn>,<tac>,<AcT>,<band>,
	 * <cell_id>,<phys_cell_id>,<EARFCN>,<rsrp>,<snr>,...
	 */
	int n = split_fields(p, f, ARRAY_SIZE(f));
	int reg = n > 0 ? atoi(f[0]) : 0;

	if ((reg != 1 && reg != 5) || n < 12) {
		return -ENOTCONN; /* not registered: only reg_status is present */
	}

	strncpy(cs->plmn, f[3], sizeof(cs->plmn) - 1);
	cs->tac = strtoul(f[4], NULL, 16);
	cs->band = (uint16_t)atoi(f[6]);
	cs->cell_id = strtoul(f[7], NULL, 16);

	int rsrp_raw = atoi(f[10]);
	int snr_raw = atoi(f[11]);

	if (rsrp_raw == 255) {
		return -EAGAIN;
	}
	cs->rsrp = (int16_t)(rsrp_raw - 140);
	if (snr_raw != 127) {
		cs->snr = (float)(snr_raw - 24);
	}

	/* RSRQ is not in XMONITOR; CESQ: 0..34 -> -20 + n/2 dB, 255 = unknown. */
	if (nrf_modem_at_scanf("AT+CESQ", "+CESQ: %hd,%hd,%hd,%hd,%hd,%hd",
			       &rxlev, &ber, &rscp, &ecno, &rsrq, &rsrp) == 6 &&
	    rsrq != 255) {
		cs->rsrq = -20.0f + (float)rsrq / 2.0f;
	}

	cs->valid = true;
	return 0;
}

int bringup_cell_psm(bool enable, int rptau_s, int rat_s)
{
	static int cur_enable = -1;
	static int cur_tau = -1;
	static int cur_rat = -1;
	int err;

	if (!modem_ready) {
		return -ENODEV;
	}
	if (cur_enable == (int)enable &&
	    (!enable || (cur_tau == rptau_s && cur_rat == rat_s))) {
		return 0;
	}
	if (enable) {
		err = lte_lc_psm_param_set_seconds(rptau_s, rat_s);
		if (err) {
			LOG_WRN("PSM params (%d s / %d s) rejected: %d", rptau_s, rat_s, err);
			return err;
		}
	}
	err = lte_lc_psm_req(enable);
	if (err) {
		LOG_WRN("PSM %s request failed: %d", enable ? "enable" : "disable", err);
		return err;
	}
	cur_enable = enable;
	cur_tau = rptau_s;
	cur_rat = rat_s;

	if (enable) {
		LOG_INF("PSM requested: TAU %d s, active time %d s "
			"(the network decides what it grants)", rptau_s, rat_s);
	} else {
		LOG_INF("PSM off");
	}
	return 0;
}

bool bringup_cell_registered(void)
{
	enum lte_lc_nw_reg_status st;

	if (!modem_ready || lte_lc_nw_reg_status_get(&st) != 0) {
		return false;
	}
	return st == LTE_LC_NW_REG_REGISTERED_HOME ||
	       st == LTE_LC_NW_REG_REGISTERED_ROAMING;
}

void bringup_cell_keep_searching(void)
{
	/* lte_lc_connect() restores the previous CFUN after a timeout (CFUN=4
	 * right after the CA write) — put the radio back to normal so the modem
	 * keeps looking for a cell in the background while the app samples on.
	 */
	if (modem_ready) {
		(void)lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL);
	}
}

const char *bringup_cell_imei(void)
{
	static char imei[24];

	if (!imei[0] && modem_ready) {
		if (nrf_modem_at_scanf("AT+CGSN=1", "+CGSN: \"%23[^\"]\"", imei) != 1) {
			imei[0] = '\0';
		}
	}
	return imei;
}

int bringup_cell_reach(void)
{
	struct addrinfo *res = NULL;
	struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};
	int fd = -1;
	int err;
	int64_t t0;

	if (!attached) {
		LOG_WRN("Not attached yet — running 'cell attach' first");
		if (bringup_cell_attach() != 0) {
			return -ENETDOWN;
		}
	}

	LOG_INF("Resolving %s ...", BROKER_HOST);
	err = getaddrinfo(BROKER_HOST, BROKER_PORT, &hints, &res);
	if (err) {
		LOG_ERR("getaddrinfo failed: %d (no data connectivity)", err);
		return -EIO;
	}

	fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
	if (fd < 0) {
		LOG_ERR("socket() failed: %d", errno);
		freeaddrinfo(res);
		return -EIO;
	}

	t0 = k_uptime_get();
	err = connect(fd, res->ai_addr, res->ai_addrlen);
	if (err) {
		LOG_ERR("connect() to broker:%s failed: %d", BROKER_PORT, errno);
	} else {
		LOG_INF("TCP connect to broker:%s OK (%lld ms) -> data path works",
			BROKER_PORT, k_uptime_get() - t0);
	}

	(void)close(fd);
	freeaddrinfo(res);
	return err ? -EIO : 0;
}
