/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Cellular-only modem diagnostic for the CU board (no TLS, no MQTT, no keys).
 * Answers the basics in isolation — is the SIM readable, does the modem
 * register, how strong is the signal — the way the DK proved out, before TLS.
 *
 * Visibility: the CU board's only console is SEGGER RTT over SWD, but the
 * app-core debug port powers down when the CPU idles, so live RTT is
 * unreliable. Instead, every result is written into the global `g_diag`
 * struct in RAM (with a magic + a `stage` counter). A held J-Link connection
 * can mem-read that struct at any time — even mid-run — to see exactly how far
 * the modem got and the raw AT responses. The final busy-hold keeps the CPU
 * active (never k_sleep) so the debug port stays powered while reading.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <modem/lte_lc.h>
#include <nrf_modem_at.h>
#include <nrf_modem.h>

#include "bringup.h"

LOG_MODULE_REGISTER(modemdiag, LOG_LEVEL_INF);

#define DIAG_MAGIC 0xD1A65A1E

/* Read via J-Link: `mem8 <&g_diag> <sizeof>`; address is in zephyr.map. */
struct diag_result {
	volatile uint32_t magic;   /* DIAG_MAGIC once populated            */
	volatile uint32_t stage;   /* increments as each step completes    */
	volatile int32_t  init_err;
	volatile int32_t  init_raw; /* raw nrf_modem_lib_init(): >0=DFU/fw */
	volatile int32_t  attach_err;
	volatile int32_t  rsrp_dbm; /* -140 = invalid                      */
	volatile int32_t  rsrq_raw;
	volatile uint32_t fault_count;  /* # of modem faults seen          */
	volatile uint32_t fault_reason; /* nrf_modem fault reason           */
	volatile uint32_t fault_pc;     /* modem PC at fault                */
	char at_ping[24];           /* bare "AT" — does the modem answer?  */
	char cgmr[48];              /* modem firmware version              */
	char imei[24];
	char iccid[32];
	char imsi[24];
	char cfun[24];
	char sysmode[48];
	char cereg[64];
	char xmonitor[160];
	char cops[48];
	char cgpaddr[64];
};

volatile struct diag_result g_diag;

/* Application-specific modem fault handler (CONFIG_NRF_MODEM_LIB_ON_FAULT_
 * APPLICATION_SPECIFIC). Records the fault so the JLink readout shows whether
 * the modem crashed and why, then leaves it (does not auto-reset).
 */
void nrf_modem_fault_handler(struct nrf_modem_fault_info *fault_info)
{
	g_diag.fault_count++;
	if (fault_info) {
		g_diag.fault_reason = fault_info->reason;
		g_diag.fault_pc = fault_info->program_counter;
	}
	LOG_ERR("MODEM FAULT #%u reason 0x%x PC 0x%x",
		g_diag.fault_count, g_diag.fault_reason, g_diag.fault_pc);
}

/* Run an AT command, store trimmed response into dst[dstlen]. */
static void at_get(const char *cmd, char *dst, size_t dstlen)
{
	char buf[200] = {0};
	int err = nrf_modem_at_cmd(buf, sizeof(buf), "%s", cmd);

	for (int i = (int)strlen(buf) - 1; i >= 0; i--) {
		if (buf[i] == '\r' || buf[i] == '\n' || buf[i] == ' ') {
			buf[i] = '\0';
		} else {
			break;
		}
	}
	if (err) {
		snprintk(dst, dstlen, "ERR%d:%s", err, buf);
	} else {
		strncpy(dst, buf, dstlen - 1);
		dst[dstlen - 1] = '\0';
	}
	LOG_INF("%s -> %s", cmd, dst);
}

void bringup_modem_diag(void)
{
	int err;
	int16_t rxlev, ber, rscp, ecno, rsrq, rsrp;

	memset((void *)&g_diag, 0, sizeof(g_diag));
	g_diag.rsrp_dbm = -140;
	g_diag.stage = 1;

	LOG_INF("========== MODEM DIAGNOSTIC (cellular only) ==========");

	if (bringup_modem_init() != 0) {
		g_diag.init_err = -1;
		g_diag.magic = DIAG_MAGIC;
		LOG_ERR("modem init failed");
		goto hold;
	}
	g_diag.stage = 2;
	g_diag.init_raw = bringup_modem_init_raw;

	/* Does the modem answer AT at all? */
	at_get("AT",             g_diag.at_ping, sizeof(g_diag.at_ping));
	at_get("AT+CGMR",        g_diag.cgmr,    sizeof(g_diag.cgmr));

	/* Pre-attach: modem + SIM health. */
	at_get("AT+CFUN?",       g_diag.cfun,    sizeof(g_diag.cfun));
	at_get("AT+CGSN=1",      g_diag.imei,    sizeof(g_diag.imei));
	at_get("AT%XICCID",      g_diag.iccid,   sizeof(g_diag.iccid));
	at_get("AT+CIMI",        g_diag.imsi,    sizeof(g_diag.imsi));
	at_get("AT%XSYSTEMMODE?",g_diag.sysmode, sizeof(g_diag.sysmode));
	g_diag.stage = 3;
	g_diag.magic = DIAG_MAGIC; /* pre-attach info valid even if attach hangs */

	/* Full publish path: provision CA -> attach -> TLS connect -> publish.
	 * Stage-by-stage results land in g_pub (see mqtt.c) so we can read exactly
	 * where it fails. This blocks through attach + TLS handshake.
	 */
	LOG_INF("Running full MQTT publish (provision+attach+TLS+publish)...");
	g_diag.attach_err = bringup_mqtt_pub(NULL);
	g_diag.stage = 4;

	/* Post: registration, operator, signal, IP (if still attached). */
	at_get("AT+CEREG?",   g_diag.cereg,    sizeof(g_diag.cereg));
	at_get("AT%XMONITOR", g_diag.xmonitor, sizeof(g_diag.xmonitor));
	at_get("AT+COPS?",    g_diag.cops,     sizeof(g_diag.cops));
	at_get("AT+CGPADDR",  g_diag.cgpaddr,  sizeof(g_diag.cgpaddr));

	err = nrf_modem_at_scanf("AT+CESQ",
		"+CESQ: %hd,%hd,%hd,%hd,%hd,%hd",
		&rxlev, &ber, &rscp, &ecno, &rsrq, &rsrp);
	if (err == 6 && rsrp != 255) {
		g_diag.rsrp_dbm = rsrp - 140;
		g_diag.rsrq_raw = rsrq;
	}
	g_diag.stage = 5;
	LOG_INF("========== DIAGNOSTIC DONE (stage=5) ==========");

hold:
	/* Repeat the publish every 30 s so the dashboard shows an ongoing stream
	 * (the first publish already happened above). Each call reuses the existing
	 * attach + provisioned CA and just reconnects + publishes + disconnects.
	 */
	for (uint32_t n = 1;; n++) {
		k_sleep(K_SECONDS(30));
		int r = bringup_mqtt_pub(NULL);
		g_diag.rsrq_raw = (int32_t)n; /* reuse field as publish counter */
		LOG_INF("[repeat pub #%u] result %d", n, r);
	}
}
