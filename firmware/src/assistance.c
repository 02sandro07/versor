/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright (c) 2024 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * nRF Cloud A-GNSS assistance over REST. Adapted from the NCS GNSS sample and
 * the production firmware (CDC-INCO-Thingy9151-R1A). One-shot HTTPS request to
 * nRF Cloud, authenticated with a device JWT; the received A-GNSS data
 * (ephemerides, time, coarse position) is injected into the modem so the GNSS
 * core gets a fix in seconds instead of minutes. Independent of the app's own
 * HiveMQ MQTT connection.
 */
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#include <modem/modem_jwt.h>
#include <net/nrf_cloud_rest.h>
#include <net/nrf_cloud_agnss.h>

#include "assistance.h"

LOG_MODULE_DECLARE(gnss, LOG_LEVEL_INF);

static char jwt_buf[600];
static char rx_buf[2048];
static char agnss_data_buf[NRF_CLOUD_AGNSS_MAX_DATA_SIZE];

static int serving_cell_info_get(struct lte_lc_cell *serving_cell)
{
	int err;
	char resp_buf[MODEM_INFO_MAX_RESPONSE_SIZE];

	err = modem_info_init();
	if (err) {
		return err;
	}

	err = modem_info_string_get(MODEM_INFO_CELLID, resp_buf, sizeof(resp_buf));
	if (err < 0) {
		return err;
	}
	serving_cell->id = strtol(resp_buf, NULL, 16);

	err = modem_info_string_get(MODEM_INFO_AREA_CODE, resp_buf, sizeof(resp_buf));
	if (err < 0) {
		return err;
	}
	serving_cell->tac = strtol(resp_buf, NULL, 16);

	err = modem_info_string_get(MODEM_INFO_OPERATOR, resp_buf, sizeof(resp_buf));
	if (err < 0) {
		return err;
	}
	serving_cell->mnc = strtol(&resp_buf[3], NULL, 10);
	resp_buf[3] = '\0';
	serving_cell->mcc = strtol(resp_buf, NULL, 10);

	return 0;
}

int assistance_init(struct k_work_q *assistance_work_q)
{
	ARG_UNUSED(assistance_work_q);
	/* A-GNSS over REST needs no persistent state. */
	return 0;
}

int assistance_request(struct nrf_modem_gnss_agnss_data_frame *agnss_request)
{
	int err;

	err = nrf_cloud_jwt_generate(0, jwt_buf, sizeof(jwt_buf));
	if (err) {
		LOG_ERR("JWT generate failed: %d (device not onboarded to nRF Cloud?)",
			err);
		return err;
	}

	struct nrf_cloud_rest_context rest_ctx = {
		.connect_socket = -1,
		.keep_alive = false,
		/* MUST be bounded: A-GNSS is best-effort and must never block GNSS.
		 * With NONE, a stalled TLS/connect to nRF Cloud hangs forever, so
		 * agnss_fetch_done never gets set and GNSS priority mode is never
		 * enabled → the receiver is starved by LTE and tracks 0 satellites
		 * (observed 2026-09-17 on the first onboarded boot). 30 s cap → the
		 * call errors out, GNSS falls back to a standalone (slower) fix.
		 */
		.timeout_ms = 30000,
		.auth = jwt_buf,
		.rx_buf = rx_buf,
		.rx_buf_len = sizeof(rx_buf),
		.fragment_size = 0,
	};

	struct nrf_cloud_rest_agnss_request request = {
		.type = NRF_CLOUD_REST_AGNSS_REQ_CUSTOM,
		.agnss_req = agnss_request,
		.net_info = NULL,
	};

	struct nrf_cloud_rest_agnss_result result = {
		.buf = agnss_data_buf,
		.buf_sz = sizeof(agnss_data_buf),
		.agnss_sz = 0,
	};

	struct lte_lc_cells_info net_info = { 0 };

	if (serving_cell_info_get(&net_info.current_cell) == 0) {
		request.net_info = &net_info;
	} else {
		LOG_WRN("No cell info — requesting A-GNSS without it");
	}

	LOG_INF("Requesting A-GNSS data (flags 0x%02x)", agnss_request->data_flags);

	err = nrf_cloud_rest_agnss_data_get(&rest_ctx, &request, &result);
	if (err) {
		LOG_ERR("nrf_cloud_rest_agnss_data_get failed: %d", err);
		return err;
	}

	LOG_INF("Processing %d bytes of A-GNSS data", result.agnss_sz);

	err = nrf_cloud_agnss_process(result.buf, result.agnss_sz);
	if (err) {
		LOG_ERR("nrf_cloud_agnss_process failed: %d", err);
		return err;
	}

	LOG_INF("A-GNSS data processed and injected");
	return 0;
}
