/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Provision the broker's CA certificate to the modem so `bu pub` can bring up a
 * TLS/MQTT session. Called once from the attach path (cell.c) after the modem
 * is fully initialised AND put into offline mode (CFUN=4) — writing keys during
 * the nrf_modem_lib init hook proved too early here (the AT interface was not
 * ready yet: "CMEE status err -77" -> "Phone failure" -> modem crash). The CA
 * is the same ISRG Root X1 used by the production firmware; it is embedded from
 * src/credentials/ca-cert.pem via generate_inc_file_for_target() (see
 * CMakeLists.txt). Only the CA is provisioned — HiveMQ uses username/password,
 * not client certificates.
 */
#include <modem/modem_key_mgmt.h>
#include <zephyr/logging/log.h>

#include "bringup.h"

LOG_MODULE_REGISTER(credentials, LOG_LEVEL_INF);

static const unsigned char ca_certificate[] = {
#include "ca-cert.pem"
};

int bringup_credentials_provision(void)
{
	int err;

	if (sizeof(ca_certificate) <= 1) {
		LOG_ERR("CA certificate empty — src/credentials/ca-cert.pem missing?");
		return -ENOENT;
	}

	/* Write unconditionally (idempotent overwrite), exactly like the
	 * production firmware. Must run while the modem is offline — this is
	 * called from the NRF_MODEM_LIB_ON_INIT hook, before any LTE attach.
	 */
	err = modem_key_mgmt_write(BROKER_SEC_TAG,
				   MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
				   ca_certificate, sizeof(ca_certificate) - 1);
	if (err) {
		LOG_ERR("modem_key_mgmt_write(CA) failed: %d", err);
		return err;
	}

	LOG_INF("CA certificate provisioned to sec tag %d", BROKER_SEC_TAG);
	return 0;
}
