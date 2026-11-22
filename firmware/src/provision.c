/*
 * Copyright (c) 2024 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * nRF Cloud device provisioning ("check-in") client for the CU-board bring-up.
 * Adapted from the production firmware (CDC-INCO-Thingy9151-R1A) but driven
 * WITHOUT the connection manager — LTE is toggled directly via lte_lc, matching
 * the rest of the bring-up app.
 *
 * Purpose: once the board is claimed at nRF Cloud (with the attestation token
 * printed at boot), this client fetches the provisioning commands that generate
 * + store the nRF Cloud credentials at CONFIG_NRF_CLOUD_SEC_TAG (16842753).
 * After that the A-GNSS REST call's JWT succeeds and real ephemerides download.
 *
 * Ordering / safety: NOT auto-started. main() calls bringup_onboard_begin()
 * only AFTER the MQTT session is up, so the HiveMQ CA write (offline) has
 * already happened and cannot collide with the provisioning key writes. Writing
 * credentials requires the radio off, so the NEED_LTE_* callbacks take LTE (and
 * GNSS) offline first — the modem-safe sequence, no brick. On success we reboot
 * so MQTT/GNSS restart cleanly with the new credentials.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <modem/lte_lc.h>
#include <net/nrf_provisioning.h>

#include "bringup.h"

LOG_MODULE_REGISTER(provision, LOG_LEVEL_INF);

static K_SEM_DEFINE(onboard_sem, 0, 1);
static atomic_t onboard_active = ATOMIC_INIT(0);

void bringup_onboard_begin(void)
{
	k_sem_give(&onboard_sem);
}

bool bringup_onboard_active(void)
{
	return atomic_get(&onboard_active) != 0;
}

static void deactivate_gnss_if_active(void)
{
	enum lte_lc_func_mode mode;

	if (lte_lc_func_mode_get(&mode) == 0 &&
	    mode == LTE_LC_FUNC_MODE_ACTIVATE_GNSS) {
		LOG_INF("Deactivating GNSS for credential storage");
		(void)lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_GNSS);
	}
}

static void provisioning_callback(const struct nrf_provisioning_callback_data *event)
{
	switch (event->type) {
	case NRF_PROVISIONING_EVENT_START:
		LOG_INF("Provisioning check started");
		break;
	case NRF_PROVISIONING_EVENT_STOP:
		LOG_INF("Provisioning check stopped");
		break;
	case NRF_PROVISIONING_EVENT_NEED_LTE_DEACTIVATED:
		LOG_INF("Provisioning: taking LTE offline to store credentials");
		atomic_set(&onboard_active, 1);
		deactivate_gnss_if_active();
		(void)lte_lc_func_mode_set(LTE_LC_FUNC_MODE_OFFLINE);
		break;
	case NRF_PROVISIONING_EVENT_NEED_LTE_ACTIVATED:
		LOG_INF("Provisioning: reactivating LTE");
		(void)lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL);
		atomic_set(&onboard_active, 0);
		break;
	case NRF_PROVISIONING_EVENT_DONE:
		LOG_INF("Provisioning DONE -> rebooting to apply new credentials");
		k_sleep(K_SECONDS(2));
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case NRF_PROVISIONING_EVENT_NO_COMMANDS:
		LOG_INF("Provisioning: nothing to do (device already provisioned)");
		break;
	case NRF_PROVISIONING_EVENT_FAILED_DEVICE_NOT_CLAIMED:
		LOG_WRN("Device NOT claimed yet. Claim it on nrfcloud.com "
			"(Add New Devices) with this attestation token:");
#if defined(CONFIG_NRF_PROVISIONING_PROVIDE_ATTESTATION_TOKEN)
		if (event->token != NULL) {
			LOG_WRN("ATTESTTOKEN: %.*s.%.*s",
				event->token->attest_sz, event->token->attest,
				event->token->cose_sz, event->token->cose);
		}
#endif
		break;
	case NRF_PROVISIONING_EVENT_FAILED_NO_VALID_DATETIME:
		LOG_WRN("Provisioning: no valid network time yet, will retry");
		break;
	case NRF_PROVISIONING_EVENT_FAILED_WRONG_ROOT_CA:
		LOG_ERR("Provisioning: wrong root CA on sec tag %d",
			CONFIG_NRF_PROVISIONING_ROOT_CA_SEC_TAG);
		break;
	case NRF_PROVISIONING_EVENT_SCHEDULED_PROVISIONING:
		LOG_INF("Next provisioning check in %lld s",
			event->next_attempt_time_seconds);
		break;
	case NRF_PROVISIONING_EVENT_FAILED:
	case NRF_PROVISIONING_EVENT_FATAL_ERROR:
		LOG_WRN("Provisioning check failed (event %d)", event->type);
		atomic_set(&onboard_active, 0);
		break;
	default:
		LOG_DBG("Provisioning event %d", event->type);
		break;
	}
}

static void provision_task(void)
{
	int err;

	/* Gate: only run after main() has the MQTT session up (CA written, LTE
	 * attached, network time valid).
	 */
	k_sem_take(&onboard_sem, K_FOREVER);

	for (int attempt = 0; attempt < 5; attempt++) {
		err = nrf_provisioning_init(provisioning_callback);
		if (err == 0) {
			LOG_INF("nRF Cloud provisioning client initialized");
			break;
		}
		LOG_WRN("nrf_provisioning_init failed (%d), retrying...", err);
		k_sleep(K_SECONDS(5));
	}
	if (err) {
		LOG_ERR("provisioning client could not be initialized");
		return;
	}

	/* One-shot check-in now (auto-start is disabled). */
	err = nrf_provisioning_trigger_manually();
	if (err) {
		LOG_WRN("nrf_provisioning_trigger_manually: %d", err);
	}
}

K_THREAD_DEFINE(provision_task_id, 4096, provision_task, NULL, NULL, NULL,
		6, 0, 0);
