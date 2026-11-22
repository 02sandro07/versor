/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * FOTA (firmware over-the-air) for the CU board — self-hosted HTTP(S) variant.
 *
 * The board runs under MCUboot with two application slots (primary = running
 * app, secondary = staged update), both in the nRF9151's internal 1 MB flash
 * (the app is small enough that no external flash is needed). This module wraps
 * the NCS `fota_download` library:
 *
 *   1) A URL to `app_update.bin` (built automatically next to the app) is handed
 *      in — either from the `bu fota <url>` RTT shell command, or pushed as an
 *      MQTT command over the existing HiveMQ link (mqtt.c parses it and calls
 *      bringup_fota_start_url()).
 *   2) fota_download streams the image blockwise over LTE-M into the secondary
 *      slot while the main app keeps running. HTTPS uses the ISRG Root X1 CA
 *      already provisioned on BROKER_SEC_TAG (same Let's Encrypt chain the NAS
 *      uses), so no extra credential write is needed.
 *   3) On FINISHED the secondary slot is tagged valid. We do NOT reboot from the
 *      download callback (that thread must not touch the MQTT client): instead we
 *      queue a status + a reboot request, and main() publishes the status and
 *      reboots from its own thread. MCUboot then swaps the slots on reboot.
 *   4) The new image boots in MCUboot "test" mode. Once the app proves healthy
 *      (LTE + MQTT + a successful publish), main() calls bringup_fota_confirm()
 *      -> boot_write_img_confirmed(), making the swap permanent. If the new image
 *      ever crashes/hangs before confirming, MCUboot rolls back to the previous
 *      working image on the next reset — the field unit never bricks.
 *
 * Thread-safety: the download runs in fota_download's own workqueue/thread and
 * only calls back into this file. All MQTT traffic stays on the main thread via
 * the queued-status API below, so the mqtt_client is never touched concurrently.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/dfu/mcuboot.h>
#include <net/fota_download.h>
#include <string.h>

#include "bringup.h"

LOG_MODULE_REGISTER(fota, LOG_LEVEL_INF);

/* fota_download does NOT copy the host/file pointers, so they must stay valid
 * for the whole download — keep them in static storage owned by this module.
 */
static char fota_host[128]; /* e.g. "https://nas.example.com:443" (scheme kept) */
static char fota_file[192]; /* e.g. "fw/app_update.bin" (no leading slash)      */

static bool fota_inited;
static volatile bool fota_busy;         /* a download is in flight               */
static volatile bool fota_reboot_req;   /* image staged -> main() should reboot  */

/* Single queued status word for main() to publish as {"fota":"<word>"} (last
 * one wins; FOTA events are infrequent, so a 1-deep mailbox is enough).
 * Guarded by a tiny spinlock.
 */
static struct k_spinlock status_lock;
static char status_msg[24];
static bool status_pending;

static void queue_status(const char *word)
{
	k_spinlock_key_t key = k_spin_lock(&status_lock);

	strncpy(status_msg, word, sizeof(status_msg) - 1);
	status_msg[sizeof(status_msg) - 1] = '\0';
	status_pending = true;
	k_spin_unlock(&status_lock, key);
}

bool bringup_fota_dequeue_status(char *out, size_t n)
{
	bool had = false;
	k_spinlock_key_t key = k_spin_lock(&status_lock);

	if (status_pending) {
		strncpy(out, status_msg, n - 1);
		out[n - 1] = '\0';
		status_pending = false;
		had = true;
	}
	k_spin_unlock(&status_lock, key);
	return had;
}

bool bringup_fota_reboot_requested(void)
{
	return fota_reboot_req;
}

bool bringup_fota_busy(void)
{
	return fota_busy;
}

static void fota_dl_handler(const struct fota_download_evt *evt)
{
	switch (evt->id) {
	case FOTA_DOWNLOAD_EVT_PROGRESS:
		LOG_INF("FOTA progress: %d%%", evt->progress);
		break;
	case FOTA_DOWNLOAD_EVT_FINISHED:
		LOG_INF("FOTA download finished — image staged in secondary slot. "
			"Requesting reboot to let MCUboot swap.");
		queue_status("downloaded");
		fota_busy = false;
		fota_reboot_req = true; /* main() publishes + reboots on its thread */
		break;
	case FOTA_DOWNLOAD_EVT_ERASE_PENDING:
		LOG_INF("FOTA: erasing secondary slot...");
		break;
	case FOTA_DOWNLOAD_EVT_ERASE_TIMEOUT:
		LOG_WRN("FOTA: erase timeout");
		break;
	case FOTA_DOWNLOAD_EVT_ERASE_DONE:
		LOG_INF("FOTA: secondary slot erased");
		break;
	case FOTA_DOWNLOAD_EVT_ERROR:
		LOG_ERR("FOTA error (cause %d) — download aborted, running image "
			"untouched", evt->cause);
		queue_status("error");
		fota_busy = false;
		break;
	case FOTA_DOWNLOAD_EVT_CANCELLED:
		LOG_WRN("FOTA cancelled");
		queue_status("cancelled");
		fota_busy = false;
		break;
	default:
		break;
	}
}

int bringup_fota_init(void)
{
	if (fota_inited) {
		return 0;
	}

	int err = fota_download_init(fota_dl_handler);

	if (err) {
		LOG_ERR("fota_download_init failed: %d", err);
		return err;
	}
	fota_inited = true;

	/* Log where we stand: swap_type tells us if a staged image is pending or
	 * a test image is running unconfirmed; boot_is_img_confirmed() is false
	 * only for a freshly swapped-in FOTA image that has not confirmed yet.
	 */
	LOG_INF("FOTA ready (MCUboot). swap_type=%d, running image %sconfirmed",
		mcuboot_swap_type(),
		boot_is_img_confirmed() ? "" : "NOT ");
	return 0;
}

int bringup_fota_confirm(void)
{
	if (!IS_ENABLED(CONFIG_MCUBOOT_IMG_MANAGER)) {
		return 0;
	}
	if (boot_is_img_confirmed()) {
		return 0; /* already permanent (normal JLink flash or prior confirm) */
	}

	int err = boot_write_img_confirmed();

	if (err) {
		LOG_ERR("boot_write_img_confirmed failed: %d (image may roll back!)",
			err);
		return err;
	}
	LOG_INF("Running image CONFIRMED — MCUboot will keep it (no rollback)");
	return 0;
}

/* Split "http(s)://host[:port]/path/file" into fota_host (scheme+host[+port])
 * and fota_file (everything after the first '/' of the path). Returns 0 on
 * success. sec_tag_out is set to BROKER_SEC_TAG for https, -1 for plain http.
 */
static int split_url(const char *url, int *sec_tag_out)
{
	const char *scheme_end;
	bool https;

	if (strncmp(url, "https://", 8) == 0) {
		scheme_end = url + 8;
		https = true;
	} else if (strncmp(url, "http://", 7) == 0) {
		scheme_end = url + 7;
		https = false;
	} else {
		LOG_ERR("FOTA URL must start with http:// or https:// : %s", url);
		return -EINVAL;
	}

	const char *slash = strchr(scheme_end, '/');

	if (slash == NULL || slash[1] == '\0') {
		LOG_ERR("FOTA URL has no file path: %s", url);
		return -EINVAL;
	}

	size_t host_len = (size_t)(slash - url); /* includes scheme */

	if (host_len >= sizeof(fota_host) || strlen(slash + 1) >= sizeof(fota_file)) {
		LOG_ERR("FOTA URL too long");
		return -ENAMETOOLONG;
	}

	memcpy(fota_host, url, host_len);
	fota_host[host_len] = '\0';
	strcpy(fota_file, slash + 1); /* length checked above */

	*sec_tag_out = https ? FOTA_SEC_TAG : -1;
	return 0;
}

int bringup_fota_start_url(const char *url)
{
	int sec_tag;
	int err;

	if (!fota_inited) {
		err = bringup_fota_init();
		if (err) {
			return err;
		}
	}
	if (fota_busy) {
		LOG_WRN("FOTA already in progress — ignoring new request");
		return -EALREADY;
	}

	err = split_url(url, &sec_tag);
	if (err) {
		return err;
	}

	LOG_INF("Starting FOTA: host=%s file=%s sec_tag=%d",
		fota_host, fota_file, sec_tag);

	fota_busy = true;
	/* pdn_id 0 = default PDN; fragment_size 0 = library default. */
	err = fota_download_start(fota_host, fota_file, sec_tag, 0, 0);
	if (err) {
		LOG_ERR("fota_download_start failed: %d", err);
		fota_busy = false;
		queue_status("start_failed");
		return err;
	}
	queue_status("started");
	return 0;
}
