/*
 * Copyright (c) 2024 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#ifndef CDC_INCO_ASSISTANCE_H_
#define CDC_INCO_ASSISTANCE_H_

#include <nrf_modem_gnss.h>

/* Initialise the assistance backend. Call once before assistance_request(). */
int assistance_init(struct k_work_q *assistance_work_q);

/* Fetch A-GNSS assistance for the satellites the modem asked for and inject it
 * into the GNSS core. Runs a one-shot HTTPS REST request to nRF Cloud,
 * authenticated with the device JWT. Requires an active LTE connection + valid
 * network time. Returns 0 on success.
 */
int assistance_request(struct nrf_modem_gnss_agnss_data_frame *agnss_request);

#endif /* CDC_INCO_ASSISTANCE_H_ */
