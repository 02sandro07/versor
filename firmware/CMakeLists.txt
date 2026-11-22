#
# Copyright (c) 2026 Cadenced
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(versor)

target_sources(app PRIVATE
	src/main.c
	src/imu.c
	src/temp.c
	src/cell.c
	src/gnss.c
	src/mqtt.c
	src/credentials.c
	src/modemdiag.c
	src/devcfg.c
	src/pmic.c
	src/telemetry.c
	src/shipment.c
)

# FOTA (self-hosted HTTP(S) update via MCUboot dual-slot). Only compiled when
# the fota_download pipeline is enabled (CONFIG_FOTA_DOWNLOAD in prj.conf +
# SB_CONFIG_BOOTLOADER_MCUBOOT in sysbuild.conf).
target_sources_ifdef(CONFIG_FOTA_DOWNLOAD app PRIVATE src/fota.c)

# A-GNSS assistance backend (nRF Cloud REST). Only compiled when enabled.
target_sources_ifdef(CONFIG_NRF_CLOUD_AGNSS app PRIVATE src/assistance.c)

# nRF Cloud device provisioning ("check-in") client. Only when enabled.
target_sources_ifdef(CONFIG_NRF_PROVISIONING app PRIVATE src/provision.c)

# Turn the broker CA (PEM) into a byte-array include for credentials.c,
# exactly like the production firmware does. Written to the build tree.
set(gen_dir ${CMAKE_CURRENT_BINARY_DIR}/certs)
generate_inc_file_for_target(app src/credentials/ca-cert.pem ${gen_dir}/ca-cert.pem)
target_include_directories(app PRIVATE ${gen_dir})
