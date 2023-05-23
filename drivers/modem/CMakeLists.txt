#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

if (CONFIG_MODEM_NRF_SLM)
	zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/net/ip)
    zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/modem)
	zephyr_library_sources(nrf_slm.c)
endif()