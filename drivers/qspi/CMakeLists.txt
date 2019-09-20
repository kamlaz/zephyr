# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_QSPI		qspi_nrfx_qspi.c)

zephyr_library_sources_ifdef(CONFIG_USERSPACE		qspi_handlers.c)
