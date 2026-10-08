/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set the minimum report interval used while reporting over BLE.
 *
 * Replaces CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN_BLE at runtime. The value is a
 * floor: with CONFIG_PMW3610_ALT_REPORT_INTERVAL_FOLLOW_CONN the negotiated
 * connection interval can still raise it. 0 disables BLE throttling. The USB
 * interval is not affected. Takes effect on the next sensor frame and is safe
 * to call from any thread, before or after the sensor has finished its init.
 *
 * CONFIG_PMW3610_ALT_REPORT_INTERVAL_RUNTIME is defined whenever this API exists.
 *
 * @param ms Minimum time between two reports, in milliseconds.
 */
void pmw3610_alt_set_ble_report_interval(uint16_t ms);

/** @brief Current BLE report interval floor, in milliseconds. */
uint16_t pmw3610_alt_get_ble_report_interval(void);

/** @brief Boot value of the floor, i.e. CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN_BLE. */
uint16_t pmw3610_alt_get_ble_report_interval_default(void);

#ifdef __cplusplus
}
#endif
