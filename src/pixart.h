#pragma once

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/* device data structure */
struct pixart_data {
    const struct device          *dev;
    int64_t                      dx;
    int64_t                      dy;
    int64_t                      last_smp_time;
    int64_t                      last_rpt_time;
    bool                         sw_smart_flag; // for pmw3610 smart algorithm

    struct gpio_callback         irq_gpio_cb; // motion pin irq callback
    struct k_work                trigger_work; // realtrigger job

    struct k_work_delayable      init_work; // the work structure for delayable init steps
    int                          async_init_step;

    bool                         ready; // whether init is finished successfully
    int                          err; // error code during async init

    /* --- frame-rate diagnostics --- */
    volatile uint32_t            irq_ticks;        // cycle stamp taken in the motion ISR
    uint32_t                     prev_cb;          // cycle stamp of the previous callback
    uint32_t                     delta_buckets[4]; // inter-frame interval histogram
    uint32_t                     sched_worst_us;   // worst ISR -> callback latency
    uint32_t                     work_worst_us;    // worst callback duration
    uint32_t                     min_delta_us;     // shortest inter-frame gap in window
    struct k_work_delayable      diag_work;        // periodic histogram dump

    /* --- frame-rate watchdog --- */
    uint8_t                      stuck_streak;     // consecutive suspicious windows
    int64_t                      last_reinit_ms;   // rate limit for re-init
};

// device config data structure
struct pixart_config {
	struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    uint16_t cpi;
    bool swap_xy;
    bool inv_x;
    bool inv_y;
    uint8_t evt_type;
    uint8_t x_input_code;
    uint8_t y_input_code;
    bool force_awake;
    bool force_awake_4ms_mode;
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
