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
    uint32_t                     rpt_drops;        // input_report() rejected by a full queue
    uint32_t                     smart_toggles;    // smart-algorithm mode flips in window
    uint8_t                      perf_shadow;      // last PERFORMANCE value this driver wrote
    uint32_t                     perf_drift;       // times it was found changed behind our back
    uint8_t                      drift_was;        // last drift: value we had written
    uint8_t                      drift_found;      // last drift: value actually read back
    uint32_t                     drift_at_s;       // last drift: uptime in seconds
    uint32_t                     recovery_at_s;    // last successful init: uptime in seconds
    uint32_t                     recovery_retries; // attempts that init took
    uint32_t                     fast_frames;      // consecutive impossibly short frame gaps
    bool                         storm_detected;   // irq line is ringing, not reporting motion
    int64_t                      ready_since_ms;   // uptime when init last completed
    uint32_t                     failed_recoveries;// re-inits that fell apart again
    bool                         discard_frame;    // drop the first motion frame after init
    bool                         reset_suspected;  // a read returned the power-up default
    uint32_t                     init_retries;     // consecutive failed init attempts
    bool                         fault_active;     // a fault was already reported, do not recount
    uint32_t                     unresolved;       // consecutive windows the fault survived
    uint32_t                     cpi_wanted;       // CPI set at runtime; 0 = use the devicetree value
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
