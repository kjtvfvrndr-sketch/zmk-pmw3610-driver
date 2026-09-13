/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610_alt

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zmk/keymap.h>
#include <zmk/event_manager.h>
#include <zmk/endpoints.h>
#include <zmk/activity.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/endpoint_changed.h>
#if IS_ENABLED(CONFIG_ZMK_USB)
#include <zmk/usb.h>
#include <zmk/events/usb_conn_state_changed.h>
#endif
#if IS_ENABLED(CONFIG_BT)
#include <zephyr/bluetooth/conn.h>
#endif
#include "pmw3610.h"

#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_ALT_LOG_LEVEL);

/* Frame-rate diagnostics + dedicated work queue.
 *
 * The motion interrupt is re-armed only after the work item has finished, so
 * if scheduling latency plus work duration exceed the sensor frame period the
 * next frame is missed outright and the effective rate halves.  Run the sensor
 * work on its own cooperative queue (priority -2, just above the system work
 * queue at -1) and measure the inter-frame interval directly.
 */
#define PMW3610_DIAG_PRIO       (-2)
#define PMW3610_DIAG_STACK_SIZE 1024
#define PMW3610_DIAG_PERIOD     K_SECONDS(2)

/* Frame-rate watchdog.
 *
 * A stuck sensor halves its frame rate, so the shortest possible gap becomes
 * 8 ms and the 4 ms bucket empties completely.  Slow ball movement also
 * produces long gaps, but any burst of quick motion still lands in the 4 ms
 * bucket - in eight reference windows with n >= 15 the 4 ms bucket was never
 * empty (31/33, 89/91, 45/53, 43/46, 13/15, 40/41).  Require several
 * consecutive empty windows so a slow stretch cannot trip it.
 */
#define PMW3610_WD_MIN_SAMPLES  15
#define PMW3610_WD_STREAK       5
#define PMW3610_WD_HOLDOFF_MS   60000

/* PERFORMANCE holds two unrelated things: the force-awake bits in the high
 * nibble and the run-rate fields in the low one. The run rate has been seen
 * reverting to the power-up 0x01 at runtime, and read-modify-preserve then
 * writes that back forever. So the wanted rate lives here and is re-asserted
 * on every write instead of being inherited from the register. */
#define PMW3610_PERF_FORCE_AWAKE 0xF0
#define PMW3610_PERF_RUN_RATE_MASK 0x0F
/* bit3 VEL, bit2 POSHI, bits1-0 POSLO -- all at 4 ms */
#define PMW3610_PERF_RUN_RATE 0x0d
/* A frame gap this short cannot come from a part sampling at 4 ms. It means
 * the interrupt line is floating rather than being driven, which is what a
 * module off its pogo pins looks like: the healthy minimum sits around
 * 2.6 ms, a floating line rings at a few hundred microseconds. Requiring a
 * run of them keeps a single bunched-up pair of frames from counting. */
#define PMW3610_IMPLAUSIBLE_GAP_US 1500
#define PMW3610_STORM_FRAMES 32

/* Value PERFORMANCE holds after a power-up reset of the part. */
#define PMW3610_PERF_POWER_UP 0x01
/* A read of all ones means nobody is driving MISO -- the sensor is not
 * answering at all, which on a pogo-pin module means lost contact. */
#define PMW3610_BUS_SILENT 0xFF

/* The reset detector works by spotting the power-up value in PERFORMANCE,
 * so the rate this driver writes must never be that value. Only POSLO can
 * go below 4 ms anyway, so there is no reason to pick anything else. */
BUILD_ASSERT(PMW3610_PERF_RUN_RATE != PMW3610_PERF_POWER_UP,
             "the configured run rate must differ from the power-up default, "
             "otherwise a sensor reset cannot be told apart from a healthy part");

/* Recovery policy lives in Kconfig, not here: these are the knobs someone
 * might actually want to turn, unlike the register values above. */
#define PMW3610_UNRESOLVED_MAX CONFIG_PMW3610_ALT_UNRESOLVED_MAX
#define PMW3610_INIT_RETRY_FAST_TRIES CONFIG_PMW3610_ALT_INIT_RETRY_FAST_TRIES
#define PMW3610_INIT_RETRY_FAST_MS CONFIG_PMW3610_ALT_INIT_RETRY_FAST_MS
#define PMW3610_INIT_RETRY_SLOW_MS CONFIG_PMW3610_ALT_INIT_RETRY_SLOW_MS

/* A re-init that falls apart again this soon did not fix anything. After a
 * few of those the contact is marginal rather than momentarily lost, and
 * repeating at full speed only produces a cursor that jumps once every
 * couple of seconds forever. */
#define PMW3610_RECOVERY_STICK_MS 10000
#define PMW3610_RECOVERY_PATIENCE 2

/* Accumulated but unreported delta is dropped once it is older than this.
 * Tied to perception, not to the report interval: below it the residue is the
 * tail of the stroke still in progress, above it emitting it reads as a jump. */
#define PMW3610_STALE_DELTA_MS 50

/* Dead band around the smart-algorithm shutter threshold of 45. */
#define PMW3610_SMART_HYST 5

/* k_cyc_to_us_near32() overflows a few milliseconds in at 32768 Hz - go 64-bit. */
#define PMW3610_CYC_TO_US(c) ((uint32_t)k_cyc_to_us_near64((uint64_t)(uint32_t)(c)))

K_THREAD_STACK_DEFINE(pmw3610_diag_stack, PMW3610_DIAG_STACK_SIZE);
static struct k_work_q pmw3610_workq;
static bool pmw3610_workq_started;

//////// Report interval, adapted to the active endpoint //////////
// USB and BLE cap the achievable report rate very differently,   //
// and BLE hosts disagree among themselves (7.5 ms on Windows,    //
// 15 ms on macOS). Follow the active endpoint and, on BLE, the   //
// connection interval actually negotiated with the host, so the  //
// driver never produces more reports than the link can carry.    //

static atomic_t rpt_interval_min = ATOMIC_INIT(CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN_BLE);
static atomic_t ble_interval_min = ATOMIC_INIT(CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN_BLE);

static void pmw3610_refresh_report_interval(void) {
    int32_t v = atomic_get(&ble_interval_min);

#if IS_ENABLED(CONFIG_ZMK_USB)
    if (zmk_endpoints_selected().transport == ZMK_TRANSPORT_USB) {
        v = CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN_USB;
    }
#endif

    if (atomic_set(&rpt_interval_min, v) != v) {
        LOG_INF("Report interval min -> %d ms", v);
    }
}

#if IS_ENABLED(CONFIG_BT) && IS_ENABLED(CONFIG_PMW3610_ALT_REPORT_INTERVAL_FOLLOW_CONN)
static void pmw3610_track_conn(struct bt_conn *conn) {
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info) != 0 || info.type != BT_CONN_TYPE_LE) {
        return;
    }

    /* Only the host link matters. On a split central the other LE connection
       is the peripheral half, and its interval must not be picked up here. */
    if (info.role != BT_CONN_ROLE_PERIPHERAL) {
        return;
    }

    /* le.interval is in 1.25 ms units; round up so the driver never emits
       reports faster than the link is able to carry them. */
    int32_t ms = ((int32_t)info.le.interval * 5 + 3) / 4;
    atomic_set(&ble_interval_min, MAX(CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN_BLE, ms));
    pmw3610_refresh_report_interval();
}

static void pmw3610_connected(struct bt_conn *conn, uint8_t err) {
    /* le_param_updated only fires on an actual update procedure, which some
       hosts never run, so sample the interval on connect as well. */
    if (err == 0) {
        pmw3610_track_conn(conn);
    }
}

static void pmw3610_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                                     uint16_t timeout) {
    ARG_UNUSED(interval);
    ARG_UNUSED(latency);
    ARG_UNUSED(timeout);
    pmw3610_track_conn(conn);
}

BT_CONN_CB_DEFINE(pmw3610_conn_cb) = {
    .connected = pmw3610_connected,
    .le_param_updated = pmw3610_le_param_updated,
};
#endif

static int pmw3610_endpoint_listener(const zmk_event_t *eh) {
    if (as_zmk_endpoint_changed(eh) != NULL) {
        pmw3610_refresh_report_interval();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(pmw3610_endpoint, pmw3610_endpoint_listener);
ZMK_SUBSCRIPTION(pmw3610_endpoint, zmk_endpoint_changed);

//////// Force-awake while the board runs on USB power ////////
// On battery, letting the sensor downshift into REST1 is the  //
// whole point of the downshift timing. On USB there is no     //
// power to save, so the sensor is pinned in RUN and the        //
// wake-up lag on the first motion after a pause disappears.    //

static bool pmw3610_want_force_awake(const struct device *dev) {
    const struct pixart_config *config = dev->config;

    /* The devicetree property keeps its original meaning: force whenever
       active, on any transport. It can hold on battery, so idle releases it. */
    if (config->force_awake) {
        return zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE;
    }

#if IS_ENABLED(CONFIG_PMW3610_ALT_FORCE_AWAKE_ON_USB_POWER) && IS_ENABLED(CONFIG_ZMK_USB)
    /* Cable presence, not the selected endpoint: what makes REST pointless is
       mains power, and that is there whichever output happens to be active.
       Not gated on the activity state either -- idle saves nothing on mains,
       while releasing the force on idle would put the lag back on the first
       stroke after every return. It cannot carry into deep sleep regardless:
       is_usb_power_present() blocks ZMK_ACTIVITY_SLEEP while the cable is in. */
    return zmk_usb_is_powered();
#else
    return false;
#endif
}

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // reset cs line and assert power-up reset
    ASYNC_INIT_STEP_CLEAR_OB1, // clear observation1 register for self-test check
    ASYNC_INIT_STEP_CHECK_OB1, // check the value of observation1 register after self-test check
    ASYNC_INIT_STEP_CONFIGURE, // set other registes like cpi and donwshift time (run, rest1, rest2)
                               // and clear motion registers

    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10 + CONFIG_PMW3610_ALT_INIT_POWER_UP_EXTRA_DELAY_MS, // >10ms needed
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200, // 150 us required, test shows too short,
                                       // also power-up reset is added in this step, thus using 50 ms
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,  // 10 ms required in spec,
                                       // test shows too short,
                                       // especially when integrated with display,
                                       // > 50ms is needed
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Function definitions //////////

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value, uint8_t len) {
	const struct pixart_config *cfg = dev->config;
	const struct spi_buf tx_buf = { .buf = &addr, .len = sizeof(addr) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf rx_buf[] = {
		{ .buf = NULL, .len = sizeof(addr), },
		{ .buf = value, .len = len, },
	};
	const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };
	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int pmw3610_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
	return pmw3610_read(dev, addr, value, 1);
}

static int pmw3610_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
	const struct pixart_config *cfg = dev->config;
	uint8_t write_buf[] = {addr | SPI_WRITE_BIT, value};
	const struct spi_buf tx_buf = { .buf = write_buf, .len = sizeof(write_buf), };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1, };
	return spi_write_dt(&cfg->spi, &tx);
}

static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val) {
	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    int err = pmw3610_write_reg(dev, reg, val);

    /* The clock request is released on the error path too. It is best effort:
       if the bus is broken this write fails as well, but leaving the sensor
       clock forced on is the one outcome worth ruling out. */
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    return err;
}

/* Пишет регистр и перечитывает его. Сенсор может ещё не закончить
   внутренний power-up, тогда запись теряется молча. */
static int pmw3610_write_verified(const struct device *dev, uint8_t reg, uint8_t val) {
    for (int attempt = 1; attempt <= 3; attempt++) {
        int err = pmw3610_write(dev, reg, val);
        if (err) {
            LOG_ERR("reg 0x%02x: write failed %d (attempt %d)", reg, err, attempt);
            return err;
        }

        uint8_t rb = 0xFF;
        err = pmw3610_read_reg(dev, reg, &rb);
        if (err) {
            LOG_ERR("reg 0x%02x: readback failed %d", reg, err);
            return err;
        }

        if (rb == val) {
            if (attempt > 1) {
                LOG_WRN("reg 0x%02x: settled as 0x%02x on attempt %d", reg, rb, attempt);
            }
            return 0;
        }

        LOG_ERR("reg 0x%02x: wrote 0x%02x, read back 0x%02x (attempt %d)", reg, val, rb, attempt);
        k_msleep(10);
    }
    return -EIO;
}

static int pmw3610_set_cpi(const struct device *dev, uint32_t cpi, 
                           bool swap_xy, bool inv_x, bool inv_y) {
    /* Set resolution with CPI step of 200 cpi
     * 0x1: 200 cpi (minimum cpi)
     * 0x2: 400 cpi
     * 0x3: 600 cpi
     * :
     */

    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    uint8_t value = 0x00;
    int err = 0;

    LOG_INF("Setting cpi: %d", cpi);
    // Convert CPI to register value
    // Set prefered RES_STEP
    //   BIT 4-0: CPI
    uint8_t cpi_val = cpi / 200;
    value = (value & 0xE0) | (cpi_val & 0x1F);

    // Convert axis to register value
    // Set prefered RES_STEP
    //   BIT 7: SWAP_XY
    //   BIT 6: INV_X
    //   BIT 5: INV_Y
    LOG_INF("Setting axis swap_xy: %s inv_x: %s inv_y: %s", 
            swap_xy ? "yes" : "no", inv_x ? "yes" : "no", inv_y ? "yes" : "no");

#if IS_ENABLED(CONFIG_PMW3610_ALT_SWAP_XY)
    value |= (1 << 7);
#else
    if (swap_xy) { value |= (1 << 7); } else { value &= ~(1 << 7); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_X)
    value |= (1 << 6);
#else
    if (inv_x) { value |= (1 << 6); } else { value &= ~(1 << 6); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_Y)
    value |= (1 << 5);
#else
    if (inv_y) { value |= (1 << 5); } else { value &= ~(1 << 5); }
#endif

    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    /* set the cpi */
    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value,                0x00};

	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    /* Write data */
    for (size_t i = 0; i < sizeof(data); i++) {
        err = pmw3610_write_reg(dev, addr[i], data[i]);
        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            break;
        }
    }
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

    if (err) {
        LOG_ERR("Failed to set CPI");
        return err;
    }

    return 0;
}

/* Set sampling rate in each mode (in ms) */
static int pmw3610_set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
        return -EINVAL;
    }

    uint8_t value = sample_time / mintime;
    LOG_INF("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

    /* The sample time is (reg_value * mintime ) ms. 0x00 is rounded to 0x1 */
    int err = pmw3610_write_verified(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }

    return err;
}

/* Set downshift time in ms. */
// NOTE: The unit of run-mode downshift is related to pos mode rate, which is hard coded to be 4 ms
// The pos-mode rate is configured in pmw3610_async_init_configure
/* The REST downshift ranges below are derived from the matching sample time,
 * so a plausible-looking pair of Kconfig values can be rejected at runtime --
 * which aborts async init and leaves the sensor dead with only a log line to
 * say why. All four values are compile-time constants, so check them here. */
BUILD_ASSERT(CONFIG_PMW3610_ALT_RUN_DOWNSHIFT_TIME_MS >= 32 &&
                 CONFIG_PMW3610_ALT_RUN_DOWNSHIFT_TIME_MS <= 8160,
             "PMW3610_ALT_RUN_DOWNSHIFT_TIME_MS must be within 32..8160 ms");

BUILD_ASSERT(CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS >= 10 &&
                 CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS <= 2550,
             "PMW3610_ALT_REST1_SAMPLE_TIME_MS must be within 10..2550 ms");
BUILD_ASSERT(CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS >= 10 &&
                 CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS <= 2550,
             "PMW3610_ALT_REST2_SAMPLE_TIME_MS must be within 10..2550 ms");
BUILD_ASSERT(CONFIG_PMW3610_ALT_REST3_SAMPLE_TIME_MS >= 10 &&
                 CONFIG_PMW3610_ALT_REST3_SAMPLE_TIME_MS <= 2550,
             "PMW3610_ALT_REST3_SAMPLE_TIME_MS must be within 10..2550 ms");

BUILD_ASSERT(CONFIG_PMW3610_ALT_REST1_DOWNSHIFT_TIME_MS >=
                 16 * CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS,
             "PMW3610_ALT_REST1_DOWNSHIFT_TIME_MS must be at least 16x "
             "PMW3610_ALT_REST1_SAMPLE_TIME_MS -- raise the downshift or lower the sample time");
BUILD_ASSERT(CONFIG_PMW3610_ALT_REST1_DOWNSHIFT_TIME_MS <=
                 255 * 16 * CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS,
             "PMW3610_ALT_REST1_DOWNSHIFT_TIME_MS must be at most 4080x "
             "PMW3610_ALT_REST1_SAMPLE_TIME_MS");

BUILD_ASSERT(CONFIG_PMW3610_ALT_REST2_DOWNSHIFT_TIME_MS >=
                 128 * CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS,
             "PMW3610_ALT_REST2_DOWNSHIFT_TIME_MS must be at least 128x "
             "PMW3610_ALT_REST2_SAMPLE_TIME_MS -- raise the downshift or lower the sample time");
BUILD_ASSERT(CONFIG_PMW3610_ALT_REST2_DOWNSHIFT_TIME_MS <=
                 255 * 128 * CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS,
             "PMW3610_ALT_REST2_DOWNSHIFT_TIME_MS must be at most 32640x "
             "PMW3610_ALT_REST2_SAMPLE_TIME_MS");

static int pmw3610_set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        /*
         * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                      * 8 * pos-rate (fixed to 4ms)
         */
        maxtime = 8160; // 32 * 255;
        mintime = 32; // hard-coded in pmw3610_async_init_configure
        break;

    case PMW3610_REG_REST1_DOWNSHIFT:
        /*
         * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                        * 16 * Rest1_sample_period (default 40 ms)
         */
        maxtime = 255 * 16 * CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS;
        break;

    case PMW3610_REG_REST2_DOWNSHIFT:
        /*
         * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT
         *                        * 128 * Rest2 rate (default 100 ms)
         */
        maxtime = 255 * 128 * CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS;
        break;

    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range (%u - %u)", time, mintime, maxtime);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    /* Convert time to register value */
    uint8_t value = time / mintime;

    LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

    int err = pmw3610_write_verified(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

static int pmw3610_set_performance(const struct device *dev, bool enabled) {
    const struct pixart_config *config = dev->config;
    int err = 0;

    if (config->force_awake || IS_ENABLED(CONFIG_PMW3610_ALT_FORCE_AWAKE_ON_USB_POWER)) {
        uint8_t value;
        err = pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &value);
        if (err) {
            LOG_ERR("Can't read ref-performance %d", err);
            return err;
        }
        LOG_DBG("Get performance register (reg value 0x%x)", value);

        // Set prefered RUN RATE        
        //   BIT 3:   VEL_RUNRATE    0x0: 8ms; 0x1 4ms;
        //   BIT 2:   POSHI_RUN_RATE 0x0: 8ms; 0x1 4ms;
        //   BIT 1-0: POSLO_RUN_RATE 0x0: 8ms; 0x1 4ms; 0x2 2ms; 0x4 Reserved
        /* The run rate is asserted, not inherited: whatever the low nibble
           currently holds is not authoritative. */
        uint8_t perf = PMW3610_PERF_RUN_RATE;

        if (enabled) {
            perf |= PMW3610_PERF_FORCE_AWAKE;
        }

        if ((value & PMW3610_PERF_RUN_RATE_MASK) != PMW3610_PERF_RUN_RATE) {
            LOG_WRN("PERFORMANCE run rate was 0x%02x, expected 0x%02x -- re-asserting",
                    value & PMW3610_PERF_RUN_RATE_MASK, PMW3610_PERF_RUN_RATE);
        }

        if (value == PMW3610_PERF_POWER_UP) {
            /* Reading the power-up default here means the part reset since the
               last write. Writing our value on top would hide that from the
               drift check, so flag it and let the diag work re-initialise --
               calling reinit from here would recurse through async init. */
            struct pixart_data *rd = dev->data;
            rd->reset_suspected = true;
            LOG_WRN("PERFORMANCE read as power-up default -- sensor reset suspected");
        }

        if (perf != value) {
            /* Verified: a silently lost write here means force-awake does not
               work at all, and the read-back also proves the high nibble is
               writable on this part. */
            err = pmw3610_write_verified(dev, PMW3610_REG_PERFORMANCE, perf);
            if (err) {
                LOG_ERR("Can't write performance register %d", err);
                return err;
            }
            LOG_INF("%s performance mode (reg 0x%02x -> 0x%02x)",
                    enabled ? "enable" : "disable", value, perf);
        }

        struct pixart_data *data = dev->data;
        data->perf_shadow = perf;
    }

    return err;
}

static int pmw3610_set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
    return ret;
}

static int pmw3610_async_init_power_up(const struct device *dev) {
	int ret = pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    return pmw3610_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    uint8_t value;
    int err = pmw3610_read_reg(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Can't do self-test");
        return err;
    }

    if ((value & 0x0F) != 0x0F) {
        LOG_ERR("Failed self-test (0x%x)", value);
        return -EINVAL;
    }

    uint8_t product_id = 0x01;
    err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }

    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    int err = 0;
    const struct pixart_config *config = dev->config;
    struct pixart_data *data = dev->data;

    // clear motion registers first (required in datasheet)
    for (uint8_t reg = 0x02; (reg <= 0x05) && !err; reg++) {
        uint8_t buf[1];
        err = pmw3610_read_reg(dev, reg, buf);
    }

    if (!err) {
        err = pmw3610_set_cpi(dev, data->cpi_wanted ? data->cpi_wanted : config->cpi,
                              config->swap_xy, config->inv_x, config->inv_y);
    }

	/* Апстрим inorichi/zmk-pmw3610-driver пишет PERFORMANCE безусловно:
     * старший ниббл (force-mode) = 0x0, младший = 0x0d. Это же значение
     * пишет штатная прошивка RMK. Форки Ergohaven/badjeff спрятали запись
     * под `if (config->force_awake)`, и без него регистр не трогается вообще. */
    if (!err) {
        uint8_t old = 0xFF;
        pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &old);
        LOG_INF("Performance register: 0x%02x -> 0x0d", old);
        err = pmw3610_write_verified(dev, PMW3610_REG_PERFORMANCE, PMW3610_PERF_RUN_RATE);
        if (!err) {
            /* Seed the shadow here too, so drift detection keeps working even
               when the force-awake option is off and set_performance is a no-op. */
            data->perf_shadow = PMW3610_PERF_RUN_RATE;
        }
    }

    /* Force-awake lives in the high nibble of the same register, so it has to
       be applied after the 0x0d write above, not before it. */
    if (!err) {
        err = pmw3610_set_performance(dev, pmw3610_want_force_awake(dev));
    }
	
    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                         CONFIG_PMW3610_ALT_RUN_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                         CONFIG_PMW3610_ALT_REST1_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                         CONFIG_PMW3610_ALT_REST2_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE,
                                      CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE,
                                      CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE,
                                      CONFIG_PMW3610_ALT_REST3_SAMPLE_TIME_MS);
    }

    if (err) {
        LOG_ERR("Config the sensor failed");
        return err;
    }

    /* The endpoint-changed event only fires on a later switch, so pick up
       whichever endpoint is already active at boot. */
    pmw3610_refresh_report_interval();

    return 0;
}

static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    /* Quiet on retries: with the module off this runs every few seconds
       forever, and the first failure already said what happened. */
    if (data->init_retries == 0) {
        LOG_INF("PMW3610 async init step %d", data->async_init_step);
    } else {
        LOG_DBG("PMW3610 async init step %d (retry %u)", data->async_init_step,
                data->init_retries);
    }

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        /* Retry rather than stop. On a pogo-pin module the usual cause is a
           contact that is still bouncing -- and a re-init triggered by that very
           bounce would otherwise leave ready=false forever, which also silences
           the diagnostics that would have noticed. Retrying forever costs a few
           SPI transactions every few seconds and makes re-seating the module
           recover on its own. */
        int failed_step = data->async_init_step;
        data->init_retries++;
        data->async_init_step = 0;

        uint32_t delay = data->init_retries <= PMW3610_INIT_RETRY_FAST_TRIES
                             ? PMW3610_INIT_RETRY_FAST_MS
                             : PMW3610_INIT_RETRY_SLOW_MS;

        if (data->init_retries == 1) {
            LOG_ERR("PMW3610 init failed in step %d -- retrying", failed_step);
        } else {
            LOG_DBG("PMW3610 init retry %u", data->init_retries);
        }

        k_work_schedule_for_queue(&pmw3610_workq, &data->init_work, K_MSEC(delay));
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true; // sensor is ready to work
            /* Cleared on success, not only where it is raised: a suspicion
               re-raised during the init sequence itself would otherwise keep
               triggering re-inits forever. */
            data->reset_suspected = false;
            /* Recorded, not only logged: recovery often happens while the cable
               is being plugged in, so the messages above go to a CDC backend
               that is not up yet. This rides along in the diag line instead and
               says how long the part was down and how many attempts it took. */
            data->recovery_at_s = (uint32_t)(k_uptime_get() / 1000);
            data->recovery_retries = data->init_retries;
            data->ready_since_ms = k_uptime_get();
            data->discard_frame = true;

            if (data->init_retries) {
                LOG_WRN("PMW3610 initialized after %u retries", data->init_retries);
            }
            data->init_retries = 0;
            LOG_INF("PMW3610 initialized");
            pmw3610_set_interrupt(dev, true);
        } else {
            k_work_schedule_for_queue(&pmw3610_workq, &data->init_work,
                                      K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    uint8_t buf[PMW3610_BURST_SIZE];

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    const int32_t rpt_min = atomic_get(&rpt_interval_min);
    const int64_t now = k_uptime_get();

	int err = pmw3610_read(dev, PMW3610_REG_MOTION_BURST, buf, PMW3610_BURST_SIZE);
    if (err) {
        return err;
    }

    if (unlikely(data->discard_frame)) {
        /* Whatever the part accumulated while it was being reconfigured lands
           in this first frame and reads as one jump across the screen. The
           burst read above has already cleared it out of the sensor; throw
           the value away rather than reporting it. */
        data->discard_frame = false;
        data->dx = 0;
        data->dy = 0;
        data->last_smp_time = now;
        return 0;
    }
    // LOG_HEXDUMP_DBG(buf, PMW3610_BURST_SIZE, "buf");

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

    int16_t x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
    int16_t y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);
    LOG_DBG("x/y: %d/%d", x, y);

#ifdef CONFIG_PMW3610_ALT_SMART_ALGORITHM
    int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) 
                    + buf[PMW3610_SHUTTER_L_POS];
    /* Pixart's reference code flips on a single threshold of 45.  This write
       costs three SPI transfers and a 300 us sleep inside the frame budget,
       and the motion interrupt is re-armed only after the work item returns,
       so a shutter value sitting on the threshold can flip every frame and
       cost frames outright.  A dead band around 45 removes that case; the
       counter below says whether it was ever happening. */
    if (data->sw_smart_flag && shutter < (45 - PMW3610_SMART_HYST)) {
        pmw3610_write(dev, 0x32, 0x00);
        data->sw_smart_flag = false;
        data->smart_toggles++;
    }
    if (!data->sw_smart_flag && shutter > (45 + PMW3610_SMART_HYST)) {
        pmw3610_write(dev, 0x32, 0x80);
        data->sw_smart_flag = true;
        data->smart_toggles++;
    }
#endif

    // purge accumulated delta once it is stale enough that emitting it would
    // read as a jump rather than as the tail of the current stroke.  The old
    // threshold was rpt_min itself, which also discarded the residue of any
    // normal stroke the moment the frames spaced out past one report interval.
    if (rpt_min > 0 && (now - data->last_smp_time) >= PMW3610_STALE_DELTA_MS) {
        data->dx = 0;
        data->dy = 0;
    }
    data->last_smp_time = now;

    // accumulate delta until report in next iteration
    data->dx += x;
    data->dy += y;

    // strict to report inerval
    if (rpt_min > 0 && (now - data->last_rpt_time) < rpt_min) {
        return 0;
    }

    // fetch report value
    int16_t rx = (int16_t)CLAMP(data->dx, INT16_MIN, INT16_MAX);
    int16_t ry = (int16_t)CLAMP(data->dy, INT16_MIN, INT16_MAX);
    bool have_x = rx != 0;
    bool have_y = ry != 0;

    if (have_x || have_y) {
        data->last_rpt_time = now;
        data->dx = 0;
        data->dy = 0;
        /* K_NO_WAIT: a full input queue drops the event and returns -EAGAIN.
           The frame histogram above only measures the sensor side, so without
           this counter a report lost between the driver and the listeners is
           invisible.  Counted, not retried: pushing the residue back would
           turn a small loss into a jump once the queue drains, and it is not
           yet known whether this ever fires. */
        if (have_x && input_report(dev, config->evt_type, config->x_input_code, rx,
                                   !have_y, K_NO_WAIT)) {
            data->rpt_drops++;
        }
        if (have_y && input_report(dev, config->evt_type, config->y_input_code, ry,
                                   true, K_NO_WAIT)) {
            data->rpt_drops++;
        }
    }

    return err;
}

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    pmw3610_set_interrupt(dev, false);
    data->irq_ticks = k_cycle_get_32();
    k_work_submit_to_queue(&pmw3610_workq, &data->trigger_work);
}

/* Re-run the async init state machine.  A full reboot is known to clear the
 * stuck frame rate, and this is what a reboot does to the sensor.  The
 * interrupt is disarmed first and data->ready gates pmw3610_report_data(), so
 * motion work cannot touch the SPI bus while the sequence runs. */
static void pmw3610_reinit(struct pixart_data *data, uint32_t delay_ms) {
    pmw3610_set_interrupt(data->dev, false);
    data->ready = false;
    data->async_init_step = 0;
    data->init_retries = 0;
    /* One place for all of it: a re-init is the response to a fault, so it
       hands back a fresh retry budget and a fresh escalation budget. The
       incident flag is deliberately NOT cleared -- the fault only counts as
       over when a check actually passes, otherwise a re-init that does not
       help would book a new incident every couple of seconds. */
    data->unresolved = 0;
    data->reset_suspected = false;
    data->storm_detected = false;
    data->fast_frames = 0;
    data->last_reinit_ms = k_uptime_get();
    k_work_schedule_for_queue(&pmw3610_workq, &data->init_work,
                              K_MSEC(delay_ms + async_init_delay[0]));
}

/* The re-init entry point for faults found by the diagnostics: it notices a
 * recovery that did not hold and slows the next attempt down. The delay is
 * dead time, not garbage time -- pmw3610_reinit() disarms the interrupt
 * before it, so nothing reaches the host while the part is left alone. */
static void pmw3610_redo(struct pixart_data *data) {
    const bool stuck = data->ready_since_ms != 0 &&
                       (k_uptime_get() - data->ready_since_ms) <
                           PMW3610_RECOVERY_STICK_MS;

    if (stuck) {
        data->failed_recoveries++;
    } else {
        data->failed_recoveries = 0;
    }

    uint32_t delay = 0;

    if (data->failed_recoveries > PMW3610_RECOVERY_PATIENCE) {
        delay = PMW3610_INIT_RETRY_SLOW_MS;
        LOG_WRN("recovery has not held %u times -- backing off %u ms",
                data->failed_recoveries, delay);
    }

    pmw3610_reinit(data, delay);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;

    uint32_t t0 = k_cycle_get_32();
    uint32_t delta_us = PMW3610_CYC_TO_US(t0 - data->prev_cb);
    uint32_t sched_us = PMW3610_CYC_TO_US(t0 - data->irq_ticks);
    data->prev_cb = t0;

    /* A disconnected module leaves the interrupt line floating, and the driver
       would happily service thousands of phantom frames a second, flooding the
       input queue with garbage. Stop at the first run of impossible gaps and
       leave the interrupt disarmed until a re-init puts the part back. */
    if (delta_us < PMW3610_IMPLAUSIBLE_GAP_US) {
        data->fast_frames++;
    } else {
        data->fast_frames = 0;
    }

    if (data->fast_frames >= PMW3610_STORM_FRAMES && !data->storm_detected) {
        data->storm_detected = true;
        LOG_WRN("interrupt storm: %u frames under %u us -- line is floating",
                data->fast_frames, PMW3610_IMPLAUSIBLE_GAP_US);
    }

    if (!data->storm_detected) {
        pmw3610_report_data(dev);
    }

    /* Not unconditional: a re-init disarms the interrupt on purpose, and a
       trigger already sitting in the queue would otherwise put it straight
       back and rain interrupts through the reconfiguration. */
    if (data->ready && !data->storm_detected) {
        pmw3610_set_interrupt(dev, true);
    }

    uint32_t work_us = PMW3610_CYC_TO_US(k_cycle_get_32() - t0);

    /* The inter-frame interval is the primary observable: a missed frame shows
     * up as an exact doubling, never as an intermediate value. */
    if (delta_us < data->min_delta_us) {
        data->min_delta_us = delta_us;
    }

    if (delta_us < 6000) {
        data->delta_buckets[0]++;   /* 4 ms - nominal */
    } else if (delta_us < 10000) {
        data->delta_buckets[1]++;   /* 8 ms - one frame lost */
    } else if (delta_us < 20000) {
        data->delta_buckets[2]++;   /* 16 ms - two frames lost */
    } else {
        data->delta_buckets[3]++;   /* slower - idle gap or REST mode */
    }

    if (sched_us > data->sched_worst_us) {
        data->sched_worst_us = sched_us;
    }
    if (work_us > data->work_worst_us) {
        data->work_worst_us = work_us;
    }
}

/* Runs on the same queue as the motion work, so the SPI bus stays serialised.
 * Reading PERFORMANCE here rather than in pmw3610_report_data() keeps the SPI
 * transaction out of the hot path, where it would perturb what we measure. */
static void pmw3610_diag_dump(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pixart_data *data = CONTAINER_OF(dwork, struct pixart_data, diag_work);
    uint32_t *b = data->delta_buckets;
    uint32_t n = b[0] + b[1] + b[2] + b[3];

    const bool ready = data->ready;
    uint8_t perf = PMW3610_BUS_SILENT;

    /* Deliberately not gated on n. A part that reset into a state where it
       reports no motion at all would otherwise never be looked at again --
       no motion means no window, no window means no check, and the trackball
       stays silently dead until a reboot. Two register reads every couple of
       seconds cost nothing next to that. */
    if (ready) {
        pmw3610_read_reg(data->dev, PMW3610_REG_PERFORMANCE, &perf);

        const bool silent = perf == PMW3610_BUS_SILENT;
        const bool perf_lost = data->perf_shadow != 0 && perf != data->perf_shadow;

        /* One shadow, three outcomes. All ones means the part is not driving the
           bus at all. The power-up value means a glitch reset it, and PERFORMANCE
           is then the least of it -- CPI, every downshift time and every sample
           time are back at their defaults too, so only a full re-init fixes it.
           Anything else is a stray write and one register is enough to put back.

           CPI is deliberately not watched alongside: RES_STEP lives behind a
           0x7f bank switch, so a plain read of it always returns 0xff, and doing
           the bank switch here every two seconds would risk leaving the part in
           the wrong bank if it were interrupted on a flaky contact. PERFORMANCE
           catches the reset on its own. */
        const bool fault = perf_lost || data->reset_suspected || data->storm_detected;

        if (!fault) {
            data->fault_active = false;
            data->unresolved = 0;
        }

        if (fault) {
            /* Counted and logged on the rising edge only. The check now runs in
               every window, motion or not, so a module lying on the desk would
               otherwise add to the counter and to the log every two seconds and
               turn a count of incidents into a count of windows. */
            const bool first = !data->fault_active;

            data->fault_active = true;
            data->unresolved++;

            if (first) {
                /* One coherent snapshot per incident. Kept rather than only
                   logged, because a fault raised while the cable is out reaches
                   a dead CDC backend; these fields ride along in the diag line
                   instead and survive until the next reconnect. */
                data->perf_drift++;
                data->drift_was = data->perf_shadow;
                data->drift_found = perf;
                data->drift_at_s = (uint32_t)(k_uptime_get() / 1000);
            }
            if (data->unresolved > PMW3610_UNRESOLVED_MAX) {
                /* Whatever it was, nursing it has not worked: either the part
                   still is not answering, or the one-register repair keeps
                   failing. Re-init retries on its own and recovers once the
                   module is back, which beats spinning here forever. */
                LOG_WRN("fault unresolved for %u windows -- re-initialising",
                        data->unresolved);
                pmw3610_redo(data);
            } else if (silent || data->storm_detected) {
                /* Nothing is driving the bus, or the interrupt line is ringing.
                   Waiting this out was a mistake: a part that does not answer
                   will not start answering on its own, and every window spent
                   waiting is a window with a floating line still armed. */
                LOG_WRN("sensor not answering (drift #%u) -- re-initialising",
                        data->perf_drift);
                pmw3610_redo(data);
            } else if (perf == PMW3610_PERF_POWER_UP || data->reset_suspected) {
                /* The power-up default means the part reset, and PERFORMANCE is
                   then the least of it: CPI, every downshift time and every REST
                   sample time are back at their defaults too. Repairing this one
                   register would leave the rest silently wrong while the log went
                   back to looking healthy. */
                LOG_WRN("sensor reset detected (0x%02x -> 0x%02x, drift #%u) "
                        "-- re-initialising",
                        data->perf_shadow, perf, data->perf_drift);
                pmw3610_redo(data);
            } else {
                if (first) {
                    LOG_WRN("PERFORMANCE drifted: 0x%02x -> 0x%02x (drift #%u) "
                            "-- repairing",
                            data->perf_shadow, perf, data->perf_drift);
                }
                if (pmw3610_write_verified(data->dev, PMW3610_REG_PERFORMANCE,
                                           data->perf_shadow) == 0) {
                    perf = data->perf_shadow;
                }
            }
        }
    }

    if (n > 0 && ready) {
        LOG_INF("diag perf=0x%02x n=%u | 4ms=%u 8ms=%u 16ms=%u slow=%u | "
                "min=%uus sched_max=%uus work_max=%uus | drops=%u smart=%u drift=%u",
                perf, n, b[0], b[1], b[2], b[3],
                data->min_delta_us, data->sched_worst_us, data->work_worst_us,
                data->rpt_drops, data->smart_toggles, data->perf_drift);

        if (data->perf_drift) {
            LOG_INF("  last drift @%us: 0x%02x -> 0x%02x | recovered @%us "
                    "after %u retries",
                    data->drift_at_s, data->drift_was, data->drift_found,
                    data->recovery_at_s, data->recovery_retries);
        }

        bool suspicious = (n >= PMW3610_WD_MIN_SAMPLES) && (b[0] == 0);

        if (suspicious) {
            data->stuck_streak++;
        } else {
            data->stuck_streak = 0;
        }

        if (data->stuck_streak >= PMW3610_WD_STREAK) {
            int64_t now = k_uptime_get();
            bool armed = IS_ENABLED(CONFIG_PMW3610_ALT_FRAME_WATCHDOG_ACTION);
            bool cooled = (data->last_reinit_ms == 0) ||
                          (now - data->last_reinit_ms > PMW3610_WD_HOLDOFF_MS);

            LOG_WRN("frame rate stuck: %u windows, min gap %u us, perf=0x%02x%s",
                    data->stuck_streak, data->min_delta_us, perf,
                    armed ? (cooled ? " -> re-init" : " -> holdoff")
                          : " -> detect only");

            data->stuck_streak = 0;
            if (armed && cooled) {
                pmw3610_reinit(data, 0);
            }
        }

        memset(data->delta_buckets, 0, sizeof(data->delta_buckets));
        data->sched_worst_us = 0;
        data->work_worst_us = 0;
        data->min_delta_us = UINT32_MAX;
        data->rpt_drops = 0;
        data->smart_toggles = 0;
    }

    k_work_reschedule_for_queue(&pmw3610_workq, &data->diag_work, PMW3610_DIAG_PERIOD);
}

static int pmw3610_init_irq(const struct device *dev) {
    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    // check readiness of irq gpio pin
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    // init the irq pin
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    // setup and add the irq callback associated
    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    return err;
}

static int pmw3610_init(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("%s is not ready", config->spi.bus->name);
		return -ENODEV;
	}

    // init device pointer
    data->dev = dev;
    data->dx = 0;
    data->dy = 0;
    data->last_smp_time = 0;
    data->last_rpt_time = 0;

    // init smart algorithm flag;
    data->sw_smart_flag = false;

    // dedicated cooperative work queue, shared by all driver instances
    if (!pmw3610_workq_started) {
        k_work_queue_start(&pmw3610_workq, pmw3610_diag_stack,
                           K_THREAD_STACK_SIZEOF(pmw3610_diag_stack),
                           PMW3610_DIAG_PRIO, NULL);
        k_thread_name_set(&pmw3610_workq.thread, "pmw3610");
        pmw3610_workq_started = true;
    }

    // init trigger handler work
    k_work_init(&data->trigger_work, pmw3610_work_callback);

    // periodic frame-rate diagnostics
    k_work_init_delayable(&data->diag_work, pmw3610_diag_dump);
    k_work_reschedule_for_queue(&pmw3610_workq, &data->diag_work, PMW3610_DIAG_PERIOD);
    data->min_delta_us = UINT32_MAX;

    // init irq routine
    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    // Setup delayable and non-blocking init jobs, including following steps:
    // 1. power reset
    // 2. upload initial settings
    // 3. other configs like cpi, downshift time, sample time etc.
    // The sensor is ready to work (i.e., data->ready=true after the above steps are finished)
    k_work_init_delayable(&data->init_work, pmw3610_async_init);

    k_work_schedule_for_queue(&pmw3610_workq, &data->init_work,
                              K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

/* Runs on pmw3610_workq only -- see the wrapper below. */
static int pmw3610_attr_apply(const struct device *dev, enum sensor_attribute attr,
                              const struct sensor_value *val) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    switch ((uint32_t)attr) {
    case PMW3610_ALT_ATTR_CPI:
        /* Remembered, not just written: a re-init re-runs the whole
           configuration, and taking the devicetree value there would silently
           undo a runtime setting the next time the sensor is recovered. */
        data->cpi_wanted = PMW3610_SVALUE_TO_CPI(*val);
        err = pmw3610_set_cpi(dev, data->cpi_wanted,
                              config->swap_xy, config->inv_x, config->inv_y);
        break;

    case PMW3610_ALT_ATTR_RUN_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ALT_ATTR_REST1_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ALT_ATTR_REST2_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ALT_ATTR_REST1_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ALT_ATTR_REST2_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ALT_ATTR_REST3_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }

    return err;
}

/* Every attribute write touches SPI, and every other SPI access in this driver
 * goes through pmw3610_workq. A caller from a keymap behaviour or an input
 * processor would otherwise drive the bus underneath the motion read, and the
 * diagnostics could sample a register mid-change and mistake it for drift --
 * which now escalates to a re-init. So the request is handed to that queue and
 * waited on, which also keeps the synchronous return code the API promises. */
struct pmw3610_attr_req {
    struct k_work work;
    struct k_sem done;
    const struct device *dev;
    enum sensor_attribute attr;
    struct sensor_value val;
    int err;
};

static void pmw3610_attr_work(struct k_work *work) {
    struct pmw3610_attr_req *req = CONTAINER_OF(work, struct pmw3610_attr_req, work);

    req->err = pmw3610_attr_apply(req->dev, req->attr, &req->val);
    k_sem_give(&req->done);
}

static int pmw3610_alt_attr_set(const struct device *dev, enum sensor_channel chan,
                                enum sensor_attribute attr, const struct sensor_value *val) {
    struct pixart_data *data = dev->data;

    if (unlikely(chan != SENSOR_CHAN_ALL)) {
        return -ENOTSUP;
    }

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    if (k_is_in_isr()) {
        /* The wait below is not legal in interrupt context, and no caller has a
           reason to set an attribute from there. */
        return -EWOULDBLOCK;
    }

    /* Already on the right thread: going through the queue from here would
       wait for a work item that cannot run until we return. */
    if (k_current_get() == &pmw3610_workq.thread) {
        return pmw3610_attr_apply(dev, attr, val);
    }

    struct pmw3610_attr_req req = {
        .dev = dev,
        .attr = attr,
        .val = *val,
        .err = 0,
    };

    k_work_init(&req.work, pmw3610_attr_work);
    k_sem_init(&req.done, 0, 1);
    k_work_submit_to_queue(&pmw3610_workq, &req.work);

    /* req lives on this stack, so this wait cannot be given a timeout: returning
       early would leave the queue holding a work item that points at a frame
       which no longer exists. The queue is cooperative and its longest item is
       a verified register write, tens of milliseconds, so there is nothing to
       time out against anyway. */
    k_sem_take(&req.done, K_FOREVER);

    return req.err;
}

static const struct sensor_driver_api pmw3610_driver_api = {
    .attr_set = pmw3610_alt_attr_set,
};

// #if IS_ENABLED(CONFIG_PM_DEVICE)
// static int pmw3610_pm_action(const struct device *dev, enum pm_device_action action) {
//     switch (action) {
//     case PM_DEVICE_ACTION_SUSPEND:
//         return pmw3610_set_interrupt(dev, false);
//     case PM_DEVICE_ACTION_RESUME:
//         return pmw3610_set_interrupt(dev, true);
//     default:
//         return -ENOTSUP;
//     }
// }
// #endif // IS_ENABLED(CONFIG_PM_DEVICE)
// PM_DEVICE_DT_INST_DEFINE(n, pmw3610_pm_action);

#define PMW3610_SPI_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | \
                        SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define PMW3610_DEFINE(n)                                                                          \
    static struct pixart_data data##n;                                                             \
    static const struct pixart_config config##n = {                                                \
		.spi = SPI_DT_SPEC_INST_GET(n, PMW3610_SPI_MODE, 0),		                               \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                                       \
        .swap_xy = DT_PROP(DT_DRV_INST(n), swap_xy),                                               \
        .inv_x = DT_PROP(DT_DRV_INST(n), invert_x),                                                \
        .inv_y = DT_PROP(DT_DRV_INST(n), invert_y),                                                \
        .evt_type = DT_PROP(DT_DRV_INST(n), evt_type),                                             \
        .x_input_code = DT_PROP(DT_DRV_INST(n), x_input_code),                                     \
        .y_input_code = DT_PROP(DT_DRV_INST(n), y_input_code),                                     \
        .force_awake = DT_PROP(DT_DRV_INST(n), force_awake),                                       \
        .force_awake_4ms_mode = DT_PROP(DT_DRV_INST(n), force_awake_4ms_mode),                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n, POST_KERNEL,                \
                          CONFIG_INPUT_PMW3610_INIT_PRIORITY, &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)


#define GET_PMW3610_DEV(node_id) DEVICE_DT_GET(node_id),

static const struct device *pmw3610_devs[] = {
    DT_FOREACH_STATUS_OKAY(pixart_pmw3610_alt, GET_PMW3610_DEV)
};

/* Runs on the same queue as the motion work, so the SPI bus stays serialised.
 * The ZMK listener below fires on the system workqueue, and touching
 * PERFORMANCE from there would race the motion burst read. */
static void pmw3610_force_awake_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    /* Both the activity state and the active endpoint feed the same decision,
       so the state is read back rather than taken from the event payload.
       That makes this level-triggered: whatever changed while idle, the
       correct value is applied on the next event. */
    for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
        const struct device *dev = pmw3610_devs[i];
        struct pixart_data *data = dev->data;

        /* An endpoint event can land while async init is still walking the
           power-up sequence, or during a watchdog re-init. Both paths apply
           the force themselves once configure completes. */
        if (!data->ready) {
            continue;
        }

        pmw3610_set_performance(dev, pmw3610_want_force_awake(dev));
    }
}

static K_WORK_DEFINE(pmw3610_force_awake_work, pmw3610_force_awake_work_cb);

static int on_power_profile_changed(const zmk_event_t *eh) {
    ARG_UNUSED(eh);

    if (pmw3610_workq_started) {
        k_work_submit_to_queue(&pmw3610_workq, &pmw3610_force_awake_work);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_power_profile_changed);
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);
#if IS_ENABLED(CONFIG_ZMK_USB)
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_usb_conn_state_changed);
#endif
