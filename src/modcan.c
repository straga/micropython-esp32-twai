/* The MIT License (MIT)
 *
 * Copyright (c) 2019 Musumeci Salvatore
 * Copyright (c) 2021 Ihor Nehrutsa
 * Copyright (c) 2022 Yuriy Makarov
 * Copyright (c) 2023 Viktor Vorobjov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "esp32_compat.h"

#include "esp_random.h"
#include "esp_system.h"
#include "esp_idf_version.h"
#include "esp_task.h"

#include "driver/twai.h"
#include "hal/twai_types.h"
#include "hal/twai_hal.h"
#include "soc/twai_periph.h"
#include "soc/clk_tree_defs.h"
#include "soc/soc_caps.h"

// micropython includes
#include "mphalport.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/objarray.h"
#include "py/binary.h"
#include "py/objint.h"
#include "py/objstr.h"
#include "py/objtype.h"
#include "py/objexcept.h"
#include "py/mphal.h" 
#include "py/mperrno.h"
#include "py/mpprint.h"
#include "modcan.h"


// #define _TO_STR(x) #x
// #define TO_STR(x) _TO_STR(x)

// // Define the macros to check the configuration
// #ifdef configSUPPORT_DYNAMIC_ALLOCATION
//     #pragma message ("configSUPPORT_DYNAMIC_ALLOCATION - Define: " TO_STR(configSUPPORT_DYNAMIC_ALLOCATION))
// #else
//     #pragma message ("configSUPPORT_DYNAMIC_ALLOCATION - Not Define")
// #endif

// #ifdef CONFIG_FREERTOS_UNICORE
//     #pragma message ("CONFIG_FREERTOS_UNICORE - Define: " TO_STR(CONFIG_FREERTOS_UNICORE))
// #else
//     #pragma message ("CONFIG_FREERTOS_UNICORE - Not Define")
// #endif

// #ifdef portNUM_PROCESSORS
//     #pragma message ("portNUM_PROCESSORS - Define: " TO_STR(portNUM_PROCESSORS))
// #else
//     #pragma message ("portNUM_PROCESSORS - Not Define")
// #endif

// Need for micropython second step compilation
extern BaseType_t xTaskCreatePinnedToCore(TaskFunction_t pxTaskCode,
    const char * const pcName,
    const uint32_t usStackDepth,
    void * const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t * const pxCreatedTask,
    const BaseType_t xCoreID);


#ifndef __ASSEMBLER__
#include "soc/dport_access.h"
#endif



#define CAN_MODE_SILENT_LOOPBACK (0x10)

// Default baudrate: 500kb

#define CAN_TASK_PRIORITY           (ESP_TASK_PRIO_MIN + 1)  // Приоритет задачи CAN
#define CAN_TASK_STACK_SIZE         (1024)
//{.clk_src = TWAI_CLK_SRC_DEFAULT, 
//.quanta_resolution_hz = 10000000, 
//.brp = 0, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling 

#define CAN_DEFAULT_PRESCALER       (8)
#define CAN_DEFAULT_SJW             (3)
#define CAN_DEFAULT_BS1             (15)
#define CAN_DEFAULT_BS2             (4)

#define CAN_MAX_DATA_FRAME          (8)


// Default bitrate from esp-idf for each baudrate and xtal frequency and chip: set aiutomatically
// #if CONFIG_XTAL_FREQ == 32   // TWAI_CLK_SRC_XTAL = 32M
// #define TWAI_TIMING_CONFIG_25KBITS()    {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 400000, .brp = 0, .tseg_1 = 11, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_50KBITS()    {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 1000000, .brp = 0, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_100KBITS()   {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 2000000, .brp = 0, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_125KBITS()   {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 4000000, .brp = 0, .tseg_1 = 23, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_250KBITS()   {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 4000000, .brp = 0, .tseg_1 = 11, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_500KBITS()   {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 8000000, .brp = 0, .tseg_1 = 11, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_800KBITS()   {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 16000000, .brp = 0, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_1MBITS()     {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 16000000, .brp = 0, .tseg_1 = 11, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}

// #elif CONFIG_XTAL_FREQ == 40   // TWAI_CLK_SRC_XTAL = 40M
// #define TWAI_TIMING_CONFIG_25KBITS()    {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 625000, .brp = 0, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_50KBITS()    {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 1000000, .brp = 0, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_100KBITS()   {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 2000000, .brp = 0, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_125KBITS()   {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 2500000, .brp = 0, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_250KBITS()   {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 5000000, .brp = 0, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_500KBITS()   {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 10000000, .brp = 0, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_800KBITS()   {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 20000000, .brp = 0, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
// #define TWAI_TIMING_CONFIG_1MBITS()     {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 20000000, .brp = 0, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
// #endif  //CONFIG_XTAL_FREQ

/*
// Internal Functions
mp_obj_t esp32_hw_can_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args);
static mp_obj_t esp32_hw_can_init_helper(esp32_can_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args);
static void esp32_hw_can_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind);
*/
// INTERNAL Deinitialize can
void can_deinit(const esp32_can_obj_t *self) {
    check_esp_err(twai_stop());
    check_esp_err(twai_driver_uninstall());
    if (self->irq_handler != NULL) {
        vTaskDelete(self->irq_handler);
    }
    self->config->initialized = false;
}

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
//{.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 400000, .brp = 0, .tseg_1 = 11, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
//static const twai_timing_config_t t_config = {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 400000, .brp = 0, .tseg_1 = 11, .tseg_2 = 4, .sjw = 3, .triple_sampling = false};
// static const twai_timing_config_t t_config = {
//     twai_timing_config_t config = TWAI_TIMING_CONFIG_25KBITS();
//     config.clk_src = TWAI_CLK_SRC_DEFAULT;
//     return config;
// }();

// singleton CAN device object
esp32_can_config_t can_config = {
    .general = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_2, GPIO_NUM_4, TWAI_MODE_NORMAL),
    .filter = f_config,  // TWAI_FILTER_CONFIG_ACCEPT_ALL(),
    .timing = t_config,  // TWAI_TIMING_CONFIG_25KBITS()
    .initialized = false
};

static esp32_can_obj_t esp32_can_obj = {
    {&machine_can_type},
    .config = &can_config
};

// INTERNAL FUNCTION Return status information
static twai_status_info_t _esp32_hw_can_get_status() {
    twai_status_info_t status;
    check_esp_err(twai_get_status_info(&status));
    return status;
}

static void esp32_hw_can_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (self->config->initialized) {
        qstr mode;
        switch (self->config->general.mode) {
        case TWAI_MODE_LISTEN_ONLY:
            mode = MP_QSTR_LISTEN;
            break;
        case TWAI_MODE_NO_ACK:
            mode = MP_QSTR_NO_ACK;
            break;
        case TWAI_MODE_NORMAL:
            mode = MP_QSTR_NORMAL;
            break;
        default:
            mode = MP_QSTR_UNKNOWN;
            break;
        }
        mp_printf(print, "CAN(tx=%u, rx=%u, baudrate=%lukb, mode=%q, loopback=%u, extframe=%u)",
                  self->config->general.tx_io,
                  self->config->general.rx_io,
                  self->config->baudrate,
                  mode,
                  self->loopback,
                  self->extframe);
    } else {
        mp_printf(print, "Device is not initialized");
    }
}

// INTERNAL FUNCTION FreeRTOS IRQ task
static void esp32_hw_can_irq_task(void *self_in) {
    esp32_can_obj_t *self = (esp32_can_obj_t *)self_in;
    uint32_t alerts;

    
    check_esp_err(twai_reconfigure_alerts(TWAI_ALERT_ALL,
        // TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS |
        // TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_TX_FAILED | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_BUS_RECOVERED,
        // TWAI_ALERT_TX_IDLE | TWAI_ALERT_BELOW_ERR_WARN | TWAI_ALERT_ERR_ACTIVE | TWAI_ALERT_RECOVERY_IN_PROGRESS |
        // TWAI_ALERT_ARB_LOST | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_FIFO_OVERRUN | TWAI_ALERT_TX_RETRIED | TWAI_ALERT_PERIPH_RESET,
        NULL
        ));

    mp_printf(&mp_plat_print, "CAN IRQ Alert: task started\n");

    while (1) {
        check_esp_err(twai_read_alerts(&alerts, portMAX_DELAY));

        if (alerts & TWAI_ALERT_BUS_OFF) {
            ++self->num_bus_off;
        }
        if (alerts & TWAI_ALERT_ERR_PASS) {
            ++self->num_error_passive;
        }
        if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
            ++self->num_error_warning;
        }

        if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_TX_SUCCESS)) {
            self->last_tx_success = (alerts & TWAI_ALERT_TX_SUCCESS) > 0;
        }

        if (alerts & (TWAI_ALERT_BUS_RECOVERED)) {
            self->bus_recovery_success = true;
        }

        if (self->rxcallback != mp_const_none) {
            if (alerts & TWAI_ALERT_RX_DATA) {
                uint32_t msgs_to_rx = _esp32_hw_can_get_status().msgs_to_rx;

                if (msgs_to_rx == 1) {
                    // first message in queue
                    mp_sched_schedule(self->rxcallback, MP_OBJ_NEW_SMALL_INT(0));
                } else if (msgs_to_rx >= self->config->general.rx_queue_len) {
                    // queue is full
                    mp_sched_schedule(self->rxcallback, MP_OBJ_NEW_SMALL_INT(1));
                }
            }
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
                // queue overflow
                mp_sched_schedule(self->rxcallback, MP_OBJ_NEW_SMALL_INT(2));
            }
        }
    }
}

#define TWAI_CHECK(cond, ret_val) ({                                        \
            if (!(cond)) {                                                  \
                return (ret_val);                                           \
            }                                                               \
})

// init(mode, tx=5, rx=4, baudrate=500000, prescaler=8, sjw=3, bs1=15, bs2=4, auto_restart=False, tx_queue=1, rx_queue=1)
static mp_obj_t esp32_hw_can_init_helper(esp32_can_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_mode, ARG_extframe, ARG_prescaler, ARG_sjw, ARG_bs1, ARG_bs2, ARG_auto_restart, ARG_baudrate, 
           ARG_tx_io, ARG_rx_io, ARG_tx_queue, ARG_rx_queue};

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode,         MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = TWAI_MODE_NORMAL} },
        { MP_QSTR_extframe,     MP_ARG_BOOL,                    {.u_bool = false} },
        { MP_QSTR_prescaler,    MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = CAN_DEFAULT_PRESCALER} },
        { MP_QSTR_sjw,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = CAN_DEFAULT_SJW} },
        { MP_QSTR_bs1,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = CAN_DEFAULT_BS1} },
        { MP_QSTR_bs2,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = CAN_DEFAULT_BS2} },
        { MP_QSTR_auto_restart, MP_ARG_BOOL,                    {.u_bool = false} },
        { MP_QSTR_baudrate,     MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 500000} },
        { MP_QSTR_tx,           MP_ARG_INT,                     {.u_int = 4} },
        { MP_QSTR_rx,           MP_ARG_INT,                     {.u_int = 5} },
        { MP_QSTR_tx_queue,     MP_ARG_INT,                     {.u_int = 1} },
        { MP_QSTR_rx_queue,     MP_ARG_INT,                     {.u_int = 1} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Configure device
    self->loopback = ((args[ARG_mode].u_int & CAN_MODE_SILENT_LOOPBACK) > 0);
    
    // If loopback mode is set, use TWAI_MODE_NO_ACK as in the official example
    if (self->loopback) {
        self->config->general.mode = TWAI_MODE_NO_ACK; 
    } else {
        self->config->general.mode = args[ARG_mode].u_int & 0x0F;
    }
    
    self->config->general.tx_io = args[ARG_tx_io].u_int;
    self->config->general.rx_io = args[ARG_rx_io].u_int;
    self->config->general.clkout_io = TWAI_IO_UNUSED;
    self->config->general.bus_off_io = TWAI_IO_UNUSED;
    self->config->general.tx_queue_len = args[ARG_tx_queue].u_int;
    self->config->general.rx_queue_len = args[ARG_rx_queue].u_int;
    self->config->general.alerts_enabled = TWAI_ALERT_ALL;
    
    self->config->general.clkout_divider = 0;

    self->extframe = args[ARG_extframe].u_bool;
    if (args[ARG_auto_restart].u_bool) {
        mp_raise_NotImplementedError(MP_ERROR_TEXT("Auto-restart not supported"));
    }
    self->config->filter = f_config; // TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // clear errors
    self->num_error_warning = 0;
    self->num_error_passive = 0;
    self->num_bus_off = 0;

    // Calculate CAN nominal bit timing from baudrate if provided

    // .clk_src = TWAI_CLK_SRC_DEFAULT, 
    // .quanta_resolution_hz = 20000, 
    // .brp = 0, 
    // .tseg_1 = 15, 
    // .tseg_2 = 4, 
    // .sjw = 3, 
    // .triple_sampling = false
    // {.brp = 4000, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}


    switch ((int)args[ARG_baudrate].u_int) {
        case 0:
            self->config->timing = (twai_timing_config_t) {
                .brp = args[ARG_prescaler].u_int,
                .sjw = args[ARG_sjw].u_int,
                .tseg_1 = args[ARG_bs1].u_int,
                .tseg_2 = args[ARG_bs2].u_int,
                .triple_sampling = false
            };
            break;
        #ifdef TWAI_TIMING_CONFIG_1KBITS
        case 1000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_1KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_5KBITS
        case 5000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_5KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_10KBITS
        case 10000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_10KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_12_5KBITS
        case 12500:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_12_5KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_16KBITS
        case 16000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_16KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_20KBITS
        case 20000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_20KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_25KBITS
        case 25000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_25KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_50KBITS
        case 50000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_50KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_100KBITS
        case 100000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_125KBITS
        case 125000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_250KBITS
        case 250000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_500KBITS
        case 500000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_800KBITS
        case 800000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS();
            break;
        #endif
        #ifdef TWAI_TIMING_CONFIG_1MBITS
        case 1000000:
            self->config->timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
            break;
        #endif
        default:
            self->config->baudrate = 0;
            mp_raise_ValueError("Unable to set bitrate");
            return mp_const_none;
    }

    self->config->baudrate = (int)args[ARG_baudrate].u_int;

    mp_printf(&mp_plat_print, "CAN: TIMING\n");
    mp_printf(&mp_plat_print, "CAN: timing brp=%lu\n", self->config->timing.brp);
    mp_printf(&mp_plat_print, "CAN: timing tseg_1=%u\n", self->config->timing.tseg_1);
    mp_printf(&mp_plat_print, "CAN: timing tseg_2=%u\n", self->config->timing.tseg_2);
    mp_printf(&mp_plat_print, "CAN: timing sjw=%u\n", self->config->timing.sjw);
    mp_printf(&mp_plat_print, "CAN: timing triple_sampling=%u\n", self->config->timing.triple_sampling);

    mp_printf(&mp_plat_print, "CAN: BRP_MIN=%u, BRP_MAX=%u\n", SOC_TWAI_BRP_MIN, SOC_TWAI_BRP_MAX);
    mp_printf(&mp_plat_print, "CAN: baudrate %lukb\n", self->config->baudrate);
    mp_printf(&mp_plat_print, "CAN: Mode %u\n", self->config->general.mode);
    
    mp_printf(&mp_plat_print, "CAN: Loopback flag %u\n", self->loopback);



    check_esp_err(twai_driver_install(&self->config->general, &self->config->timing, &self->config->filter));
    check_esp_err(twai_start());

    if (xTaskCreatePinnedToCore(esp32_hw_can_irq_task, "can_irq_task", CAN_TASK_STACK_SIZE, self, CAN_TASK_PRIORITY, (TaskHandle_t *)&self->irq_handler, 1) != pdPASS) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to create can irq task handler"));
    }
    self->config->initialized = true;
    return mp_const_none;
}

// CAN(bus, ...) No argument to get the object
// If no arguments are provided, the initialized object will be returned
static mp_obj_t esp32_hw_can_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);
    if (mp_obj_is_int(args[0]) != true) {
        mp_raise_TypeError(MP_ERROR_TEXT("bus must be a number"));
    }
    // ESP_LOGI("CAN", "check arguments");

    // work out port
    mp_uint_t can_idx = mp_obj_get_int(args[0]);
    if (can_idx != 0) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("CAN(%d) doesn't exist"), can_idx);
    }
    // ESP_LOGI("CAN", "work out port");

    esp32_can_obj_t *self = &esp32_can_obj;
    if (!self->config->initialized || n_args > 1 || n_kw > 0) {
        if (self->config->initialized) {
            // The caller is requesting a reconfiguration of the hardware
            // this can only be done if the hardware is in init mode
            can_deinit(self);
        }
        self->rxcallback = mp_const_none;
        self->irq_handler = NULL;
        self->rx_state = RX_STATE_FIFO_EMPTY;

        // ESP_LOGI("CAN", "STEP:1");

        if (n_args > 1 || n_kw > 0) {
          // start the peripheral
          mp_map_t kw_args;
          
        //   ESP_LOGI( "CAN", "%s", &kw_args );
          mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        //   ESP_LOGI("CAN", "mp_map_init_fixed_table");
          esp32_hw_can_init_helper(self, n_args - 1, args + 1, &kw_args);
        //   ESP_LOGI("CAN", "esp32_hw_can_init_helper");
      }

    }
    return MP_OBJ_FROM_PTR(self);
}

// init(tx, rx, baudrate, mode=NORMAL, tx_queue=2, rx_queue=5)
static mp_obj_t esp32_hw_can_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (self->config->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Device is already initialized"));
        return mp_const_none;
    }

    return esp32_hw_can_init_helper(self, n_args - 1, args + 1, kw_args);

}
static MP_DEFINE_CONST_FUN_OBJ_KW(esp32_hw_can_init_obj, 4, esp32_hw_can_init);

// deinit()
static mp_obj_t esp32_hw_can_deinit(const mp_obj_t self_in) {
    const esp32_can_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->config->initialized != true) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Device is not initialized"));
        return mp_const_none;
    }
    can_deinit(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_hw_can_deinit_obj, esp32_hw_can_deinit);

// Force a software restart of the controller, to allow transmission after a bus error
static mp_obj_t esp32_hw_can_restart(mp_obj_t self_in) {
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(self_in);
    twai_status_info_t status = _esp32_hw_can_get_status();
    if (!self->config->initialized || status.state != TWAI_STATE_BUS_OFF) {
        mp_raise_ValueError(NULL);
    }

    self->bus_recovery_success = -1;
    check_esp_err(twai_initiate_recovery());
    mp_hal_delay_ms(200); // FIXME: replace it with a smarter solution

    while (self->bus_recovery_success < 0) {
        MICROPY_EVENT_POLL_HOOK
    }

    if (self->bus_recovery_success) {
        check_esp_err(twai_start());
    } else {
        mp_raise_OSError(MP_EIO);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_hw_can_restart_obj, esp32_hw_can_restart);

// Get the state of the controller
static mp_obj_t esp32_hw_can_state(mp_obj_t self_in) {
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t state = TWAI_STATE_STOPPED;
    if (self->config->initialized) {
        state = _esp32_hw_can_get_status().state;
    }
    return mp_obj_new_int(state);
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_hw_can_state_obj, esp32_hw_can_state);

// info() -- Get info about error states and TX/RX buffers
static mp_obj_t esp32_hw_can_info(size_t n_args, const mp_obj_t *args) {

    twai_status_info_t status = _esp32_hw_can_get_status();
    mp_obj_t dict = mp_obj_new_dict(0);
    #define dict_key(key) mp_obj_new_str(#key, strlen(#key))
    #define dict_value(key) MP_OBJ_NEW_SMALL_INT(status.key)
    #define dict_store(key) mp_obj_dict_store(dict, dict_key(key), dict_value(key));
    dict_store(state);
    dict_store(msgs_to_tx);
    dict_store(msgs_to_rx);
    dict_store(tx_error_counter);
    dict_store(rx_error_counter);
    dict_store(tx_failed_count);
    dict_store(rx_missed_count);
    dict_store(arb_lost_count);
    dict_store(bus_error_count);
    return dict;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp32_hw_can_info_obj, 1, 2, esp32_hw_can_info);

// Get Alert info
static mp_obj_t esp32_hw_can_alert(mp_obj_t self_in) {
    uint32_t alerts;
    check_esp_err(twai_read_alerts(&alerts, 0));
    return mp_obj_new_int(alerts);
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_hw_can_alert_obj, esp32_hw_can_alert);

// any() - return `True` if any message waiting, else `False`
static mp_obj_t esp32_hw_can_any(mp_obj_t self_in) {
    twai_status_info_t status = _esp32_hw_can_get_status();
    return mp_obj_new_bool((status.msgs_to_rx) > 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_hw_can_any_obj, esp32_hw_can_any);

// send([data], id, *, timeout=0, rtr=false, extframe=false)
static mp_obj_t esp32_hw_can_send(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_data, ARG_id, ARG_timeout, ARG_rtr, ARG_extframe };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_data,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_id,       MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_timeout,  MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 0} },
        { MP_QSTR_rtr,      MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_extframe, MP_ARG_BOOL,                  {.u_bool = false} },
    };

    // parse args
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // populate message
    twai_message_t tx_msg;

    size_t length;
    mp_obj_t *items;
    mp_obj_get_array(args[ARG_data].u_obj, &length, &items);
    if (length > CAN_MAX_DATA_FRAME) {
        mp_raise_ValueError(MP_ERROR_TEXT("CAN data field too long"));
    }
    tx_msg.data_length_code = length;
    tx_msg.flags = (args[ARG_rtr].u_bool ? TWAI_MSG_FLAG_RTR : TWAI_MSG_FLAG_NONE);

    if (args[ARG_extframe].u_bool) {
        tx_msg.identifier = args[ARG_id].u_int & 0x1FFFFFFF;
        tx_msg.flags |= TWAI_MSG_FLAG_EXTD;
    } else {
        tx_msg.identifier = args[ARG_id].u_int & 0x7FF;
    }
    
    // So that the message is not sent to the bus
    if (self->loopback) {
        tx_msg.flags |= TWAI_MSG_FLAG_SELF;
    }

    for (uint8_t i = 0; i < length; i++) {
        tx_msg.data[i] = mp_obj_get_int(items[i]);
    }

    if (_esp32_hw_can_get_status().state == TWAI_STATE_RUNNING) {
        uint32_t timeout_ms = args[ARG_timeout].u_int;

        if (timeout_ms != 0) {
            self->last_tx_success = -1;
            uint32_t start = mp_hal_ticks_us();
            check_esp_err(twai_transmit(&tx_msg, pdMS_TO_TICKS(timeout_ms)));
            while (self->last_tx_success < 0) {
                if (timeout_ms != portMAX_DELAY) {
                    if (mp_hal_ticks_us() - start >= timeout_ms) {
                        mp_raise_OSError(MP_ETIMEDOUT);
                    }
                }
                MICROPY_EVENT_POLL_HOOK
            }

            if (!self->last_tx_success) {
                mp_raise_OSError(MP_EIO);
            }
        } else {
            check_esp_err(twai_transmit(&tx_msg, portMAX_DELAY));
        }

        return mp_const_none;
    } else {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Device is not ready"));
    }
}
static MP_DEFINE_CONST_FUN_OBJ_KW(esp32_hw_can_send_obj, 3, esp32_hw_can_send);

// recv(list=None, *, timeout=5000)
static mp_obj_t esp32_hw_can_recv(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_list, ARG_timeout };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_list, MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5000} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // receive the data
    twai_message_t rx_msg;
    check_esp_err(twai_receive(&rx_msg, pdMS_TO_TICKS(args[ARG_timeout].u_int)));
    uint32_t rx_dlc = rx_msg.data_length_code;

    // Create the tuple, or get the list, that will hold the return values
    // Also populate the fourth element, either a new bytes or reuse existing memoryview
    mp_obj_t ret_obj = args[ARG_list].u_obj;
    mp_obj_t *items;
    if (ret_obj == mp_const_none) {
        ret_obj = mp_obj_new_tuple(4, NULL);
        items = ((mp_obj_tuple_t *)MP_OBJ_TO_PTR(ret_obj))->items;
        items[3] = mp_obj_new_bytes(rx_msg.data, rx_dlc);
    } else {
        // User should provide a list of length at least 4 to hold the values
        if (!mp_obj_is_type(ret_obj, &mp_type_list)) {
            mp_raise_TypeError(NULL);
        }
        mp_obj_list_t *list = MP_OBJ_TO_PTR(ret_obj);
        if (list->len < 4) {
            mp_raise_ValueError(NULL);
        }
        items = list->items;
        // Fourth element must be a memoryview which we assume points to a
        // byte-like array which is large enough, and then we resize it inplace
        if (!mp_obj_is_type(items[3], &mp_type_memoryview)) {
            mp_raise_TypeError(NULL);
        }
        mp_obj_array_t *mv = MP_OBJ_TO_PTR(items[3]);
        if (!(mv->typecode == (MP_OBJ_ARRAY_TYPECODE_FLAG_RW | BYTEARRAY_TYPECODE) || (mv->typecode | 0x20) == (MP_OBJ_ARRAY_TYPECODE_FLAG_RW | 'b'))) {
            mp_raise_ValueError(NULL);
        }
        mv->len = rx_dlc;
        memcpy(mv->items, rx_msg.data, rx_dlc);
    }
    items[0] = MP_OBJ_NEW_SMALL_INT(rx_msg.identifier);
    items[1] = rx_msg.extd ? mp_const_true : mp_const_false;
    items[2] = rx_msg.rtr ? mp_const_true : mp_const_false;

    // Return the result
    return ret_obj;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(esp32_hw_can_recv_obj, 0, esp32_hw_can_recv);

// Clear filters setting
static mp_obj_t esp32_hw_can_clearfilter(mp_obj_t self_in) {
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(self_in);

    // Defaults from TWAI_FILTER_CONFIG_ACCEPT_ALL
    self->config->filter = f_config; // TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Apply filter
    check_esp_err(twai_stop());
    check_esp_err(twai_driver_uninstall());
    check_esp_err(twai_driver_install(
                          &self->config->general,
                          &self->config->timing,
                          &self->config->filter
                          ));
    check_esp_err(twai_start());
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_hw_can_clearfilter_obj, esp32_hw_can_clearfilter);

// bank: 0 only
// mode: FILTER_RAW_SINGLE, FILTER_RAW_DUAL or FILTER_ADDR_SINGLE or FILTER_ADDR_DUAL
// params: [id, mask]
// rtr: ignored if FILTER_RAW
// Set CAN HW filter
static mp_obj_t esp32_hw_can_setfilter(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_bank, ARG_mode, ARG_params, ARG_rtr, ARG_extframe };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_bank,     MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_mode,     MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_params,   MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_rtr,      MP_ARG_KW_ONLY  | MP_ARG_OBJ, {.u_bool = false} },
        { MP_QSTR_extframe, MP_ARG_BOOL,                  {.u_bool = false} },
    };

    // parse args
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    const int can_idx = args[ARG_bank].u_int;

    if (can_idx != 0) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("Bank (%d) doesn't exist"), can_idx);
    }

    size_t len;
    mp_obj_t *params;
    mp_obj_get_array(args[ARG_params].u_obj, &len, &params);
    const int mode = args[ARG_mode].u_int;

    uint32_t id = mp_obj_get_int(params[0]);
    uint32_t mask = mp_obj_get_int(params[1]); // FIXME: Overflow in case 0xFFFFFFFF for mask
    if (mode == FILTER_RAW_SINGLE || mode == FILTER_RAW_DUAL) {
        if (len != 2) {
            mp_raise_ValueError(MP_ERROR_TEXT("params must be a 2-values list"));
        }
        self->config->filter.single_filter = (mode == FILTER_RAW_SINGLE);
        self->config->filter.acceptance_code = id;
        self->config->filter.acceptance_mask = mask;
    } else {
        self->config->filter.single_filter = self->extframe;
        // esp32_hw_can_setfilter(self, id, mask, args[ARG_bank].u_int, args[ARG_rtr].u_int);
        //Check if bank is allowed
        int bank = 0;
        if (bank > ((self->extframe && self->config->filter.single_filter) ? 0 : 1 )) {
            mp_raise_ValueError(MP_ERROR_TEXT("CAN filter parameter error"));
        }
        uint32_t preserve_mask;
        int addr = 0;
        int rtr = 0;
        if (self->extframe) {
            addr = (addr & 0x1FFFFFFF) << 3 | (rtr ? 0x04 : 0);
            mask = (mask & 0x1FFFFFFF) << 3 | 0x03;
            preserve_mask = 0;
        } else if (self->config->filter.single_filter) {
            addr = ( ( (addr & 0x7FF) << 5 ) | (rtr ? 0x10 : 0) );
            mask = ( (mask & 0x7FF) << 5 );
            mask |= 0xFFFFF000;
            preserve_mask = 0;
        } else {
            addr = ( ( (addr & 0x7FF) << 5 ) | (rtr ? 0x10 : 0) );
            mask = ( (mask & 0x7FF) << 5 );
            preserve_mask = 0xFFFF << ( bank == 0 ? 16 : 0 );
            if ( (self->config->filter.acceptance_mask & preserve_mask) == ( 0xFFFF << (bank == 0 ? 16 : 0) ) ) {
                // Other filter accepts all; it will replaced duplicating current filter
                addr = addr | (addr << 16);
                mask = mask | (mask << 16);
                preserve_mask = 0;
            } else {
                addr = addr << (bank == 1 ? 16 : 0);
                mask = mask << (bank == 1 ? 16 : 0);
            }
        }
        self->config->filter.acceptance_code &= preserve_mask;
        self->config->filter.acceptance_code |= addr;
        self->config->filter.acceptance_mask &= preserve_mask;
        self->config->filter.acceptance_mask |= mask;
    }
    // Apply filter
    check_esp_err(twai_stop());
    check_esp_err(twai_driver_uninstall());
    check_esp_err(twai_driver_install(
                          &self->config->general,
                          &self->config->timing,
                          &self->config->filter
                          ));
    check_esp_err(twai_start());

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(esp32_hw_can_setfilter_obj, 1, esp32_hw_can_setfilter);

// rxcallback(callable)
static mp_obj_t esp32_hw_can_rxcallback(mp_obj_t self_in, mp_obj_t callback_in) {
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (callback_in == mp_const_none) {
        // disable callback
        self->rxcallback = mp_const_none;
    } else if (mp_obj_is_callable(callback_in)) {
        // set up interrupt
        self->rxcallback = callback_in;
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(esp32_hw_can_rxcallback_obj, esp32_hw_can_rxcallback);

// Clear TX Queue
static mp_obj_t esp32_hw_can_clear_tx_queue(mp_obj_t self_in) {
    return mp_obj_new_bool(twai_clear_transmit_queue() == ESP_OK);
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_hw_can_clear_tx_queue_obj, esp32_hw_can_clear_tx_queue);

// Clear RX Queue
static mp_obj_t esp32_hw_can_clear_rx_queue(mp_obj_t self_in) {
    return mp_obj_new_bool(twai_clear_receive_queue() == ESP_OK);
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_hw_can_clear_rx_queue_obj, esp32_hw_can_clear_rx_queue);

// Get state of the controller
static mp_obj_t esp32_hw_can_get_state(size_t n_args, const mp_obj_t *args) {
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (!self->config->initialized) {
        mp_raise_ValueError(NULL);
    }
    twai_status_info_t status;
    check_esp_err(twai_get_status_info(&status));
    return mp_obj_new_int(status.state);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp32_hw_can_get_state_obj, 1, 1, esp32_hw_can_get_state);

// Get counters
static mp_obj_t esp32_hw_can_get_counters(size_t n_args, const mp_obj_t *args) {
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (!self->config->initialized) {
        mp_raise_ValueError(NULL);
    }
    
    mp_obj_list_t *list;
    if (n_args == 1) {
        list = MP_OBJ_TO_PTR(mp_obj_new_list(8, NULL));
    } else {
        if (!mp_obj_is_type(args[1], &mp_type_list)) {
            mp_raise_TypeError(NULL);
        }
        list = MP_OBJ_TO_PTR(args[1]);
        if (list->len < 8) {
            mp_raise_ValueError(NULL);
        }
    }
    
    twai_status_info_t status;
    check_esp_err(twai_get_status_info(&status));
        
    list->items[0] = MP_OBJ_NEW_SMALL_INT(status.tx_error_counter);
    list->items[1] = MP_OBJ_NEW_SMALL_INT(status.rx_error_counter);
    list->items[2] = MP_OBJ_NEW_SMALL_INT(self->num_error_warning);
    list->items[3] = MP_OBJ_NEW_SMALL_INT(self->num_error_passive);
    list->items[4] = MP_OBJ_NEW_SMALL_INT(self->num_bus_off);
    list->items[5] = MP_OBJ_NEW_SMALL_INT(status.msgs_to_tx);
    list->items[6] = MP_OBJ_NEW_SMALL_INT(status.msgs_to_rx);
    list->items[7] = mp_const_none;

    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp32_hw_can_get_counters_obj, 1, 2, esp32_hw_can_get_counters);

// Get timings
static mp_obj_t esp32_hw_can_get_timings(size_t n_args, const mp_obj_t *args) {
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (!self->config->initialized) {
        mp_raise_ValueError(NULL);
    }
    
    mp_obj_list_t *list;
    if (n_args == 1) {
        list = MP_OBJ_TO_PTR(mp_obj_new_list(5, NULL));
    } else {
        if (!mp_obj_is_type(args[1], &mp_type_list)) {
            mp_raise_TypeError(NULL);
        }
        list = MP_OBJ_TO_PTR(args[1]);
        if (list->len < 5) {
            mp_raise_ValueError(NULL);
        }
    }
    
    // Заполняем список параметрами таймингов
    list->items[0] = MP_OBJ_NEW_SMALL_INT(self->config->timing.brp);
    list->items[1] = MP_OBJ_NEW_SMALL_INT(self->config->timing.tseg_1);
    list->items[2] = MP_OBJ_NEW_SMALL_INT(self->config->timing.tseg_2);
    list->items[3] = MP_OBJ_NEW_SMALL_INT(self->config->timing.sjw);
    list->items[4] = mp_obj_new_bool(self->config->timing.triple_sampling);
    
    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp32_hw_can_get_timings_obj, 1, 2, esp32_hw_can_get_timings);

// Reset CAN controller
static mp_obj_t esp32_hw_can_reset(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_mode };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode, MP_ARG_INT, {.u_int = TWAI_MODE_NORMAL} },
    };
    
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    if (!self->config->initialized) {
        mp_raise_ValueError(NULL);
    }
    
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
       
    // Stop the driver and uninstall it
    check_esp_err(twai_stop());
    check_esp_err(twai_driver_uninstall());
    
    // Update mode
    self->config->general.mode = args[ARG_mode].u_int;
    
    // Initialize the driver with the new mode
    check_esp_err(twai_driver_install(&self->config->general, &self->config->timing, &self->config->filter));
    check_esp_err(twai_start());
    
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(esp32_hw_can_reset_obj, 1, esp32_hw_can_reset);

// Get/Set CAN controller mode
static mp_obj_t esp32_hw_can_mode(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_mode };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode, MP_ARG_INT | MP_ARG_REQUIRED, {.u_int = TWAI_MODE_NORMAL} },
    };
    
    esp32_can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    if (!self->config->initialized) {
        mp_raise_ValueError(NULL);
    }
    
    // If no arguments are provided, return the current mode
    if (n_args == 1 && kw_args->used == 0) {
        return mp_obj_new_int(self->config->general.mode);
    }
    
    // Extract the mode argument
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
       
    // before changing the mode , stop and uninstall the driver
    check_esp_err(twai_stop());
    check_esp_err(twai_driver_uninstall());
    
    // Update the mode
    self->config->general.mode = args[ARG_mode].u_int;
    
    // initialize the driver with the new mode
    check_esp_err(twai_driver_install(&self->config->general, &self->config->timing, &self->config->filter));
    check_esp_err(twai_start());
    
    return mp_obj_new_int(self->config->general.mode);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(esp32_hw_can_mode_obj, 1, esp32_hw_can_mode);

static const mp_rom_map_elem_t esp32_can_locals_dict_table[] = {
    // CAN_ATTRIBUTES
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_CAN) },
    // Micropython Generic API
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&esp32_hw_can_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&esp32_hw_can_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_restart), MP_ROM_PTR(&esp32_hw_can_restart_obj) },
    { MP_ROM_QSTR(MP_QSTR_state), MP_ROM_PTR(&esp32_hw_can_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&esp32_hw_can_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_any), MP_ROM_PTR(&esp32_hw_can_any_obj) },
    { MP_ROM_QSTR(MP_QSTR_send), MP_ROM_PTR(&esp32_hw_can_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv), MP_ROM_PTR(&esp32_hw_can_recv_obj) },
    { MP_ROM_QSTR(MP_QSTR_setfilter), MP_ROM_PTR(&esp32_hw_can_setfilter_obj) },
    { MP_ROM_QSTR(MP_QSTR_clearfilter), MP_ROM_PTR(&esp32_hw_can_clearfilter_obj) },
    { MP_ROM_QSTR(MP_QSTR_rxcallback), MP_ROM_PTR(&esp32_hw_can_rxcallback_obj) },
    // ESP32 Specific API
    { MP_OBJ_NEW_QSTR(MP_QSTR_clear_tx_queue), MP_ROM_PTR(&esp32_hw_can_clear_tx_queue_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_clear_rx_queue), MP_ROM_PTR(&esp32_hw_can_clear_rx_queue_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_alerts), MP_ROM_PTR(&esp32_hw_can_alert_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_state), MP_ROM_PTR(&esp32_hw_can_get_state_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_counters), MP_ROM_PTR(&esp32_hw_can_get_counters_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_timings), MP_ROM_PTR(&esp32_hw_can_get_timings_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_reset), MP_ROM_PTR(&esp32_hw_can_reset_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_mode), MP_ROM_PTR(&esp32_hw_can_mode_obj) },
    // CAN_MODE
    { MP_ROM_QSTR(MP_QSTR_NORMAL), MP_ROM_INT(TWAI_MODE_NORMAL) },                                     //ESP32 exist TWAI_MODE_NORMAL
    { MP_ROM_QSTR(MP_QSTR_LOOPBACK), MP_ROM_INT(TWAI_MODE_NORMAL | CAN_MODE_SILENT_LOOPBACK) },
    { MP_ROM_QSTR(MP_QSTR_SILENT), MP_ROM_INT(TWAI_MODE_NO_ACK) },                                     //ESP32 exist TWAI_MODE_NO_ACK
    { MP_ROM_QSTR(MP_QSTR_SILENT_LOOPBACK), MP_ROM_INT(TWAI_MODE_NO_ACK | CAN_MODE_SILENT_LOOPBACK) }, 
    { MP_ROM_QSTR(MP_QSTR_LISTEN_ONLY), MP_ROM_INT(TWAI_MODE_LISTEN_ONLY) },                           //ESP32 exist TWAI_MODE_LISTEN_ONLY


    // MODE_NORMAL = TWAI_MODE_NORMAL,
    // MODE_SLEEP = -1,
    // MODE_LOOPBACK = -2, // TWAI_MODE_NORMAL | CAN_MODE_SILENT_LOOPBACK,
    // MODE_SILENT = TWAI_MODE_NO_ACK,
    // MODE_SILENT_LOOPBACK = -3,
    // MODE_LISTEN_ONLY = TWAI_MODE_LISTEN_ONLY, // esp32 specific

    // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html#_CPPv411twai_mode_t

    /* esp32 can modes
    TWAI_MODE_NORMAL      - Normal operating mode where TWAI controller can send/receive/acknowledge messages
    TWAI_MODE_NO_ACK      - Transmission does not require acknowledgment. Use this mode for self testing. // This mode is useful when self testing the TWAI controller (loopback of transmissions).
    TWAI_MODE_LISTEN_ONLY - The TWAI controller will not influence the bus (No transmissions or acknowledgments) but can receive messages. // This mode is suited for bus monitor applications.
    */
    /* stm32 can modes
    #define CAN_MODE_NORMAL             FDCAN_MODE_NORMAL
    #define CAN_MODE_LOOPBACK           FDCAN_MODE_EXTERNAL_LOOPBACK
    #define CAN_MODE_SILENT             FDCAN_MODE_BUS_MONITORING
    #define CAN_MODE_SILENT_LOOPBACK    FDCAN_MODE_INTERNAL_LOOPBACK
    */
   s
    // CAN_STATE
    // class CAN.State
    { MP_ROM_QSTR(MP_QSTR_STOPPED), MP_ROM_INT(TWAI_STATE_STOPPED) },
    { MP_ROM_QSTR(MP_QSTR_ERROR_ACTIVE), MP_ROM_INT(TWAI_STATE_RUNNING) },
    { MP_ROM_QSTR(MP_QSTR_ERROR_WARNING), MP_ROM_INT(-1) },
    { MP_ROM_QSTR(MP_QSTR_ERROR_PASSIVE), MP_ROM_INT(-1) },
    { MP_ROM_QSTR(MP_QSTR_BUS_OFF), MP_ROM_INT(TWAI_STATE_BUS_OFF) },
    { MP_ROM_QSTR(MP_QSTR_RECOVERING), MP_ROM_INT(TWAI_STATE_RECOVERING) }, // esp32 specific

    // class CAN.MessageFlags
    { MP_ROM_QSTR(MP_QSTR_RTR), MP_ROM_INT(RTR) },
    { MP_ROM_QSTR(MP_QSTR_EXTENDED_ID), MP_ROM_INT(EXTENDED_ID) },
    { MP_ROM_QSTR(MP_QSTR_FD_F), MP_ROM_INT(FD_F) },
    { MP_ROM_QSTR(MP_QSTR_BRS), MP_ROM_INT(BRS) },
    // class CAN.RecvErrors
    { MP_ROM_QSTR(MP_QSTR_CRC), MP_ROM_INT(CRC) },
    { MP_ROM_QSTR(MP_QSTR_FORM), MP_ROM_INT(FORM) },
    { MP_ROM_QSTR(MP_QSTR_OVERRUN), MP_ROM_INT(OVERRUN) },
    { MP_ROM_QSTR(MP_QSTR_ESI), MP_ROM_INT(ESI) },
    // class CAN.SendErrors
    { MP_ROM_QSTR(MP_QSTR_ARB), MP_ROM_INT(ARB) },
    { MP_ROM_QSTR(MP_QSTR_NACK), MP_ROM_INT(NACK) },
    { MP_ROM_QSTR(MP_QSTR_ERR), MP_ROM_INT(ERR) },
    // CAN_FILTER_MODE
    { MP_ROM_QSTR(MP_QSTR_FILTER_RAW_SINGLE), MP_ROM_INT(FILTER_RAW_SINGLE) },
    { MP_ROM_QSTR(MP_QSTR_FILTER_RAW_DUAL), MP_ROM_INT(FILTER_RAW_DUAL) },
    { MP_ROM_QSTR(MP_QSTR_FILTER_ADDRESS), MP_ROM_INT(FILTER_ADDRESS) },
    // CAN_ALERT
    { MP_ROM_QSTR(MP_QSTR_ALERT_ALL), MP_ROM_INT(TWAI_ALERT_ALL) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_TX_IDLE), MP_ROM_INT(TWAI_ALERT_TX_IDLE) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_TX_SUCCESS), MP_ROM_INT(TWAI_ALERT_TX_SUCCESS) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_BELOW_ERR_WARN), MP_ROM_INT(TWAI_ALERT_BELOW_ERR_WARN) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_ERR_ACTIVE), MP_ROM_INT(TWAI_ALERT_ERR_ACTIVE) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_RECOVERY_IN_PROGRESS), MP_ROM_INT(TWAI_ALERT_RECOVERY_IN_PROGRESS) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_BUS_RECOVERED), MP_ROM_INT(TWAI_ALERT_BUS_RECOVERED) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_ARB_LOST), MP_ROM_INT(TWAI_ALERT_ARB_LOST) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_ABOVE_ERR_WARN), MP_ROM_INT(TWAI_ALERT_ABOVE_ERR_WARN) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_BUS_ERROR), MP_ROM_INT(TWAI_ALERT_BUS_ERROR) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_TX_FAILED), MP_ROM_INT(TWAI_ALERT_TX_FAILED) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_RX_QUEUE_FULL), MP_ROM_INT(TWAI_ALERT_RX_QUEUE_FULL) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_ERR_PASS), MP_ROM_INT(TWAI_ALERT_ERR_PASS) },
    { MP_ROM_QSTR(MP_QSTR_ALERT_BUS_OFF), MP_ROM_INT(TWAI_ALERT_BUS_OFF) }
};
static MP_DEFINE_CONST_DICT(esp32_can_locals_dict, esp32_can_locals_dict_table);

// Python object definition
MP_DEFINE_CONST_OBJ_TYPE(
    machine_can_type,
    MP_QSTR_CAN,
    MP_TYPE_FLAG_NONE,
    make_new, esp32_hw_can_make_new,
    print, esp32_hw_can_print,
    locals_dict, (mp_obj_dict_t *)&esp32_can_locals_dict
    );

MP_REGISTER_MODULE(MP_QSTR_CAN, machine_can_type);

