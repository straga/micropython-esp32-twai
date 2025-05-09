/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Musumeci Salvatore
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
#ifndef MICROPY_INCLUDED_ESP32_CAN_H
#define MICROPY_INCLUDED_ESP32_CAN_H

// #include "modmachine.h"
// #include "freertos/task.h"

//#include "py/obj.h"


#define DEVICE_NAME "CAN"

#define CAN_MODE_SILENT_LOOPBACK (0x10)

//esp32 have only that mode 3 modes
typedef enum {
    MODE_NORMAL = TWAI_MODE_NORMAL,
    MODE_SLEEP = -1,
    MODE_LOOPBACK = -2, // TWAI_MODE_NORMAL | CAN_MODE_SILENT_LOOPBACK,
    MODE_SILENT = TWAI_MODE_NO_ACK,
    MODE_SILENT_LOOPBACK = -3,
    MODE_LISTEN_ONLY = TWAI_MODE_LISTEN_ONLY, // esp32 specific
} can_mode_t;

typedef enum _filter_mode_t {
    FILTER_RAW_SINGLE = 0,
    FILTER_RAW_DUAL,
    FILTER_ADDRESS
} filter_mode_t;

typedef struct _esp32_can_config_t {
    twai_timing_config_t timing;
    twai_filter_config_t filter;
    twai_general_config_t general;
    uint32_t baudrate; // bit/s
    bool initialized;
} esp32_can_config_t;

typedef struct _esp32_can_obj_t {
    mp_obj_base_t base;
    esp32_can_config_t *config;
    mp_obj_t rxcallback;
    TaskHandle_t irq_handler;
    byte rx_state;
    bool extframe : 1;
    bool loopback : 1;
    byte last_tx_success : 1;
    byte bus_recovery_success : 1;
    uint16_t num_error_warning; //FIXME: populate this value somewhere
    uint16_t num_error_passive;
    uint16_t num_bus_off;
} esp32_can_obj_t;

typedef enum _rx_state_t {
    RX_STATE_FIFO_EMPTY = 0,
    RX_STATE_MESSAGE_PENDING,
    RX_STATE_FIFO_FULL,
    RX_STATE_FIFO_OVERFLOW,
} rx_state_t;

typedef enum _state_t {
    NOT_INITIATED = TWAI_STATE_STOPPED - 1,
    STOPPED = TWAI_STATE_STOPPED,
    RUNNING = TWAI_STATE_RUNNING,
    BUS_OFF = TWAI_STATE_BUS_OFF,
    RECOVERING = TWAI_STATE_RECOVERING,
} state_t;

typedef enum _error_state_t {
    ERROR = -1,
    /*
    ERROR_ACTIVE = TWAI_ERROR_ACTIVE,
    ERROR_WARNING = TWAI_ERROR_WARNING,
    ERROR_PASSIVE = TWAI_ERROR_PASSIVE,
    ERROR_BUS_OFF = TWAI_ERROR_BUS_OFF,
    */
} error_state_t;

typedef enum _message_flags_t {
    RTR = 1,
    EXTENDED_ID,
    FD_F,
    BRS,
} message_flags_t;

typedef enum _recv_errors_t {
    CRC = 1,
    FORM,
    OVERRUN,
    ESI,
} recv_errors_t;

typedef enum _send_errors_t {
    ARB = 1,
    NACK,
    ERR,
} send_errors_t;

extern const mp_obj_type_t machine_can_type;


#endif // MICROPY_INCLUDED_ESP32_CAN_H
