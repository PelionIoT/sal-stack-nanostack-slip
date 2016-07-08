/*
 * Copyright (c) 2015-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef SLIP_H
#define SLIP_H

#include "nanostack/platform/arm_hal_phy.h"
#include "eventOS_event.h"
#include "net_interface.h"
#ifdef MBED_CONF_RTOS_PRESENT
#include "CircularBuffer.h"
#ifdef MBED_CONF_APP_LED1
#define LED1  MBED_CONF_APP_LED1
#endif
#ifdef MBED_CONF_APP_SERIAL_TX
#define SERIAL_TX  MBED_CONF_APP_SERIAL_TX
#endif
#ifdef MBED_CONF_APP_SERIAL_RX
#define SERIAL_RX  MBED_CONF_APP_SERIAL_RX
#endif
#ifdef MBED_CONF_APP_SERIAL_CTS
#define SERIAL_CTS  MBED_CONF_APP_SERIAL_CTS
#endif
#ifdef MBED_CONF_APP_SERIAL_RTS
#define SERIAL_RTS  MBED_CONF_APP_SERIAL_RTS
#endif
#endif

typedef enum {
    SLIP_RX_STATE_SYNCSEARCH,
    SLIP_RX_STATE_SYNCED,
    SLIP_RX_STATE_ESCAPED
} slip_rx_state_t;

typedef enum {
    SLIP_TX_STATE_START,
    SLIP_TX_STATE_RUNNING,
    SLIP_TX_STATE_ESCAPING,
    SLIP_TX_STATE_END
} slip_tx_state_t;

typedef enum {
    TX_Interrupt,
    RX_Interrupt
} slip_IRQ_type;

#define SLIP_MTU 1500
#define SLIP_TX_RX_MAX_BUFLEN SLIP_MTU
#define SLIP_NR_TX_BUFFERS 8
#ifdef MBED_CONF_RTOS_PRESENT
#define SLIP_NR_RX_BUFFERS 4
#else
#define SLIP_NR_RX_BUFFERS 1
#endif

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD
#define SLIP_PACKET_RX 5

struct SlipBuffer {
    SlipBuffer(size_t length = 0): length(length) {}
    uint8_t buf[SLIP_TX_RX_MAX_BUFLEN];
    size_t length;
};

class SlipMACDriver : public RawSerial {
public:
    SlipMACDriver(PinName tx, PinName rx, PinName rts = NC, PinName cts = NC);
    virtual ~SlipMACDriver();
    int8_t Slip_Init(uint8_t *mac = NULL, uint32_t backhaulBaud = 115200);
    static int8_t slip_if_tx(uint8_t *buf, uint16_t len, uint8_t tx_id, data_protocol_e data_flow);
    static void slip_rx();
    void buffer_handover();

private:
    // Interrupt routines for UART rx/tx interrupts
    void rxIrq(void);
    void txIrq(void);
    void process_rx_byte(uint8_t character);
    bool tx_one_byte();
    void slip_if_rx(const SlipBuffer *rx_buf) const;
    static void print_serial_error();
#ifdef MBED_CONF_RTOS_PRESENT
    void slip_if_interrupt_handler(slip_IRQ_type type);
    void SLIP_IRQ_Thread_Create(void);
#endif
    SlipBuffer *pCurSlipRxBuffer;
    CircularBuffer<SlipBuffer *, SLIP_NR_RX_BUFFERS> pRxSlipBufferFreeList;
    CircularBuffer<SlipBuffer *, SLIP_NR_RX_BUFFERS> pRxSlipBufferToRxFuncList;
    slip_rx_state_t slip_rx_state;
    uint8_t slip_mac[6];
    SlipBuffer *pCurSlipTxBuffer;
    size_t slip_tx_count;
    slip_tx_state_t slip_tx_state;
    CircularBuffer<SlipBuffer *, SLIP_NR_TX_BUFFERS> pTxSlipBufferFreeList;
    CircularBuffer<SlipBuffer *, SLIP_NR_TX_BUFFERS> pTxSlipBufferToTxFuncList;
    phy_device_driver_s slip_phy_driver;
    int8_t net_slip_id;
};

#endif /* SLIP_H */
