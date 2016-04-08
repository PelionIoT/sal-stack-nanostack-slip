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

typedef enum {
    SLIP_RX_STATE_SYNCSEARCH,
    SLIP_RX_STATE_SYNCED,
    SLIP_RX_STATE_ESCAPED
} slip_rx_state_t;

#define SLIP_MTU 1500
#define SLIP_TX_RX_MAX_BUFLEN (1+SLIP_MTU*2+1) // End, every byte escaped, End (!)
#define SLIP_NR_BUFFERS 20

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

struct SlipBuffer {
    SlipBuffer(size_t length = 0): length(length) {}
    uint8_t buf[SLIP_TX_RX_MAX_BUFLEN];
    int length;
};

class SlipMACDriver : public RawSerial {
public:
    SlipMACDriver(PinName tx, PinName rx, PinName rts = NC, PinName cts = NC);
    virtual ~SlipMACDriver();
    int8_t Slip_Init(uint8_t *mac = NULL, uint32_t backhaulBaud = 115200);
    static int8_t slip_if_tx(uint8_t *buf, uint16_t len, uint8_t tx_id, data_protocol_e data_flow);
    static void slip_rx();

private:
    // Interrupt routines for UART rx/tx interrupts
    void rxIrq(void);
    void txIrq(void);
    void process_rx_byte(uint8_t character);
    static void print_serial_error();
    uint8_t slip_rx_buf[SLIP_TX_RX_MAX_BUFLEN];
    uint16_t slip_rx_buflen;
    slip_rx_state_t slip_rx_state;
    uint8_t slip_mac[6];
    SlipBuffer *pCurSlipTxBuffer;
    CircularBuffer<SlipBuffer *, SLIP_NR_BUFFERS> pTxSlipBufferFreeList;
    CircularBuffer<SlipBuffer *, SLIP_NR_BUFFERS> pTxSlipBufferToTxFuncList;
    phy_device_driver_s slip_phy_driver;
    int8_t net_slip_id;
};

#endif /* SLIP_H */
