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

#include "mbed-drivers/mbed.h"
#include "mbed-client-libservice/platform/arm_hal_interrupt.h"
#include "core-util/CriticalSectionLock.h"
#include "sal-stack-nanostack-slip/Slip.h"
//#define HAVE_DEBUG 1
#include "ns_trace.h"
#define TRACE_GROUP  "slip"

static SlipMACDriver *_pslipmacdriver;

SlipMACDriver::SlipMACDriver(PinName tx, PinName rx, PinName rts, PinName cts) : RawSerial(tx, rx)
{
    _pslipmacdriver = this;
    if(rts != NC && cts != NC)
    	set_flow_control(RTSCTS, rts, cts);
    slip_rx_buflen = 0;
    slip_rx_state = SLIP_RX_STATE_SYNCSEARCH;
    memset(slip_mac, 0, sizeof(slip_mac));
}

SlipMACDriver::~SlipMACDriver()
{
    attach(NULL, RxIrq);
    attach(NULL, TxIrq);
}

int8_t SlipMACDriver::slip_if_tx(uint8_t *buf, uint16_t len, uint8_t tx_id, data_protocol_e data_flow)
{
    (void) data_flow;
    SlipBuffer *pTxBuf = NULL;
    uint16_t txBuflen = 0;

    //Remove TUN Header
    if (len > 4) {
        buf += 4;
        len -= 4;
    }

    tr_debug("slip_if_tx(): datalen = %d", len);

    //TODO: error case needs to be handled
    if (len > SLIP_TX_RX_MAX_BUFLEN) {
        return 0;
    }

    {
        mbed::util::CriticalSectionLock lock;
        bool bufValid = _pslipmacdriver->pTxSlipBufferFreeList.pop(pTxBuf);
        //TODO: No more free TX buffers??
        if (!bufValid) {
            return 0;
        }
    }

    pTxBuf->buf[txBuflen++] = SLIP_END;
    while (len--) {
        if (*buf == SLIP_END) {
            pTxBuf->buf[txBuflen++] = SLIP_ESC;
            pTxBuf->buf[txBuflen++] = SLIP_ESC_END;
            buf++;
        } else if (*buf == SLIP_ESC) {
            pTxBuf->buf[txBuflen++] = SLIP_ESC;
            pTxBuf->buf[txBuflen++] = SLIP_ESC_ESC;
            buf++;
        } else {
            pTxBuf->buf[txBuflen++] = *buf++;
        }
    }

    pTxBuf->buf[txBuflen++] = SLIP_END;
    pTxBuf->length = txBuflen;

    {
        mbed::util::CriticalSectionLock lock;
        _pslipmacdriver->pTxSlipBufferToTxFuncList.push(pTxBuf);
    }

    _pslipmacdriver->attach(_pslipmacdriver, &SlipMACDriver::txIrq, TxIrq);

    // success callback
    arm_net_phy_tx_done(_pslipmacdriver->net_slip_id, tx_id, PHY_LINK_TX_SUCCESS, 0, 0);

    return 0;
}

void SlipMACDriver::txIrq(void)
{
    static int i = 0;
    static SlipBuffer *pTxSlipBuffer = NULL;

    if (!pTxSlipBuffer) {
        if (!pTxSlipBufferToTxFuncList.pop(pTxSlipBuffer)) {
            attach(NULL, TxIrq);
            return;
        }
    }

    if (pTxSlipBuffer->buf) {
        while (writeable()) {
            _base_putc(pTxSlipBuffer->buf[i++]);
            if (i == pTxSlipBuffer->length) {
                i = 0;
                pTxSlipBufferFreeList.push(pTxSlipBuffer);
                pTxSlipBuffer = NULL;
                break;
            }
        }
    }
}

void SlipMACDriver::process_rx_byte(uint8_t character)
{
    if (character == SLIP_END) {
        if (slip_rx_buflen > 0) {
            platform_interrupts_disabled();
            arm_net_phy_rx(IPV6_DATAGRAM, slip_rx_buf, slip_rx_buflen, 0x80, 0, net_slip_id);
            platform_interrupts_enabling();
        }
        slip_rx_buflen = 0;
        slip_rx_state = SLIP_RX_STATE_SYNCED;
        return;
    }

    if (slip_rx_state == SLIP_RX_STATE_SYNCSEARCH) {
        return;
    }

    if (character == SLIP_ESC) {
        slip_rx_state = SLIP_RX_STATE_ESCAPED;
        return;
    }

    if (slip_rx_state == SLIP_RX_STATE_ESCAPED) {
        switch (character) {
            case SLIP_ESC_END:
                character = SLIP_END;
                break;
            case SLIP_ESC_ESC:
                character = SLIP_ESC;
                break;
            default:
                break;
        }
        slip_rx_state = SLIP_RX_STATE_SYNCED;
    }

    if (slip_rx_buflen < sizeof slip_rx_buf) {
        slip_rx_buf[slip_rx_buflen++] = character;
    } else {
        // Buffer overrun - abandon frame
        slip_rx_state = SLIP_RX_STATE_SYNCSEARCH;
        slip_rx_buflen = 0;
    }
}

void SlipMACDriver::print_serial_error()
{
    tr_error("** Serial error, drop packet...");
}

void SlipMACDriver::rxIrq(void)
{
    //bool err = error();
    bool err = 0;

    if (err && slip_rx_state != SLIP_RX_STATE_SYNCSEARCH) {
        slip_rx_state = SLIP_RX_STATE_SYNCSEARCH;
        minar::Scheduler::postCallback(mbed::util::FunctionPointer0<void>(print_serial_error).bind())
        .tolerance(minar::milliseconds(1));
    }

    while (readable()) {
        uint8_t character = _base_getc();
        if (!err) {
            process_rx_byte(character);
        }
    }
}

int8_t SlipMACDriver::Slip_Init(uint8_t *mac, uint32_t backhaulBaud)
{
    SlipBuffer *pTmpSlipBuffer;

    if (mac != NULL) {
        // Assign user submitted MAC value
        for (uint8_t i = 0; i < sizeof(slip_mac); ++i) {
            slip_mac[i] = mac[i];
        }
    } else {
        // Generate pseudo value for MAC
        for (uint8_t i = 0; i < sizeof(slip_mac); ++i) {
            slip_mac[i] = i + 2;
        }
    }

    //Build driver data structure
    slip_phy_driver.PHY_MAC = slip_mac;
    slip_phy_driver.link_type = PHY_LINK_TUN;
    slip_phy_driver.data_request_layer = IPV6_DATAGRAMS_DATA_FLOW;
    slip_phy_driver.driver_description = (char *)"SLIP";
    slip_phy_driver.phy_MTU = 0;
    slip_phy_driver.phy_tail_length = 0;
    slip_phy_driver.phy_header_length = 0;
    slip_phy_driver.state_control = 0;
    slip_phy_driver.tx = slip_if_tx;
    slip_phy_driver.address_write = 0;
    slip_phy_driver.extension = 0;

    // define and bring up the interface
    net_slip_id = arm_net_phy_register(&slip_phy_driver);

    // init rx state machine
    tr_debug("SLIP driver id: %d\r\n", net_slip_id);

    for (int i = 0; i < SLIP_NR_BUFFERS; i++) {
        pTmpSlipBuffer = new SlipBuffer;

        memset(pTmpSlipBuffer->buf, 0, sizeof pTmpSlipBuffer->buf);
        pTmpSlipBuffer->length = 0;

        pTxSlipBufferFreeList.push(pTmpSlipBuffer);
    }

    baud(backhaulBaud);

    attach(this, &SlipMACDriver::rxIrq, RxIrq);
    return net_slip_id;
}
