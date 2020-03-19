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

#include "mbed.h"
#include "mbed-client-libservice/platform/arm_hal_interrupt.h"
#include "sal-stack-nanostack-slip/Slip.h"
#include "ns_trace.h"
#include "cmsis_os2.h"
#include "rtx_os.h"

static osThreadId_t slip_thread_id;
// Statically allocate size for worker thread
static uint64_t slip_thread_stk[512];
static osRtxThread_t slip_thread_tcb;
static osThreadAttr_t slip_thread_attr = {0};

#define SIG_SL_RX       0x0001U

static void slip_if_lock(void);
static void slip_if_unlock(void);

#define TRACE_GROUP  "slip"

static SlipMACDriver *_pslipmacdriver;
static phy_device_driver_s *drv;
#if (MBED_VERSION >= MBED_ENCODE_VERSION(6,0,0))
SlipMACDriver::SlipMACDriver(PinName tx, PinName rx, PinName rts, PinName cts) : UnbufferedSerial(tx, rx)
#else
SlipMACDriver::SlipMACDriver(PinName tx, PinName rx, PinName rts, PinName cts) : RawSerial(tx, rx)
#endif
{
    _pslipmacdriver = this;
#if DEVICE_SERIAL_FC || defined(YOTTA_CFG_THREAD_TEST_APP_SLIP_SERIAL_HW_FLOW_CONTROL)
    if(rts != NC && cts != NC) {
       set_flow_control(RTSCTS, rts, cts);
    }
#endif
    pCurSlipRxBuffer = NULL;
    pCurSlipTxBuffer = NULL;
    slip_rx_state = SLIP_RX_STATE_SYNCSEARCH;
    memset(slip_mac, 0, sizeof(slip_mac));
}

SlipMACDriver::~SlipMACDriver()
{
#if (MBED_VERSION >= MBED_ENCODE_VERSION(6,0,0))
    attach(NULL);
#else
    attach(NULL, RxIrq);
    attach(NULL, TxIrq);
#endif
}

int8_t SlipMACDriver::slip_if_tx(uint8_t *buf, uint16_t len, uint8_t tx_id, data_protocol_e data_flow)
{
    (void) data_flow;
    SlipBuffer *pTxBuf;

    //tr_debug("slip_if_tx(): datalen = %d", len);

    //TODO: error case needs to be handled
    if (len > SLIP_TX_RX_MAX_BUFLEN) {
        return 0;
    }

    core_util_critical_section_enter();

    bool bufValid = _pslipmacdriver->pTxSlipBufferFreeList.pop(pTxBuf);
    core_util_critical_section_exit();

    //TODO: No more free TX buffers??
    if (!bufValid) {
        tr_error("Ran out of TX Buffers.");
        return 0;
    }

    memcpy(pTxBuf->buf, buf, len);
    pTxBuf->length = len;

    core_util_critical_section_enter();
    _pslipmacdriver->pTxSlipBufferToTxFuncList.push(pTxBuf);
    core_util_critical_section_exit();

#if (MBED_VERSION >= MBED_ENCODE_VERSION(6,0,0))
    _pslipmacdriver->attach(callback(_pslipmacdriver, &SlipMACDriver::slipIrq));
#else
    _pslipmacdriver->attach(callback(_pslipmacdriver, &SlipMACDriver::txIrq), TxIrq);
#endif

    // success callback
    if( drv->phy_tx_done_cb ){
        drv->phy_tx_done_cb(_pslipmacdriver->net_slip_id, tx_id, PHY_LINK_TX_SUCCESS, 0, 0);
    }

    return 0;
}

void SlipMACDriver::txIrq(void)
{
    if (!pCurSlipTxBuffer) {
        if (!pTxSlipBufferToTxFuncList.pop(pCurSlipTxBuffer)) {
#if (MBED_VERSION >= MBED_ENCODE_VERSION(6,0,0))
            attach(NULL);
#else
            attach(NULL, TxIrq);
#endif
            return;
        }
        slip_tx_count = 0;
        slip_tx_state = SLIP_TX_STATE_START;
    }

    while (writeable()) {
        if (!tx_one_byte()) {
            // Transmit of current buffer finished
            pTxSlipBufferFreeList.push(pCurSlipTxBuffer);
            pCurSlipTxBuffer = NULL;
            break;
        }
    }
}

// Called when we have a buffer TX in progress, and the serial is writable.
// Transmits exactly one byte. Returns true if there are more bytes to transmit
// in the current buffer, false if current buffer is finished.
bool SlipMACDriver::tx_one_byte(void)
{
    uint8_t byte;
    switch (slip_tx_state) {
        case SLIP_TX_STATE_START:
            byte = SLIP_END;
            slip_tx_state = SLIP_TX_STATE_RUNNING;
            break;
        case SLIP_TX_STATE_RUNNING:
            byte = pCurSlipTxBuffer->buf[slip_tx_count];
            if (byte == SLIP_END || byte == SLIP_ESC) {
                byte = SLIP_ESC;
                slip_tx_state = SLIP_TX_STATE_ESCAPING;
            } else {
                slip_tx_count++;
            }
            break;
        case SLIP_TX_STATE_ESCAPING:
            byte = pCurSlipTxBuffer->buf[slip_tx_count++];
            switch (byte) {
                case SLIP_END:
                    byte = SLIP_ESC_END;
                    break;
                case SLIP_ESC:
                    byte = SLIP_ESC_ESC;
                    break;
            }
            slip_tx_state = SLIP_TX_STATE_RUNNING;
            break;
        case SLIP_TX_STATE_END:
            byte = SLIP_END;
            break;
        default:
            return false;
    }

#if (MBED_VERSION >= MBED_ENCODE_VERSION(6,0,0))
    write(&byte, 1);
#else
    _base_putc(byte);
#endif

    if (slip_tx_state == SLIP_TX_STATE_END) {
        return false;
    }

    if (slip_tx_count == pCurSlipTxBuffer->length) {
        slip_tx_state = SLIP_TX_STATE_END;
    }
    return true;
}

void SlipMACDriver::process_rx_byte(uint8_t character)
{
    if (character == SLIP_END) {
        if (pCurSlipRxBuffer && pCurSlipRxBuffer->length > 0) {

            pRxSlipBufferToRxFuncList.push(pCurSlipRxBuffer);
            pCurSlipRxBuffer = NULL;
            osThreadFlagsSet(slip_thread_id, SIG_SL_RX);
        }
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

    // Reached the point we have a data byte to store. Find buffer now.
    if (!pCurSlipRxBuffer) {
        if (!pRxSlipBufferFreeList.pop(pCurSlipRxBuffer)) {
            slip_rx_state = SLIP_RX_STATE_SYNCSEARCH;
            return;
        }
    }

    if (pCurSlipRxBuffer->length < sizeof(pCurSlipRxBuffer->buf)) {
        pCurSlipRxBuffer->buf[pCurSlipRxBuffer->length++] = character;
    } else {
        // Buffer overrun - abandon frame
        slip_rx_state = SLIP_RX_STATE_SYNCSEARCH;
        pCurSlipRxBuffer->length = 0;
    }
}

void SlipMACDriver::print_serial_error()
{
    tr_error("** Serial error, drop packet...");
}

void SlipMACDriver::rxIrq(void)
{
    bool err = 0;
    uint8_t buf[1];

    if (err && slip_rx_state != SLIP_RX_STATE_SYNCSEARCH) {
        slip_rx_state = SLIP_RX_STATE_SYNCSEARCH;
    }

    while (readable()) {
#if (MBED_VERSION >= MBED_ENCODE_VERSION(6,0,0))
        read(buf, 1);
#else
        buf[0] = _base_getc();
#endif
        if (!err) {
            process_rx_byte(buf[0]);
        }
    }
}

#if (MBED_VERSION >= MBED_ENCODE_VERSION(6,0,0))
void SlipMACDriver::slipIrq(void)
{
    if (readable()) {
        rxIrq();
    }

    if (writable()) {
        txIrq();
    }
}
#endif

int8_t SlipMACDriver::Slip_Init(uint8_t *mac, uint32_t backhaulBaud)
{
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
    memset(&slip_phy_driver, 0, sizeof(phy_device_driver_s));

    slip_phy_driver.PHY_MAC = slip_mac;
    slip_phy_driver.link_type = PHY_LINK_SLIP;
    slip_phy_driver.data_request_layer = IPV6_DATAGRAMS_DATA_FLOW;
    slip_phy_driver.driver_description = (char *)"SLIP";
    slip_phy_driver.tx = slip_if_tx;
    drv = &slip_phy_driver;

    // define and bring up the interface
    net_slip_id = arm_net_phy_register(&slip_phy_driver);

    // init rx state machine
    tr_debug("SLIP driver id: %d\r\n", net_slip_id);

    for (int i = 0; i < SLIP_NR_TX_BUFFERS; i++) {
        pTxSlipBufferFreeList.push(new SlipBuffer);
    }

    for (int i = 0; i < SLIP_NR_RX_BUFFERS; i++) {
        pRxSlipBufferFreeList.push(new SlipBuffer);
    }

    baud(backhaulBaud);

#if (MBED_VERSION >= MBED_ENCODE_VERSION(6,0,0))
    attach(callback(this, &SlipMACDriver::slipIrq));
#else
    attach(callback(this, &SlipMACDriver::rxIrq));
#endif

    _pslipmacdriver->SLIP_IRQ_Thread_Create();

    return net_slip_id;
}

void SlipMACDriver::slip_if_rx(const SlipBuffer *rx_buf) const
{
    if (slip_phy_driver.phy_rx_cb) {
        slip_phy_driver.phy_rx_cb(rx_buf->buf, rx_buf->length, 0x80, 0, net_slip_id);
    }
}

void SlipMACDriver::buffer_handover()
{
    for (;;) {
        SlipBuffer *rx_buf = NULL;

        core_util_critical_section_enter();
        pRxSlipBufferToRxFuncList.pop(rx_buf);
        core_util_critical_section_exit();

        if (!rx_buf) {
            break;
        }

        slip_if_rx(rx_buf);

        core_util_critical_section_enter();
        rx_buf->length = 0;
        pRxSlipBufferFreeList.push(rx_buf);
        core_util_critical_section_exit();
    }
}

static void slip_if_lock(void)
{
    platform_enter_critical();
}

static void slip_if_unlock(void)
{
    platform_exit_critical();
}

static __NO_RETURN void SLIP_IRQ_Thread(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t event = osThreadFlagsWait(SIG_SL_RX, osFlagsWaitAny, osWaitForever);

        slip_if_lock();
        if (event & SIG_SL_RX) {
            _pslipmacdriver->buffer_handover();
        }
        slip_if_unlock();
    }
}

void SlipMACDriver::SLIP_IRQ_Thread_Create(void)
{
    slip_thread_attr.stack_mem  = &slip_thread_stk[0];
    slip_thread_attr.cb_mem  = &slip_thread_tcb;
    slip_thread_attr.stack_size = sizeof(slip_thread_stk);
    slip_thread_attr.cb_size = sizeof(slip_thread_tcb);
    slip_thread_id = osThreadNew(SLIP_IRQ_Thread, NULL, &slip_thread_attr);
}