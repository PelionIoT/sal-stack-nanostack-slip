/*
 * Copyright (c) 2014-2015 ARM Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed-drivers/mbed.h"
#include "core-util/CriticalSectionLock.h"
#include "sal-stack-nanostack-slip/Slip.h"
//#define HAVE_DEBUG 1
#include "ns_trace.h"
#define TRACE_GROUP  "slip"

static SlipMACDriver * _pslipmacdriver;

SlipMACDriver::SlipMACDriver(PinName tx, PinName rx) : RawSerial(tx,rx)
{
    _pslipmacdriver = this;
    slip_rx_buflen = 0;
    slip_rx_state = SLIP_RX_STATE_SYNCSEARCH;

   	for(uint8_t i =0; i < sizeof(slip_mac); i++)
        slip_mac[i] = i+1;
}

SlipMACDriver::~SlipMACDriver()
{
    RawSerial::attach(NULL, Serial::RxIrq);
    RawSerial::attach(NULL, Serial::TxIrq);
}

int8_t slip_if_tx(uint8_t *buf, uint16_t len, uint8_t tx_id, data_protocol_e data_flow)
{
    (void) data_flow;
    SlipBuffer *pTxBuf = NULL;
    uint16_t txBuflen = 0;

    //Remove TUN Header
    if(len > 4)
    {
        buf+=4;
        len-=4;
    }

    //TODO: error case needs to be handled
    if(len > SLIP_TX_RX_MAX_BUFLEN)
        return 0;

    tr_debug("slip_if_tx: datalen = %d",len);

    {
        mbed::util::CriticalSectionLock lock;
        bool bufValid = _pslipmacdriver->pTxSlipBufferFreeList.pop(pTxBuf);
        //TODO: No more free TX buffers??
        if(!bufValid)
            return 0;
    }

    pTxBuf->buf[txBuflen++] = SLIP_END;
    while(len--)
    {
        if (*buf == SLIP_END)
        {
            pTxBuf->buf[txBuflen++] = SLIP_ESC;
            pTxBuf->buf[txBuflen++] = SLIP_ESC_END;
            buf++;
        }
        else
        if (*buf == SLIP_ESC)
        {
            pTxBuf->buf[txBuflen++] = SLIP_ESC;
            pTxBuf->buf[txBuflen++] = SLIP_ESC_ESC;
            buf++;
        }
        else
        {
            pTxBuf->buf[txBuflen++] = *buf++;
        }
    }

    pTxBuf->buf[txBuflen++] = SLIP_END;
    pTxBuf->length = txBuflen;

    {
        mbed::util::CriticalSectionLock lock;
        _pslipmacdriver->pTxSlipBufferToTxFuncList.push(pTxBuf);
    }

    _pslipmacdriver->RawSerial::attach(_pslipmacdriver, &SlipMACDriver::txIrq, Serial::TxIrq);

    // success callback
    arm_net_phy_tx_done(_pslipmacdriver->net_slip_id, tx_id, PHY_LINK_TX_SUCCESS, 0, 0);

    return 0;
}

void slip_rx()
{
    SlipBuffer* pRxSlipBuffer = NULL;

    while(1)
    {
        {
            mbed::util::CriticalSectionLock lock;
            bool bufValid = _pslipmacdriver->pRxSlipBufferToRxFuncList.pop(pRxSlipBuffer);
            if(!bufValid)
                break;
        }

        if(pRxSlipBuffer->length > SLIP_TX_RX_MAX_BUFLEN)
        {
            _pslipmacdriver->slip_rx_state = SLIP_RX_STATE_SYNCSEARCH;
            _pslipmacdriver->slip_rx_buflen = 0;

            {
                mbed::util::CriticalSectionLock lock;
                _pslipmacdriver->pRxSlipBufferFreeList.push(pRxSlipBuffer);
            }

            continue;
        }

        for(int i = 0; i <= pRxSlipBuffer->length; i++)
        {
            switch(_pslipmacdriver->slip_rx_state)
            {
                case SLIP_RX_STATE_SYNCSEARCH:
                    if (pRxSlipBuffer->buf[i] == SLIP_END)  // initial sync marker
                    {
                        _pslipmacdriver->slip_rx_state = SLIP_RX_STATE_SYNCED;
                        _pslipmacdriver->slip_rx_buflen = 0;
                    }
                break;

                case SLIP_RX_STATE_SYNCED:
                    switch(pRxSlipBuffer->buf[i])
                    {
                        case SLIP_ESC:  // escaped byte coming
                            _pslipmacdriver->slip_rx_state = SLIP_RX_STATE_ESCAPED;
                        break;
                        case SLIP_END:  // end sync marker
                            arm_net_phy_rx(IPV6_DATAGRAM, _pslipmacdriver->slip_rx_buf, _pslipmacdriver->slip_rx_buflen, 0x80, 0, _pslipmacdriver->net_slip_id);
                            //tr_debug("Pushed %d bytes of data to stack", slip_rx_buflen);
                            _pslipmacdriver->slip_rx_state = SLIP_RX_STATE_SYNCSEARCH;
                            _pslipmacdriver->slip_rx_buflen = 0;

                            {
                                mbed::util::CriticalSectionLock lock;
                                _pslipmacdriver->pRxSlipBufferFreeList.push(pRxSlipBuffer);
                            }

                        break;

                        default:
                            _pslipmacdriver->slip_rx_buf[_pslipmacdriver->slip_rx_buflen++] = pRxSlipBuffer->buf[i];
                    }
                break;

                case SLIP_RX_STATE_ESCAPED:
                    switch(pRxSlipBuffer->buf[i])
                    {
                        case SLIP_ESC_END:
                            _pslipmacdriver->slip_rx_buf[_pslipmacdriver->slip_rx_buflen++] = SLIP_END;
                        break;
                        case SLIP_ESC_ESC:
                            _pslipmacdriver->slip_rx_buf[_pslipmacdriver->slip_rx_buflen++] = SLIP_ESC;
                        break;
                        default:
                            _pslipmacdriver->slip_rx_buf[_pslipmacdriver->slip_rx_buflen++] = pRxSlipBuffer->buf[i];
                    }
                    // return to unescaped processing
                    _pslipmacdriver->slip_rx_state = SLIP_RX_STATE_SYNCED;
                break;
            }
        }
    }
}

void SlipMACDriver::txIrq(void)
{
    static int i = 0;
    static SlipBuffer* pTxSlipBuffer = NULL;

    if(!pTxSlipBuffer)
    {
        if(!pTxSlipBufferToTxFuncList.pop(pTxSlipBuffer))
        {
            RawSerial::attach(NULL, Serial::TxIrq);
            return;
        }
    }

    if(pTxSlipBuffer->buf)
    {
        while(serial_writable(&_serial))
        {
            serial_putc(&_serial, (int)(pTxSlipBuffer->buf[i++]));
            if(i == pTxSlipBuffer->length)
            {
                i = 0;
                pTxSlipBufferFreeList.push(pTxSlipBuffer);
                pTxSlipBuffer = NULL;
                break;
            }
        }
    }
}

void SlipMACDriver::rxIrq(void)
{
    uint8_t character;
    static uint32_t stateSync = 0, rxbuflen = 0;
    static SlipBuffer* pTmpBuf;

    character = (uint8_t) serial_getc(&_serial);
    if(character == SLIP_END && stateSync == 0)
    {
        if(!pRxSlipBufferFreeList.pop(pTmpBuf))
            return;

        stateSync = 1;
        pTmpBuf->buf[rxbuflen++] = character;
    }
    else if(character == SLIP_END && stateSync == 1)
    {
        pTmpBuf->buf[rxbuflen] = character;
        pTmpBuf->length = rxbuflen;
        rxbuflen = stateSync = 0;
        pRxSlipBufferToRxFuncList.push(pTmpBuf);
        minar::Scheduler::postCallback(mbed::util::FunctionPointer0<void>(slip_rx).bind())
            .tolerance(minar::milliseconds(1));
    }
    else if(stateSync == 1)
    {
        pTmpBuf->buf[rxbuflen++] = character;
    }
    else
    {
        //igonre unsynchronized data
    }
}

int8_t SlipMACDriver::Slip_Init(uint8_t *mac)
{
    SlipBuffer *pTmpSlipBuffer;

    if (mac != NULL) {
        for (uint8_t i = 0; i < sizeof(slip_mac); ++i) {
            slip_mac[i] = mac[i];
        }
    }
	
    //Build driver data structure
    slip_phy_driver.PHY_MAC = slip_mac;
    slip_phy_driver.link_type = PHY_LINK_TUN;
    slip_phy_driver.data_request_layer = IPV6_DATAGRAMS_DATA_FLOW;
    slip_phy_driver.driver_description = (char *)"SLIP";
    slip_phy_driver.phy_MTU = SLIP_TX_RX_MAX_BUFLEN;
    slip_phy_driver.phy_tail_length = 0;
    slip_phy_driver.phy_header_length = 0;
    slip_phy_driver.state_control = 0;
    slip_phy_driver.tx = slip_if_tx;
    slip_phy_driver.address_write = 0;
    slip_phy_driver.extension = 0;

    // define and bring up interface
    net_slip_id = arm_net_phy_register(&slip_phy_driver);

    // init rx state machine
    tr_debug("SLIP driver id: %d\r\n",net_slip_id);

    for(int i = 0; i < SLIP_NR_BUFFERS; i++)
    {
        pTmpSlipBuffer = new SlipBuffer;

        for(int j = 0; j < SLIP_TX_RX_MAX_BUFLEN; j++)
            pTmpSlipBuffer->buf[j] = 0;
        pTmpSlipBuffer->length = 0;

        pRxSlipBufferFreeList.push(pTmpSlipBuffer);
    }

    for(int i = 0; i < SLIP_NR_BUFFERS; i++)
    {
        pTmpSlipBuffer = new SlipBuffer;

        for(int j = 0; j < SLIP_TX_RX_MAX_BUFLEN; j++)
            pTmpSlipBuffer->buf[j] = 0;
        pTmpSlipBuffer->length = 0;

        pTxSlipBufferFreeList.push(pTmpSlipBuffer);
    }

    baud(115200);

    RawSerial::attach(this, &SlipMACDriver::rxIrq, Serial::RxIrq);

    return net_slip_id;
}
