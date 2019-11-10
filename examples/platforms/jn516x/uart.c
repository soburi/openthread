/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for UART communication.
 *
 */

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include <utils/code_utils.h>
#include <openthread/platform/toolchain.h>
#include <openthread/platform/uart.h>

#include "openthread-system.h"

#include "platform-jn516x.h"
#include "platform-config.h"


typedef void (* CALLBACK_REGISTER_FN) (PR_HWINT_APPCALLBACK);

static void null_cbreg(PR_HWINT_APPCALLBACK x) { }

#define UART_CALLBACK_REGISTER_FN(x) \
        (x == E_AHI_UART_0 ? vAHI_Uart0RegisterCallback : \
        (x == E_AHI_UART_1 ? vAHI_Uart1RegisterCallback : null_cbreg))

static void jn516xUartInterruptHandler(uint32_t device_id, uint32_t item_bitmap);


static uint8_t sTxFifo[UART_TX_FIFO_SIZE];
static uint8_t sRxFifo[UART_RX_FIFO_SIZE];

#if (UART_AS_SERIAL_TRANSPORT == 1)

bool sUartEnabled = false;

/**
 *  UART TX buffer variables.
 */
//static const uint8_t *sTransmitBuffer = NULL;
//static uint16_t       sTransmitLength = 0;
static bool           sTransmitDone   = 0;

/**
 *  UART RX ring buffer variables.
 */
static uint8_t  sReceiveBuffer[UART_RX_BUFFER_SIZE];
static uint16_t sReceiveHead = 0;
static uint16_t sReceiveTail = 0;

/**
 * Function for checking if RX buffer is full.
 *
 * @retval true  RX buffer is full.
 * @retval false RX buffer is not full.
 */
static __INLINE bool isRxBufferFull()
{
    uint16_t next = (sReceiveHead + 1) % UART_RX_BUFFER_SIZE;
    return (next == sReceiveTail);
}

/**
 * Function for checking if RX buffer is empty.
 *
 * @retval true  RX buffer is empty.
 * @retval false RX buffer is not empty.
 */
static __INLINE bool isRxBufferEmpty()
{
    return (sReceiveHead == sReceiveTail);
}

/**
 * Function for notifying application about new bytes received.
 */
static void processReceive(void)
{
    // Set head position to not be changed during read procedure.
    uint16_t head = sReceiveHead;

    otEXPECT(isRxBufferEmpty() == false);

    // In case head roll back to the beginning of the buffer, notify about left
    // bytes from the end of the buffer.
    if (head < sReceiveTail)
    {
        otPlatUartReceived(&sReceiveBuffer[sReceiveTail], (sizeof(sRxFifo)- sReceiveTail));
        sReceiveTail = 0;
    }

    // Notify about received bytes.
    if (head > sReceiveTail)
    {
        otPlatUartReceived(&sReceiveBuffer[sReceiveTail], (head - sReceiveTail));
        sReceiveTail = head;
    }

exit:
    return;
}

otError otPlatUartFlush(void)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

/**
 * Function for notifying application about transmission being done.
 */
static void processTransmit(void)
{
    //otEXPECT(sTransmitBuffer != NULL);

    if (sTransmitDone)
    {
        // Clear Transmition transaction and notify application.
        //sTransmitBuffer = NULL;
        //sTransmitLength = 0;
        sTransmitDone   = false;
        otPlatUartSendDone();
    }

//exit:
    return;
}

void jn516xUartProcess(void)
{
    processReceive();
    processTransmit();
}

void jn516xUartInit(void)
{
    // Intentionally empty.
}

void jn516xUartClearPendingData(void)
{
    // Intentionally empty.
}

void jn516xUartDeinit(void)
{
    if (sUartEnabled)
    {
        otPlatUartDisable();
    }
}

otError otPlatUartEnable(void)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(sUartEnabled == false, error = OT_ERROR_ALREADY);

    vAHI_UartSetRTSCTS(UART_INSTANCE, false);

    uint8_t uart_enabled = bAHI_UartEnable(UART_INSTANCE, sTxFifo, sizeof(sTxFifo),
                                                          sRxFifo, sizeof(sRxFifo));
    otEXPECT_ACTION(uart_enabled == true, error = OT_ERROR_FAILED);

    vAHI_UartReset(UART_INSTANCE, true, true);
    vAHI_UartReset(UART_INSTANCE, false, false);
    
    vAHI_UartSetClockDivisor(UART_INSTANCE, UART_BAUDRATE);

    while((u8AHI_UartReadLineStatus(UART_INSTANCE) & E_AHI_UART_LS_THRE) == 0);
    
    UART_CALLBACK_REGISTER_FN(UART_INSTANCE)(jn516xUartInterruptHandler);
    vAHI_UartSetInterrupt(UART_INSTANCE, false, false, false, true, E_AHI_UART_FIFO_LEVEL_14);

    sUartEnabled = true;
exit:
    return error;
}

otError otPlatUartDisable(void)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(sUartEnabled == true, error = OT_ERROR_ALREADY);

    vAHI_UartDisable(UART_INSTANCE);
    sUartEnabled = false;

exit:
    return error;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(u16AHI_UartReadTxFifoLevel(UART_INSTANCE) < sizeof(sTxFifo), error = OT_ERROR_FAILED);

    for(int i=0; i<aBufLength; i++) {
        volatile int16_t write = 0;
        vAHI_UartWriteData(UART_INSTANCE, aBuf[i]);
    }
    sTransmitDone = true;

exit:
    return error;
}


static void jn516xUartInterruptHandler(uint32_t device_id, uint32_t item_bitmap)
{
    int status = 0;
    int c = 0;
    
    switch(item_bitmap) {
        case E_AHI_UART_INT_TIMEOUT:
        case E_AHI_UART_INT_RXDATA:
            while(u16AHI_UartReadRxFifoLevel(UART_INSTANCE) > 0 && c++ < 32 && status == 0) {
                if (isRxBufferFull()) {
                    status = -1;
                    break;
                }
                sReceiveBuffer[sReceiveHead] = u8AHI_UartReadData(UART_INSTANCE);
                sReceiveHead                 = (sReceiveHead + 1) % UART_RX_BUFFER_SIZE;
            }
            otSysEventSignalPending();
            break;
        case E_AHI_UART_INT_TX:
        case E_AHI_UART_INT_RXLINE:
            break;
    }
}

#endif // UART_AS_SERIAL_TRANSPORT == 1

/**
 * The UART driver weak functions definition.
 *
 */
OT_TOOL_WEAK void otPlatUartSendDone(void)
{
}

OT_TOOL_WEAK void otPlatUartReceived(const uint8_t *aBuf, uint16_t aBufLength)
{
    OT_UNUSED_VARIABLE(aBuf);
    OT_UNUSED_VARIABLE(aBufLength);
}
