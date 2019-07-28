/*
 *  Copyright (c) 2019, The OpenThread Authors.
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
 *   This file includes the platform-specific configuration.
 *
 */

#ifndef PLATFORM_CONFIG_H_
#define PLATFORM_CONFIG_H_

#include "openthread-core-config.h"
#include <openthread/config.h>

/*******************************************************************************
 * @section UART Driver Configuration.
 ******************************************************************************/

/**
 * @def UART_INSTANCE
 *
 * UART Instance.
 *
 */
#ifndef UART_INSTANCE
#define UART_INSTANCE E_AHI_UART_0
#endif

#ifndef UART_BAUDRATE
#define UART_BAUDRATE E_AHI_UART_RATE_115200
#endif

#ifndef JN516X_802154_RX_BUFFERS
#define JN516X_802154_RX_BUFFERS 16
#endif

#ifndef MSTIMER_DEVICE
#define MSTIMER_DEVICE E_AHI_TIMER_0
#endif

#ifndef USTIMER_DEVICE
#define USTIMER_DEVICE E_AHI_TIMER_1
#endif

#ifndef DEBUG_UART 
#define DEBUG_UART DBG_E_UART_1
#endif

#ifndef DEBUG_BAUD 
#define DEBUG_BAUD DBG_E_UART_BAUD_RATE_115200
#endif

#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE 256
#endif

#ifndef UART_RX_FIFO_SIZE
#define UART_RX_FIFO_SIZE 2047
#endif

#ifndef UART_TX_FIFO_SIZE
#define UART_TX_FIFO_SIZE 1281
#endif

#endif //PLATFORM_CONFIG_H_
