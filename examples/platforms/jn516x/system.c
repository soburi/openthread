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
 *   This file includes the platform-specific initializers.
 *
 */

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <openthread/platform/logging.h>

#include "openthread-system.h"
#include "platform-jn516x.h"

#include <dbg_uart.h>

#include <openthread/config.h>
#include <openthread/platform/toolchain.h>

extern int main(int argc, char* argv[]);
void AppColdStart(void);
void AppWarmStart(void);
void __cxa_pure_virtual(void);

extern bool gPlatformPseudoResetWasRequested;

void __cxa_pure_virtual(void)
{
    while (1)
        ;
}

extern int main(int argc, char* argv[]);

__WEAK void AppColdStart()
{
    vAHI_WatchdogStop();
    (void)u32AHI_Init();
    char* argv[] = { "ColdStart" };
    main(1, argv );
}

__WEAK void AppWarmStart()
{
}

void otSysInit(int argc, char *argv[])
{
    OT_UNUSED_VARIABLE(argc);
    OT_UNUSED_VARIABLE(argv);

    if (gPlatformPseudoResetWasRequested)
    {
        otSysDeinit();
    }

#if (OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED) || \
    (OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_NCP_SPINEL)
    jn516xLogInit();
#endif
    jn516xAlarmInit();
    jn516xRandomInit();
    if (!gPlatformPseudoResetWasRequested)
    {
#if ((UART_AS_SERIAL_TRANSPORT == 1) || (USB_CDC_AS_SERIAL_TRANSPORT == 1))
        jn516xUartInit();
#endif
    }
    else
    {
#if ((UART_AS_SERIAL_TRANSPORT == 1) || (USB_CDC_AS_SERIAL_TRANSPORT == 1))
        jn516xUartClearPendingData();
#endif
    }

#if (SPIS_AS_SERIAL_TRANSPORT == 1)
    jn516xSpiSlaveInit();
#endif
    jn516xMiscInit();
    jn516xRadioInit();
    //jn516xTempInit();


    gPlatformPseudoResetWasRequested = false;
}

void otSysDeinit(void)
{
    //jn516xTempDeinit();
    jn516xRadioDeinit();
    jn516xMiscDeinit();
#if (SPIS_AS_SERIAL_TRANSPORT == 1)
    jn516xSpiSlaveDeinit();
#endif
    if (!gPlatformPseudoResetWasRequested)
    {
#if ((UART_AS_SERIAL_TRANSPORT == 1) || (USB_CDC_AS_SERIAL_TRANSPORT == 1))
        jn516xUartDeinit();
#endif
    }
    jn516xRandomDeinit();
    jn516xAlarmDeinit();
#if (OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED) || \
    (OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_NCP_SPINEL)
    jn516xLogDeinit();
#endif
}

bool otSysPseudoResetWasRequested(void)
{
    return gPlatformPseudoResetWasRequested;
}

void otSysProcessDrivers(otInstance *aInstance)
{
    jn516xRadioProcess(aInstance);
#if ((UART_AS_SERIAL_TRANSPORT == 1) || (USB_CDC_AS_SERIAL_TRANSPORT == 1))
    jn516xUartProcess();
#endif
#if (SPIS_AS_SERIAL_TRANSPORT == 1)
    jn516xSpiSlaveProcess();
#endif
    //jn516xTempProcess();
    jn516xAlarmProcess(aInstance);
}

__WEAK void otSysEventSignalPending(void)
{
    // Intentionally empty
}
