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
 *   This file implements the OpenThread platform abstraction for logging.
 *
 */

#include <openthread-core-config.h>
#include <openthread/config.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/logging.h>

#include <utils/code_utils.h>

#include <dbg_uart.h>

#include "platform-config.h"
#include "platform-jn516x.h"

#ifndef LOG_PARSE_BUFFER_SIZE
#define LOG_PARSE_BUFFER_SIZE 128
#endif

static bool    sLogInitialized = false;

static const char* REGION_NAME[] = {
    "        ",
    "     API",
    "     MLE",
    "     ARP",
    "NET_DATA",
    "    ICMP",
    "     IP6",
    "     MAC",
    "     MEM",
    "     NCP",
    "MESH_COP",
    "NET_DIAG",
    "PLATFORM",
    "    COAP",
    "     CLI",
    "    CORE",
    "    UTIL",
};

#if (OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED) || \
    (OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_NCP_SPINEL)
void jn516xLogInit(void)
{
    DBG_vUartInit(DEBUG_UART, DEBUG_BAUD);
    sLogInitialized = true;
}

void jn516xLogDeinit(void)
{
    //utilsLogRttDeinit();
    sLogInitialized = false;
}

OT_TOOL_WEAK void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
    va_list ap;

    va_start(ap, aFormat);

    uint16_t length = 0;
    int      charsWritten;
    char     logString[LOG_PARSE_BUFFER_SIZE + 1];

    otEXPECT(sLogInitialized == true);
    otEXPECT(aLogRegion < sizeof(REGION_NAME));

    charsWritten = vsnprintf(&logString[length], (size_t)(LOG_PARSE_BUFFER_SIZE - length), aFormat, ap);
    otEXPECT(charsWritten >= 0);
    length += charsWritten;

    if (length > LOG_PARSE_BUFFER_SIZE)
    {
        length = LOG_PARSE_BUFFER_SIZE;
    }

    logString[length++] = '\0';

//#if (LOG_REGION_ENABLE == 1)
    DBG_vPrintf(1, "%s", REGION_NAME[aLogRegion]);
//#endif
//#if (LOG_TIMESTAMP_ENABLE == 1)
    DBG_vPrintf(1, "[%010lu] ", otPlatAlarmMilliGetNow() );
//#endif
    DBG_vPrintf(1, "%s\n", logString);

exit:
    va_end(ap);
    return;
}
#endif


