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

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <utils/code_utils.h>
#include <utils/flash.h>
#include <openthread/platform/alarm-milli.h>

#include "platform-jn516x.h"


#ifndef SETTINGS_CONFIG_PAGE_SIZE
#define SETTINGS_CONFIG_PAGE_SIZE 0x800
#endif // SETTINGS_CONFIG_PAGE_SIZE

static uint8_t  sSegmentLength;
static uint16_t sSegmentNumber;

static uint8_t addr2eep(uint32_t address, uint32_t length, uint16_t* eepindex, uint8_t* eepaddr) {
    uint16_t idx = address / sSegmentLength;
    if(idx >= sSegmentNumber) {
        return -1;
    }
    *eepindex = idx;
    *eepaddr = address % sSegmentLength;

    if (length < sSegmentLength) {
        return length;
    }
    else {
        return sSegmentLength - *eepaddr;
    }
}

otError utilsFlashInit(void)
{
    sSegmentNumber = u16AHI_InitialiseEEP(&sSegmentLength);
    return OT_ERROR_NONE;
}

uint32_t utilsFlashGetSize(void)
{
    return (sSegmentNumber * sSegmentLength);
}

otError utilsFlashErasePage(uint32_t aAddress)
{
    uint16_t pageaddr = (aAddress / SETTINGS_CONFIG_PAGE_SIZE) * SETTINGS_CONFIG_PAGE_SIZE;
    uint32_t size = SETTINGS_CONFIG_PAGE_SIZE;
    int err = 0;

    while(size > 0) {
        uint16_t eepindex = 0;
        uint8_t eepaddr = 0;
        uint8_t len = addr2eep(pageaddr, size, &eepindex, &eepaddr);
        err = iAHI_EraseEEPROMsegment(eepindex);

        if(err) break;

        pageaddr += len;
        size -= len;
    }
    return OT_ERROR_NONE;
}

otError utilsFlashStatusWait(uint32_t aTimeout)
{
    OT_UNUSED_VARIABLE(aTimeout);
    return OT_ERROR_NONE;
}

uint32_t utilsFlashWrite(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
    uint32_t result = 0;
    otEXPECT(aData);
    otEXPECT(aAddress < utilsFlashGetSize());
    otEXPECT(aSize);

    while(aSize != 0) {
        uint16_t eepindex = 0;
        uint8_t eepaddr = 0;
        uint8_t len = addr2eep(aAddress, aSize, &eepindex, &eepaddr);
        int err = iAHI_WriteDataIntoEEPROMsegment(eepindex, eepaddr, aData, len);

        if(err) break;

        aAddress += len;
        aData += len;
        aSize -= len;
        result += len;
    }

exit:
    return result;
}

uint32_t utilsFlashRead(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
    uint32_t result = 0;
    otEXPECT(aData);
    otEXPECT(aAddress < utilsFlashGetSize());

    while(aSize != 0) {
        uint16_t eepindex = 0;
        uint8_t eepaddr = 0;
        uint8_t len = addr2eep(aAddress, aSize, &eepindex, &eepaddr);
        int err = iAHI_ReadDataFromEEPROMsegment(eepindex, eepaddr, aData, len);

        if(err) break;

        aAddress += len;
        aData += len;
        aSize -= len;
        result += len;
    }
    result = aSize;

exit:
    return result;
}
