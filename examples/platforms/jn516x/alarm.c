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
 *   This file implements the OpenThread platform abstraction for the alarm.
 *
 */

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/time.h>

#include "openthread-system.h"

#include "platform-config.h"
#include "platform-jn516x.h"

#define MICRO_DISABLE_AND_SAVE_INTERRUPTS(u32Store);                        \
    {                                                                       \
        __asm__ volatile ("bw.mfspr %0, r0, 17;" :"=r"(u32Store) : );       \
        __asm__ volatile ("b.di;" : : );                                    \
    }

#define MICRO_RESTORE_INTERRUPTS(u32Store);                                 \
        __asm__ volatile ("bw.mtspr r0, %0, 17;" : :"r"(u32Store));


typedef void (* CALLBACK_REGISTER_FN) (PR_HWINT_APPCALLBACK);

static void null_cbreg(PR_HWINT_APPCALLBACK x) { }


#define TIMER_MASK(x) (x == E_AHI_TIMER_0 ? E_AHI_DEVICE_TIMER0  : \
                      (x == E_AHI_TIMER_1 ? E_AHI_DEVICE_TIMER1 : \
                      (x == E_AHI_TIMER_2 ? E_AHI_DEVICE_TIMER2 : \
                      (x == E_AHI_TIMER_3 ? E_AHI_DEVICE_TIMER3 : \
                      (x == E_AHI_TIMER_4 ? E_AHI_DEVICE_TIMER4 : 0)))))

#define TIMER_CALLBACK_REGISTER_FN(x) \
        (x == E_AHI_TIMER_0 ? vAHI_Timer0RegisterCallback : \
        (x == E_AHI_TIMER_1 ? vAHI_Timer1RegisterCallback : \
        (x == E_AHI_TIMER_2 ? vAHI_Timer2RegisterCallback : \
        (x == E_AHI_TIMER_3 ? vAHI_Timer3RegisterCallback : \
        (x == E_AHI_TIMER_4 ? vAHI_Timer4RegisterCallback : null_cbreg)))))

#define MSTIMER_MASK TIMER_MASK(MSTIMER_DEVICE)
#define USTIMER_MASK TIMER_MASK(USTIMER_DEVICE)

#define US_PER_MS           1000ULL
#define US_PER_S            1000000ULL
#define US_PER_OVERFLOW     16 * US_PER_S

#define TICKS_TO_US(t)      (t / 16)
#define US_TO_TICKS(u)      (u * 16)

#define RTC_COUNTER_BITS    24  ///< Number of bits in RTC COUNTER register.

#define MS_PER_S            1000UL

#define XTAL_ACCURACY       40 // The crystal used on JN516x has ±20ppm accuracy.
// clang-format on

typedef enum
{
    kMsTimer,
    kUsTimer,
    kNumTimers
} AlarmIndex;

typedef struct
{
    volatile bool mFireAlarm;  ///< Information for processing function, that alarm should fire.
    volatile uint64_t mRemains;
} AlarmData;

typedef struct
{
    uint8_t        mTimerId;
    uint32_t       mDeviceId;
    uint8_t        mPrescaler;
    uint16_t       mFactor;
} AlarmChannelData;

static volatile uint32_t sOverflowCounter; ///< Counter of RTC overflowCounter, incremented by 2 on each OVERFLOW event.
static volatile bool     sEventPending;    ///< Timer fired and upper layer should be notified.
static AlarmData         sTimerData[kNumTimers]; ///< Data of the timers.

static const AlarmChannelData sChannelData[kNumTimers] = //
    {                                                    //
        [kMsTimer] =
            {
                .mTimerId    = MSTIMER_DEVICE,
                .mDeviceId  = MSTIMER_MASK,
                .mPrescaler   = 7,
                .mFactor      = 125,
            },
        [kUsTimer] =
            {
                .mTimerId    = USTIMER_DEVICE,
                .mDeviceId  = USTIMER_MASK,
                .mPrescaler   = 4,
                .mFactor      = 1,
            }
        };

static void TickTimerIntHandler(uint32 u32Device, uint32 u32ItemBitmap)
{
    sOverflowCounter++;
}

static void TimerIntHandler(uint32 u32Device, uint32 u32ItemBitmap)
{
    for(int i=0; i<kNumTimers; i++) {
        if(sChannelData[i].mDeviceId != u32Device) continue;

        if(sTimerData[i].mRemains <= 0) {
            vAHI_TimerStop(sChannelData[i].mTimerId);
            sTimerData[i].mFireAlarm = true;
            sEventPending            = true;
            otSysEventSignalPending();
        }
        else if(sTimerData[i].mRemains >= UINT16_MAX) {
            sTimerData[i].mRemains -= UINT16_MAX;
        }
        else {
            sTimerData[i].mRemains = 0;
        }
    }
}

static inline uint64_t TimeToTicks(uint64_t aTime, AlarmIndex aIndex)
{
    if (aIndex == kMsTimer)
    {
        aTime *= US_PER_MS;
    }

    return US_TO_TICKS(aTime);
}

static inline uint64_t TicksToTime(uint64_t aTicks, AlarmIndex aIndex)
{
    uint64_t result = TICKS_TO_US(aTicks);

    if (aIndex == kMsTimer)
    {
        result /= US_PER_MS;
    }

    return result;
}

static void GetOffsetAndCounter(uint32_t *aOffset, uint32_t *aCounter)
{
    uint32_t store_interrupts;
    MICRO_DISABLE_AND_SAVE_INTERRUPTS(store_interrupts);
    *aOffset = sOverflowCounter;
    *aCounter = u32AHI_TickTimerRead();
    MICRO_RESTORE_INTERRUPTS(store_interrupts);
}

static uint64_t GetTime(uint32_t aOffset, uint32_t aCounter, AlarmIndex aIndex)
{
    uint64_t result = (uint64_t)aOffset * US_PER_OVERFLOW + TicksToTime(aCounter, kUsTimer);

    if (aIndex == kMsTimer)
    {
        result /= US_PER_MS;
    }

    return result;
}

static uint64_t GetCurrentTime(AlarmIndex aIndex)
{
    uint32_t offset;
    uint32_t rtc_counter;

    GetOffsetAndCounter(&offset, &rtc_counter);

    return GetTime(offset, rtc_counter, aIndex);
}


static void AlarmStartAt(uint32_t aT0, uint32_t aDt, AlarmIndex aIndex)
{
    uint64_t count = (aT0 + aDt) * sChannelData[aIndex].mFactor;
    uint16_t interval = UINT16_MAX;
    if(count > UINT16_MAX) {
        sTimerData[aIndex].mRemains = (count - UINT16_MAX);
    }
    else {
        sTimerData[aIndex].mRemains = 0;
        interval = count;
    }
    vAHI_TimerStartRepeat(sChannelData[aIndex].mTimerId, aT0, interval);
}

static void AlarmStop(AlarmIndex aIndex)
{
    vAHI_TimerStop(sChannelData[aIndex].mTimerId);
    sTimerData[aIndex].mFireAlarm = false;
}

void jn516xAlarmInit(void)
{
    vAHI_TickTimerInterval(16*16000000);
    vAHI_TickTimerRegisterCallback(TickTimerIntHandler);
    vAHI_TickTimerIntEnable(true);
    vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_RESTART);

    for(uint32_t i=0; i<kNumTimers; i++) {
        vAHI_TimerEnable(sChannelData[i].mTimerId, sChannelData[i].mPrescaler, false, true, false);
        TIMER_CALLBACK_REGISTER_FN(sChannelData[i].mTimerId)(TimerIntHandler);
    }
}

void jn516xAlarmDeinit(void)
{
    vAHI_TickTimerIntEnable(false);
    vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_DISABLE);

    for (uint32_t i = 0; i < kNumTimers; i++)
    {
        vAHI_TimerDisable(sChannelData[i].mTimerId);
    }
}

void jn516xAlarmProcess(otInstance *aInstance)
{
    do
    {
        sEventPending = false;

        if (sTimerData[kMsTimer].mFireAlarm)
        {
            sTimerData[kMsTimer].mFireAlarm = false;

#if OPENTHREAD_CONFIG_DIAG_ENABLE

            if (otPlatDiagModeGet())
            {
                otPlatDiagAlarmFired(aInstance);
            }
            else
#endif
            {
                otPlatAlarmMilliFired(aInstance);
            }
        }

        if (sTimerData[kUsTimer].mFireAlarm)
        {
            sTimerData[kUsTimer].mFireAlarm = false;

            otPlatAlarmMicroFired(aInstance);
        }
    } while (sEventPending);
}

static inline uint64_t GetCurrentTimeUs(void)
{
    return GetCurrentTime(kUsTimer);
}

uint32_t otPlatAlarmMilliGetNow(void)
{
    return (uint32_t)(GetCurrentTimeUs() / US_PER_MS);
}

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    OT_UNUSED_VARIABLE(aInstance);

    AlarmStartAt(aT0, aDt, kMsTimer);
}

void otPlatAlarmMilliStop(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    AlarmStop(kMsTimer);
}

uint32_t otPlatAlarmMicroGetNow(void)
{
    return (uint32_t)GetCurrentTimeUs();
}

void otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    OT_UNUSED_VARIABLE(aInstance);

    AlarmStartAt(aT0, aDt, kUsTimer);
}

void otPlatAlarmMicroStop(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    AlarmStop(kUsTimer);
}

#if OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
uint64_t otPlatTimeGet(void)
{
    return GetCurrentTimeUs();
}

uint16_t otPlatTimeGetXtalAccuracy(void)
{
    return XTAL_ACCURACY;
}
#endif // OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
