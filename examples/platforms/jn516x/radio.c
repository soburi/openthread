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
 *   This file implements the OpenThread platform abstraction for radio communication.
 *
 */

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "utils/code_utils.h"

#include <platform-config.h>
#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/logging.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/time.h>

#include "openthread-system.h"
#include "platform-config.h"
#include "platform-jn516x.h"

#include <MMAC.h>

#include <openthread-core-config.h>
#include <openthread/config.h>
#include <openthread/random_noncrypto.h>

// clang-format off

#define SHORT_ADDRESS_SIZE    2            ///< Size of MAC short address.
#define US_PER_MS             1000ULL      ///< Microseconds in millisecond.

#define ACK_REQUEST_OFFSET    0            ///< Byte containing Ack request bit
#define ACK_REQUEST_BIT       (1 << 5)     ///< Ack request bit.
#define FRAME_PENDING_OFFSET  0            ///< Byte containing pending bit
#define FRAME_PENDING_BIT     (1 << 4)     ///< Frame Pending bit.


#define CHECKSUM_LEN 2
#if (JENNIC_CHIP == JN5169)
#define OUTPUT_POWER_MAX      10
#define OUTPUT_POWER_MIN      (-32)
#define ABS_OUTPUT_POWER_MIN  (32)
#else
#define OUTPUT_POWER_MAX      0
#define OUTPUT_POWER_MIN      (-32)
#endif

/* Default Tx power [dBm] (between OUTPUT_POWER_MIN and OUTPUT_POWER_MAX) */
#define DEFAULT_TX_POWER 0

/* this approximate formula for RSSI is taken from NXP internal docs */
#define ED2DBM(x) ((7*(x) - 1970) / 20)

typedef struct jn516xPhyFrame {
    tsPhyFrame phy;
    otRadioFrame ot;
} jn516xPhyFrame;

typedef struct jn516xReceiveBuffer {
    volatile jn516xPhyFrame buffer[JN516X_802154_RX_BUFFERS];
    volatile jn516xPhyFrame *head;
    volatile jn516xPhyFrame *tail;
} jn516xReceiveBuffer;

// clang-format on

enum
{
    JN516X_RECEIVE_SENSITIVITY  = -95, // dBm
};

static bool sDisabled;

static otError      sReceiveError = OT_ERROR_NONE;
static volatile jn516xReceiveBuffer sReceivedFrames;
static otRadioFrame sTransmitFrame;
static tsPhyFrame   sTransmitPsdu;

#if OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT
static otRadioIeInfo sTransmitIeInfo;
static otInstance *  sInstance = NULL;
#endif

static otRadioFrame sAckFrame;
static bool         sAckedWithFramePending;

static int8_t sDefaultTxPower;

static int8_t   sEnergyDetected;

static bool sPromiscuous;

static uint8_t sCurrentChannel;
static uint16_t sCurrentPanId;
static uint16_t sCurrentShortAddress;
static otExtAddress sCurrentExtendedAddress;

static volatile otRadioState sRadioState;

static void radio_interrupt_handler(uint32_t event);
static void jn516x_frame_free(uint8_t *frametofree);



static int recvframe_count() {
    if(sReceivedFrames.head >= sReceivedFrames.tail) {
        return   (sizeof(sReceivedFrames.buffer-sReceivedFrames.head) +
                  sizeof(sReceivedFrames.tail-sReceivedFrames.buffer) )
		/ sizeof(tsPhyFrame*);
    }
        return   (sizeof(sReceivedFrames.buffer-sReceivedFrames.tail) +
                  sizeof(sReceivedFrames.head-sReceivedFrames.buffer) )
		/ sizeof(tsPhyFrame*);
}

static volatile jn516xPhyFrame* recvframe_pop() {
    if(recvframe_count() == 0) return NULL;

    sReceivedFrames.tail++;
    if(sReceivedFrames.tail > (sReceivedFrames.buffer+(sizeof(sReceivedFrames.buffer)/sizeof(jn516xPhyFrame)) ) ) {
        sReceivedFrames.tail -= (sizeof(sReceivedFrames.buffer)/sizeof(jn516xPhyFrame));
    }
    return sReceivedFrames.tail;
}

static volatile jn516xPhyFrame* recvframe_next() {
    if(recvframe_count() == (sizeof(sReceivedFrames.buffer)/sizeof(jn516xPhyFrame)) ) return NULL;

    sReceivedFrames.head++;
    if(sReceivedFrames.head >= (sReceivedFrames.buffer+(sizeof(sReceivedFrames.buffer)/sizeof(jn516xPhyFrame)) ) ) {
        sReceivedFrames.tail -= (sizeof(sReceivedFrames.buffer)/sizeof(jn516xPhyFrame));
    }
    return sReceivedFrames.head;
}

static volatile jn516xPhyFrame* recvframe_head() {
    return sReceivedFrames.head;
}


typedef enum
{
    kPendingEventSleep,                // Requested to enter Sleep state.
    kPendingEventFrameTransmitted,     // Transmitted frame and received ACK (if requested).
    kPendingEventChannelAccessFailure, // Failed to transmit frame (channel busy).
    kPendingEventInvalidOrNoAck,       // Failed to transmit frame (received invalid or no ACK).
    kPendingEventReceiveFailed,        // Failed to receive a valid frame.
    kPendingEventEnergyDetected,       // Energy Detection finished.
} RadioPendingEvents;

static volatile uint32_t sPendingEvents;

static void dataInit(void)
{
    sDisabled = true;

    sTransmitFrame.mPsdu = sTransmitPsdu.uPayload.au8Byte;
#if OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT
    sTransmitFrame.mInfo.mTxInfo.mIeInfo = &sTransmitIeInfo;
#endif

    sReceiveError = OT_ERROR_NONE;

    sReceivedFrames.head = sReceivedFrames.buffer;
    sReceivedFrames.tail = sReceivedFrames.buffer;

    memset(&sAckFrame, 0, sizeof(sAckFrame));
}

static void convertShortAddress(uint8_t *aTo, uint16_t aFrom)
{
    aTo[0] = (uint8_t)aFrom;
    aTo[1] = (uint8_t)(aFrom >> 8);
}

static inline bool isPendingEventSet(RadioPendingEvents aEvent)
{
    return sPendingEvents & (1UL << aEvent);
}

static void setPendingEvent(RadioPendingEvents aEvent)
{
    uint32_t          bitToSet = 1UL << aEvent;

    sPendingEvents |= bitToSet;

    otSysEventSignalPending();
}

static void resetPendingEvent(RadioPendingEvents aEvent)
{
    uint32_t          bitsToRemain = ~(1UL << aEvent);

    sPendingEvents &= bitsToRemain;
}

static inline void clearPendingEvents(void)
{
    // Clear pending events that could cause race in the MAC layer.
    uint32_t          bitsToRemain = ~(0UL);

    bitsToRemain &= ~(1UL << kPendingEventSleep);

    sPendingEvents &= bitsToRemain;
}

#if !OPENTHREAD_CONFIG_ENABLE_PLATFORM_EUI64_CUSTOM_SOURCE
void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
	(void)aInstance;
	(void)aIeeeEui64;
}
#endif // OPENTHREAD_CONFIG_ENABLE_PLATFORM_EUI64_CUSTOM_SOURCE

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanId)
{
	(void)aInstance;
	(void)aPanId;
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aExtAddress)
{
	(void)aInstance;
	(void)aExtAddress;
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aShortAddress)
{
	(void)aInstance;
	(void)aShortAddress;
}

void jn516xRadioInit(void)
{
    dataInit();
    vMMAC_Enable();
    vMMAC_ConfigureInterruptSources(E_MMAC_INT_RX_COMPLETE | E_MMAC_INT_TX_COMPLETE);
    vMMAC_EnableInterrupts(radio_interrupt_handler);
    vMMAC_ConfigureRadio();
    
    /* Disable hardware backoff */
    vMMAC_SetTxParameters(1, 0, 0, 0);
    vMMAC_SetCutOffTimer(0, FALSE);
    sCurrentChannel = 26;
    sDefaultTxPower = DEFAULT_TX_POWER;
    vMMAC_SetChannelAndPower(sCurrentChannel, sDefaultTxPower);

    sRadioState = OT_RADIO_STATE_SLEEP;
}

void jn516xRadioDeinit(void)
{
    vMMAC_RadioOff();
    sRadioState = OT_RADIO_STATE_SLEEP;
    sPendingEvents = 0;
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    if (sDisabled)
    {
        return OT_RADIO_STATE_DISABLED;
    }


    return sRadioState;
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return !sDisabled;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
	(void)aInstance;
	return 0;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
	(void)aInstance;
	return 0;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
	(void)aInstance;
	return 0;
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
	(void)aInstance;
	(void)aChannel;
	return 0;
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
	(void)aInstance;
	(void)aFrame;
	return 0;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return &sTransmitFrame;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return 0;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return (otRadioCaps)(0);
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return sPromiscuous;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    OT_UNUSED_VARIABLE(aInstance);

    sPromiscuous = aEnable;
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aEnable);
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aShortAddress);
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aExtAddress);
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aShortAddress);
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aExtAddress);
    return OT_ERROR_NOT_IMPLEMENTED;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    sEnergyDetected = ED2DBM(u8MMAC_EnergyDetect(aScanChannel));

    setPendingEvent(kPendingEventEnergyDetected);

    return OT_ERROR_NONE;
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    OT_UNUSED_VARIABLE(aInstance);

    OT_UNUSED_VARIABLE(aPower);

    return 0;
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
    OT_UNUSED_VARIABLE(aInstance);

    sDefaultTxPower = aPower;
    OT_UNUSED_VARIABLE(aPower);

    return OT_ERROR_NONE;
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aThreshold);
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aThreshold);
    return OT_ERROR_NOT_IMPLEMENTED;
}

void jn516xRadioProcess(otInstance *aInstance)
{
}

static void jn516x_802154_received_timestamp_raw(volatile jn516xPhyFrame *phyframe, int8_t power, uint8_t lqi, uint32_t time)
{
    volatile otRadioFrame *receivedFrame = &(phyframe->ot);

    receivedFrame->mPsdu               = (uint8_t*)phyframe->phy.uPayload.au8Byte;
    receivedFrame->mLength             = phyframe->phy.u8PayloadLength;
    receivedFrame->mInfo.mRxInfo.mRssi = power;
    receivedFrame->mInfo.mRxInfo.mLqi  = lqi;
    receivedFrame->mChannel            = sCurrentChannel;

    // Inform if this frame was acknowledged with frame pending set.
    if (receivedFrame->mPsdu[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT)
    {
        receivedFrame->mInfo.mRxInfo.mAckedWithFramePending = sAckedWithFramePending;
    }
    else
    {
        receivedFrame->mInfo.mRxInfo.mAckedWithFramePending = false;
    }

    // Get the timestamp when the SFD was received.
    /* Wait for an edge */
    uint32_t t = u32MMAC_GetTime();
    while(u32MMAC_GetTime() == t);
    /* The remaining measured error is typically in range 0..16 usec.
     * Center it around zero, in the -8..+8 usec range. */
    const uint32_t RADIOCLK2US = 1000000/62500;
    const uint32_t CORRECTION  = RADIOCLK2US/2;
    /* Save SFD timestamp, converted from radio timer to RTIMER */
    uint64_t offset = ((u32MMAC_GetTime() - (u32MMAC_GetRxTime() - 1)) * RADIOCLK2US) - CORRECTION;
    receivedFrame->mInfo.mRxInfo.mTimestamp = otPlatAlarmMicroGetNow() - offset;

    sAckedWithFramePending = false;

    otSysEventSignalPending();
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return JN516X_RECEIVE_SENSITIVITY;
}

static int
is_packet_for_us(volatile uint8_t *buf, int len, int do_send_ack)
{
#if 0
  frame802154_t frame;
  int result;
  uint8_t parsed = frame802154_parse(buf, len, &frame);
  if(parsed) {
    if(frame.fcf.dest_addr_mode) {
      int has_dest_panid;
      frame802154_has_panid(&frame.fcf, NULL, &has_dest_panid);
      if(has_dest_panid
         && frame802154_get_pan_id() != FRAME802154_BROADCASTPANDID
         && frame.dest_pid != frame802154_get_pan_id()
         && frame.dest_pid != FRAME802154_BROADCASTPANDID) {
        /* Packet to another PAN */
        return 0;
      }
      if(!is_broadcast_addr(frame.fcf.dest_addr_mode, frame.dest_addr)) {
        result = linkaddr_cmp((linkaddr_t *)frame.dest_addr, &linkaddr_node_addr);
        if(autoack_enabled && result && do_send_ack) {
          /* this is a unicast frame and sending ACKs is enabled */
          send_ack(&frame);
        }
        return result;
      }
    }
    return 1;
  } else {
    return 0;
  }
#endif
  return 0;
}

static void radio_interrupt_handler(uint32_t mac_event)
{
    if(mac_event & E_MMAC_INT_TX_COMPLETE) {
        sRadioState = OT_RADIO_STATE_RECEIVE;

        uint32_t tx_error = u32MMAC_GetTxErrors();
        if(tx_error == 0) {
            setPendingEvent(kPendingEventFrameTransmitted);
        } else if(tx_error & E_MMAC_TXSTAT_CCA_BUSY) {
            setPendingEvent(kPendingEventChannelAccessFailure);
        } else {
            setPendingEvent(kPendingEventInvalidOrNoAck);
        }
    }
    else if(mac_event & E_MMAC_INT_RX_COMPLETE) {
        volatile jn516xPhyFrame *rx_frame_buffer = recvframe_head();
        uint32_t rx_status = u32MMAC_GetRxErrors();
        if(rx_status != 0) {
            switch (rx_status)
	    {
            case E_MMAC_RXSTAT_ERROR:
                sReceiveError = OT_ERROR_FCS;
                break;
            default:
                sReceiveError = OT_ERROR_FAILED;
                break;
            }
            sAckedWithFramePending = false;
            setPendingEvent(kPendingEventReceiveFailed);
	}
        else if(rx_frame_buffer->phy.u8PayloadLength > CHECKSUM_LEN) {
            int packet_for_me = 0;
            if(!sPromiscuous) {
                /* Check RX address */
                packet_for_me = is_packet_for_us(rx_frame_buffer->phy.uPayload.au8Byte, rx_frame_buffer->phy.u8PayloadLength - CHECKSUM_LEN, 1);
            } else {//if(!frame_filtering) {
                packet_for_me = 1;
            }

            if(!packet_for_me) {
                /* Prevent reading */
                rx_frame_buffer->phy.u8PayloadLength = 0;
            } else {
                /* read and cache RSSI and LQI values */
                uint8_t lqi;
                int8_t rssi = ED2DBM(u8MMAC_GetRxLqi(&lqi));

                jn516x_802154_received_timestamp_raw(rx_frame_buffer, rssi, lqi, 0);
                rx_frame_buffer = recvframe_next();
		if(!rx_frame_buffer) {
		}
	    }
        }
    }
}
