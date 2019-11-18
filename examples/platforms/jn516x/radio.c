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

// clang-format on
#define CHECKSUM_LEN 2
#if (JENNIC_CHIP == JN5169)
#define OUTPUT_POWER_MAX      10
#define OUTPUT_POWER_MIN      (-32)
#define ABS_OUTPUT_POWER_MIN  (32)
#else
#define OUTPUT_POWER_MAX      0
#define OUTPUT_POWER_MIN      (-32)
#endif

#define FRAME802154_BEACONFRAME     (0x00)
#define FRAME802154_DATAFRAME       (0x01)
#define FRAME802154_ACKFRAME        (0x02)
#define FRAME802154_CMDFRAME        (0x03)

#define FRAME802154_IEEERESERVED    (0x00)
#define FRAME802154_NOADDR          (0x00)      /**< Only valid for ACK or Beacon frames. */
#define FRAME802154_SHORTADDRMODE   (0x02)
#define FRAME802154_LONGADDRMODE    (0x03)

#define FRAME802154_IEEE802154_2003  (0x00)
#define FRAME802154_IEEE802154_2006  (0x01)
#define FRAME802154_IEEE802154_2015  (0x02)

#define FRAME802154_BROADCASTADDR   (0xFFFF)
#define FRAME802154_BROADCASTPANDID (0xFFFF)

/**
 * \brief Defines the bitfields of the frame control field (FCF).
 */
typedef struct {
  unsigned int frame_type                  : 3; /** Frame type field, see 802.15.4 */
  unsigned int security_enabled            : 1; /** True if security is used in this frame */
  unsigned int frame_pending               : 1; /** True if sender has more data to send */
  unsigned int ack_required                : 1; /** Is an ack frame required? */
  unsigned int panid_compression           : 1; /** Is this a compressed header? */
  unsigned int reserved                    : 1;            /** Unused bit */
  unsigned int sequence_number_suppression : 1; /** Does the header omit sequence number?, see 802.15.4e */
  unsigned int ie_list_present             : 1; /** Does the header contain Information Elements?, see 802.15.4e */
  unsigned int dest_addr_mode              : 2; /** Destination address mode, see 802.15.4 */
  unsigned int frame_version               : 2; /** 802.15.4 frame version */
  unsigned int src_addr_mode               : 2; /** Source address mode, see 802.15.4 */
} frame802154_fcf_t __attribute__((__packed__));

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


#define ARRAYSIZE(a) ((int)(sizeof(a) / sizeof(*(a))))

static int recvframe_count() {
    if(sReceivedFrames.head >= sReceivedFrames.tail) {
        return    (sReceivedFrames.head - sReceivedFrames.buffer)
	        - (sReceivedFrames.tail - sReceivedFrames.buffer);
    }
        return    ARRAYSIZE(sReceivedFrames.buffer)
		- (sReceivedFrames.tail - sReceivedFrames.buffer)
	        + (sReceivedFrames.head - sReceivedFrames.buffer);
}

static volatile jn516xPhyFrame* recvframe_pop() {
    if(recvframe_count() == 0) return NULL;

    sReceivedFrames.tail++;
    if((sReceivedFrames.tail - sReceivedFrames.buffer) >= ARRAYSIZE(sReceivedFrames.buffer)) {
        sReceivedFrames.tail = sReceivedFrames.buffer;
    }
    return sReceivedFrames.tail;
}

static volatile jn516xPhyFrame* recvframe_next() {
    if(recvframe_count() == ARRAYSIZE(sReceivedFrames.buffer)-1) return NULL;

    sReceivedFrames.head++;
    if((sReceivedFrames.head - sReceivedFrames.buffer) >= ARRAYSIZE(sReceivedFrames.buffer)) {
        sReceivedFrames.head = sReceivedFrames.buffer;
    }
    return sReceivedFrames.head;
}

static volatile jn516xPhyFrame* recvframe_head() {
    return sReceivedFrames.head;
}

static void dumpBytes(uint8_t* b, size_t len) {

    while(len > 0) {
	size_t dumped = len;
	if(len == 1) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x", 
b[0]);
	}
	else if(len == 2) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x", 
b[0], b[1]);
	}
	else if(len == 3) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x", 
b[0], b[1], b[2]);
	}
	else if(len == 4) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x", 
b[0], b[1], b[2], b[3]);
	}
	else if(len == 5) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x %02x", 
b[0], b[1], b[2], b[3], b[4]);
	}
	else if(len == 6) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x %02x %02x", 
b[0], b[1], b[2], b[3], b[4], b[5]);
	}
	else if(len == 7) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x %02x %02x %02x", 
b[0], b[1], b[2], b[3], b[4], b[5], b[6]);
	}
	else if(len == 8) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x %02x %02x %02x %02x", 
b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
	}
	else if(len == 9) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x %02x %02x %02x %02x     %02x", 
b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8]);
	}
	else if(len == 10) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x %02x %02x %02x %02x     %02x %02x", 
b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9]);
	}
	else if(len == 11) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x %02x %02x %02x %02x     %02x %02x %02x", 
b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10]);
	}
	else if(len == 12) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x %02x %02x %02x %02x     %02x %02x %02x %02x", 
b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11]);
	}
	else if(len == 13) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x %02x %02x %02x %02x     %02x %02x %02x %02x %02x", 
b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12]);
	}
	else if(len == 14) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x %02x %02x %02x %02x     %02x %02x %02x %02x %02x %02x", 
b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13]);
	}
	else if(len == 15) {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x %02x %02x %02x %02x     %02x %02x %02x %02x %02x %02x %02x", 
b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13], b[14]);
	}
	else {
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM,
"dump: %02x %02x %02x %02x %02x %02x %02x %02x     %02x %02x %02x %02x %02x %02x %02x %02x", 
       b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
	dumped = 16;
	}

    len -= dumped;
	b += dumped;
    }
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
    OT_UNUSED_VARIABLE(aInstance);
    tsExtAddr extAddr;
    vMMAC_GetMacAddress(&extAddr);
    uint32_t* peui64 = (uint32_t*)aIeeeEui64;

    peui64[0] = extAddr.u32H;
    peui64[1] = extAddr.u32L;
}
#endif // OPENTHREAD_CONFIG_ENABLE_PLATFORM_EUI64_CUSTOM_SOURCE

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanId)
{
    OT_UNUSED_VARIABLE(aInstance);

    sCurrentPanId = aPanId;
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    memcpy(&sCurrentExtendedAddress, aExtAddress, sizeof(otExtAddress));
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aShortAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    sCurrentShortAddress = aShortAddress;
}

void jn516xRadioInit(void)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
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
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    vMMAC_RadioOff();
    sRadioState = OT_RADIO_STATE_SLEEP;
    sPendingEvents = 0;
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);

    if (sDisabled)
    {
        return OT_RADIO_STATE_DISABLED;
    }


    return sRadioState;
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);

    return !sDisabled;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    otError error;

#if !OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT
    OT_UNUSED_VARIABLE(aInstance);
#else
    sInstance = aInstance;
#endif

    if (sDisabled)
    {
        sDisabled = false;
        error     = OT_ERROR_NONE;
    }
    else
    {
        error = OT_ERROR_INVALID_STATE;
    }

    return error;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    otError error = OT_ERROR_NONE;

    otEXPECT(otPlatRadioIsEnabled(aInstance));
    otEXPECT_ACTION(otPlatRadioGetState(aInstance) == OT_RADIO_STATE_SLEEP || isPendingEventSet(kPendingEventSleep),
                    error = OT_ERROR_INVALID_STATE);

    sDisabled = true;

exit:
    return error;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);
    vMMAC_RadioOff();
    clearPendingEvents();

    sRadioState = OT_RADIO_STATE_SLEEP;
    return OT_ERROR_NONE;
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s %d", __func__, aChannel);
    OT_UNUSED_VARIABLE(aInstance);

    sCurrentChannel = aChannel;
    vMMAC_SetChannelAndPower(sCurrentChannel, sDefaultTxPower);

    vMMAC_StartPhyReceive((tsPhyFrame*)(&recvframe_head()->phy),
		    (E_MMAC_RX_START_NOW | E_MMAC_RX_NO_FCS_ERROR) ); /* means: reject FCS errors */

    clearPendingEvents();

    sRadioState = OT_RADIO_STATE_RECEIVE;
    return 0;
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
dumpBytes(aFrame->mPsdu, aFrame->mLength);
    memcpy(sTransmitPsdu.uPayload.au8Byte, aFrame->mPsdu, aFrame->mLength);
    sTransmitPsdu.u8PayloadLength = aFrame->mLength;

    sCurrentChannel = aFrame->mChannel;
    vMMAC_SetChannelAndPower(sCurrentChannel, sDefaultTxPower);

    setPendingEvent(kPendingEventChannelAccessFailure);
    sRadioState = OT_RADIO_STATE_RECEIVE;

    vMMAC_StartPhyTransmit(&sTransmitPsdu, E_MMAC_TX_START_NOW | E_MMAC_TX_USE_CCA);

    clearPendingEvents();
    otPlatRadioTxStarted(aInstance, aFrame);

    sRadioState = OT_RADIO_STATE_TRANSMIT;
    return OT_ERROR_NONE;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return &sTransmitFrame;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);

    return ED2DBM(u8MMAC_EnergyDetect(sCurrentChannel));
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return OT_RADIO_CAPS_ENERGY_SCAN;
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
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s %d", __func__, aEnable);
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aEnable);
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s %u", __func__, aShortAddress);
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aShortAddress);
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aExtAddress);
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aShortAddress);
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aExtAddress);
    return OT_ERROR_NOT_IMPLEMENTED;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s %u %u", __func__, aScanChannel, aScanDuration);
    sEnergyDetected = ED2DBM(u8MMAC_EnergyDetect(aScanChannel));

    setPendingEvent(kPendingEventEnergyDetected);

    return OT_ERROR_NONE;
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;

    if (aPower == NULL)
    {
        error = OT_ERROR_INVALID_ARGS;
    }
    else
    {
        sDefaultTxPower = i8MMAC_GetTxPowerLevel();
        *aPower = sDefaultTxPower;
    }

    return error;
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s %d", __func__);
    OT_UNUSED_VARIABLE(aInstance);

    sDefaultTxPower = aPower;
    vMMAC_SetChannelAndPower(sCurrentChannel, sDefaultTxPower);

    return OT_ERROR_NONE;
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aThreshold);
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aThreshold);
    return OT_ERROR_NOT_IMPLEMENTED;
}

void jn516xRadioProcess(otInstance *aInstance)
{
    bool isEventPending = false;

    while (recvframe_count() > 0)
    {
        jn516xPhyFrame* recvframe = (jn516xPhyFrame*)recvframe_pop();
        if (recvframe != NULL)
        {
#if OPENTHREAD_CONFIG_DIAG_ENABLE

            if (otPlatDiagModeGet())
            {
                otPlatDiagRadioReceiveDone(aInstance, &recvframe->ot, OT_ERROR_NONE);
            }
            else
#endif
            {
                otPlatRadioReceiveDone(aInstance, &recvframe->ot, OT_ERROR_NONE);
            }
        }
    }

    if (isPendingEventSet(kPendingEventFrameTransmitted))
    {
        resetPendingEvent(kPendingEventFrameTransmitted);

#if OPENTHREAD_CONFIG_DIAG_ENABLE

        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, OT_ERROR_NONE);
        }
        else
#endif
        {
            otRadioFrame *ackPtr = (sAckFrame.mPsdu == NULL) ? NULL : &sAckFrame;
            otPlatRadioTxDone(aInstance, &sTransmitFrame, ackPtr, OT_ERROR_NONE);
        }

        if (sAckFrame.mPsdu != NULL)
        {
            sAckFrame.mPsdu = NULL;
        }
    }

    if (isPendingEventSet(kPendingEventChannelAccessFailure))
    {
        resetPendingEvent(kPendingEventChannelAccessFailure);

#if OPENTHREAD_CONFIG_DIAG_ENABLE

        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, OT_ERROR_CHANNEL_ACCESS_FAILURE);
        }
        else
#endif
        {
            otPlatRadioTxDone(aInstance, &sTransmitFrame, NULL, OT_ERROR_CHANNEL_ACCESS_FAILURE);
        }
    }

    if (isPendingEventSet(kPendingEventInvalidOrNoAck))
    {
        resetPendingEvent(kPendingEventInvalidOrNoAck);

#if OPENTHREAD_CONFIG_DIAG_ENABLE

        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, OT_ERROR_NO_ACK);
        }
        else
#endif
        {
            otPlatRadioTxDone(aInstance, &sTransmitFrame, NULL, OT_ERROR_NO_ACK);
        }
    }

    if (isPendingEventSet(kPendingEventReceiveFailed))
    {
        resetPendingEvent(kPendingEventReceiveFailed);

#if OPENTHREAD_CONFIG_DIAG_ENABLE

        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioReceiveDone(aInstance, NULL, sReceiveError);
        }
        else
#endif
        {
            otPlatRadioReceiveDone(aInstance, NULL, sReceiveError);
        }
    }

    if (isPendingEventSet(kPendingEventEnergyDetected))
    {
        resetPendingEvent(kPendingEventEnergyDetected);

        otPlatRadioEnergyScanDone(aInstance, sEnergyDetected);
    }

    if (isEventPending)
    {
        otSysEventSignalPending();
    }
}

static void jn516x_802154_received_timestamp_raw(volatile jn516xPhyFrame *phyframe, int8_t power, uint8_t lqi, uint32_t time)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
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
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
    OT_UNUSED_VARIABLE(aInstance);

    return JN516X_RECEIVE_SENSITIVITY;
}

static inline int compare_shortaddr(const uint16_t* laddr, const uint16_t* raddr) {
    return memcmp(laddr, raddr, sizeof(uint16_t));
}
static inline int is_broadcast_shortaddr(const uint16_t* addr) {
    static const uint16_t sbc = 0xFFFF;
    return compare_shortaddr(addr, &sbc);
}
static inline int compare_extaddr(const otExtAddress* laddr, const otExtAddress* raddr) {
    return memcmp(laddr, raddr, sizeof(otExtAddress));
}
static inline int is_broadcast_extaddr(const otExtAddress* addr) {
    static const otExtAddress extbc = { .m8 = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } };
    return compare_extaddr(addr, &extbc);
}

static int is_packet_for_us(volatile tsPhyFrame* phy)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);

    volatile uint8_t* buf = phy->uPayload.au8Byte;
    int len =  phy->u8PayloadLength;

otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "sizeof(frame802154_fcf_t) = %d", sizeof(frame802154_fcf_t));

    frame802154_fcf_t* fcf = (frame802154_fcf_t*)&buf[0];
    volatile uint8_t* endbuf = buf + len;

    uint8_t        frame_type = (buf[0] > 0) & 0x7;
    uint8_t panid_compression = (buf[0] > 6) & 0x1;
    uint8_t    dest_addr_mode = (buf[1] > 2) & 0x3;
    uint8_t     frame_version = (buf[1] > 4) & 0x3;
    uint8_t     src_addr_mode = (buf[1] > 6) & 0x3;

    buf += 2;
    int dest_pan_id = 0;
    uint8_t seq = *buf;
    buf += 1;
    if(buf > endbuf) return 0;
    
    if(frame_version == FRAME802154_IEEE802154_2015) {
        /*
         * IEEE 802.15.4-2015
         * Table 7-2, PAN ID Compression value for frame version 0b10
         */
        if((dest_addr_mode == FRAME802154_NOADDR &&
            src_addr_mode  == FRAME802154_NOADDR &&
            panid_compression == 1) ||
           (dest_addr_mode != FRAME802154_NOADDR &&
            src_addr_mode  == FRAME802154_NOADDR &&
            panid_compression == 0) ||
           (dest_addr_mode == FRAME802154_LONGADDRMODE &&
            src_addr_mode  == FRAME802154_LONGADDRMODE &&
            panid_compression == 0) ||
          ((dest_addr_mode == FRAME802154_SHORTADDRMODE &&
            src_addr_mode  != FRAME802154_NOADDR) ||
           (dest_addr_mode != FRAME802154_NOADDR &&
            src_addr_mode  == FRAME802154_SHORTADDRMODE)) ){
            dest_pan_id = 1;
        }
    }
    else {
        /* No PAN ID in ACK */
        if(frame_type != FRAME802154_ACKFRAME) {
            if(dest_addr_mode & 3) {
                dest_pan_id = 1;
            }
        }
    }

otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "dest_pan_id = %d", dest_pan_id);

    if(dest_pan_id) {
        uint16_t pan = *((uint16_t*)buf);
        buf += 2;
        if(buf > endbuf) return 0;

        if(sCurrentPanId != FRAME802154_BROADCASTADDR &&
                     pan != FRAME802154_BROADCASTADDR &&
                     pan != sCurrentPanId) {
            return 0;
        }
    }

    if(dest_addr_mode == FRAME802154_SHORTADDRMODE) {
        int ret = 1;
        if(!is_broadcast_shortaddr((uint16_t*)buf) ) {
            if(compare_shortaddr((uint16_t*)buf, &sCurrentShortAddress) ) {
                ret = 2;
            }
        }
        buf += 2;
        if(buf > endbuf) return 0;
        return ret;
    }
    else if(dest_addr_mode == FRAME802154_LONGADDRMODE) {
        int ret = 1;
        if(!is_broadcast_extaddr((otExtAddress*)buf) ) {
            if(compare_extaddr((otExtAddress*)buf, &sCurrentExtendedAddress) ) {
                ret = 2;
            }
        }
        buf += 8;
        if(buf > endbuf) return 0;
        return ret;
    }

    return 0;
}

static void radio_interrupt_handler(uint32_t mac_event)
{
otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "enter %s", __func__);
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
dumpBytes((uint8_t*)rx_frame_buffer->phy.uPayload.au8Byte, (size_t)rx_frame_buffer->phy.u8PayloadLength);
        uint32_t rx_status = u32MMAC_GetRxErrors();
        if(rx_status != 0) {
            switch (rx_status) {
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
            int packet_for_me = is_packet_for_us(&rx_frame_buffer->phy);
            //if(!sPromiscuous) {
                /* Check RX address */
            //} else {//if(!frame_filtering) {
            //    packet_for_me = 1;
            //}

otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "packet_for_me = %d", packet_for_me);
            if(!packet_for_me) {
                /* Prevent reading */
                rx_frame_buffer->phy.u8PayloadLength = 0;
            } else {
                if(packet_for_me == 2) {
                    //send_ack();
                }
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
