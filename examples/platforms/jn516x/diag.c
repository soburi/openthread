#include <openthread-core-config.h>
#include <openthread/config.h>
#include <openthread/cli.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/toolchain.h>

void otPlatDiagProcess(otInstance *aInstance, int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
	(void)aInstance;
	(void)argc;
	(void)argv;
	(void)aOutput;
	(void)aOutputMaxLen;
}
void otPlatDiagModeSet(bool aMode)
{
	(void)aMode;
}
bool otPlatDiagModeGet()
{
	return 0;
}
void otPlatDiagChannelSet(uint8_t aChannel)
{
	(void)aChannel;
}
void otPlatDiagTxPowerSet(int8_t aTxPower)
{
	(void)aTxPower;
}
void otPlatDiagRadioReceived(otInstance *aInstance, otRadioFrame *aFrame, otError aError)
{
	(void)aInstance;
	(void)aFrame;
	(void)aError;
}
void otPlatDiagAlarmCallback(otInstance *aInstance)
{
	(void)aInstance;
}
