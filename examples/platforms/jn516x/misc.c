#include <openthread-core-config.h>
#include <openthread/config.h>
#include <openthread/platform/misc.h>

#include "platform-jn516x.h"

//static uint32_t sResetReason;

bool gPlatformPseudoResetWasRequested;

void jn516xMiscInit(void)
{
}

void jn516xMiscDeinit(void)
{
}

void otPlatReset(otInstance *aInstance)
{
	(void)aInstance;
}
otPlatResetReason otPlatGetResetReason(otInstance *aInstance)
{
	(void)aInstance;
	return 0;
}
void otPlatWakeHost(void)
{
}
