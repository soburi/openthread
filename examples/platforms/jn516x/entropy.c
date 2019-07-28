#include <openthread/platform/entropy.h>
#include <openthread-core-config.h>
#include <openthread/config.h>
#include <utils/code_utils.h>

#include "platform-jn516x.h"

void jn516xRandomInit(void)
{
}

void jn516xRandomDeinit(void)
{
}

otError otPlatEntropyGet(uint8_t *aOutput, uint16_t aOutputLength)
{
	(void)aOutput;
	(void)aOutputLength;
	return 0;
}
