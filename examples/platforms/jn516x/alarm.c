#include <openthread-core-config.h>
#include <openthread/config.h>
#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/time.h>
#include "openthread-system.h"

#include "platform-jn516x.h"

void jn516xAlarmInit(void)
{
}

void jn516xAlarmDeinit(void)
{
}

void jn516xAlarmProcess(otInstance *aInstance)
{
	(void)aInstance;
}

uint32_t otPlatAlarmMilliGetNow(void)
{
	return 0;
}
void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
	(void)aInstance;
	(void)aT0;
	(void)aDt;
}
void otPlatAlarmMilliStop(otInstance *aInstance)
{
	(void)aInstance;
}
uint32_t otPlatAlarmMicroGetNow(void)
{
	return 0;
}
void otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
	(void)aInstance;
	(void)aT0;
	(void)aDt;
}
void otPlatAlarmMicroStop(otInstance *aInstance)
{
	(void)aInstance;
}
#if OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
uint64_t otPlatTimeGet(void)
{
	return 0;
}
uint16_t otPlatTimeGetXtalAccuracy(void)
{
	return 0;
}
#endif // OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
