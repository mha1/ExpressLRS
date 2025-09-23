#pragma once
#if defined(Regulatory_Domain_EU_CE_2400)

#include "POWERMGNT.h"
#include "LQCALC.h"
#include "SX12xxDriverCommon.h"

extern LQCALC<100> LBTSuccessCalc;

void EnableLBT();
void ICACHE_RAM_ATTR SetClearChannelAssessmentTime(void);
uint32_t getRXWaitTime();
SX12XX_Radio_Number_t ICACHE_RAM_ATTR ChannelIsClear(SX12XX_Radio_Number_t radioNumber);
#else
inline void EnableLBT() {}
inline uint32_t getRXWaitTime() { return 0; }
#endif
