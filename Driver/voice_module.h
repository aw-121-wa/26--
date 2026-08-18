#ifndef __VOICE_MODULE_H__
#define __VOICE_MODULE_H__

#include "main.h"

#define VOICE_INDEX_PLATFORM_P1      1u
#define VOICE_INDEX_PLATFORM_P2      2u
#define VOICE_INDEX_PLATFORM_P3      3u
#define VOICE_INDEX_PLATFORM_P4      4u
#define VOICE_INDEX_PLATFORM_P5      5u
#define VOICE_INDEX_PLATFORM_P6      6u
#define VOICE_INDEX_PLATFORM_P7      7u
#define VOICE_INDEX_PLATFORM_P8      8u
#define VOICE_INDEX_READY_START      9u
#define VOICE_INDEX_FIVE_MOUNTAINS   10u
#define VOICE_INDEX_FAIL_END         11u
#define VOICE_INDEX_BARRIER_DETECTED 12u
#define VOICE_INDEX_RESERVED_13      13u
#define VOICE_INDEX_RESERVED_14      14u
#define VOICE_INDEX_PLATFORM_DEFAULT VOICE_INDEX_PLATFORM_P1

HAL_StatusTypeDef VoiceModule_PlayIndex(uint16_t index);
HAL_StatusTypeDef VoiceModule_PlayBarrierDetected(void);
HAL_StatusTypeDef VoiceModule_PlayReadyStart(void);
HAL_StatusTypeDef VoiceModule_PlayFiveMountains(void);
HAL_StatusTypeDef VoiceModule_PlayFailEnd(void);

#endif /* __VOICE_MODULE_H__ */
