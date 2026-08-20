#ifndef __VOICE_MODULE_H__
#define __VOICE_MODULE_H__

#include "main.h"

/* BY8001-16P audio index mapping:
 *   1  = platform 8
 *   2  = platform 7
 *   3  = platform 6
 *   4  = platform 5
 *   5  = platform 4
 *   6  = platform 3
 *   7  = platform 2
 *   8  = platform 1
 *   9  = ready/start
 */
#define VOICE_INDEX_READY_START      9u
#define VOICE_INDEX_PLATFORM_P1      8u
#define VOICE_INDEX_PLATFORM_P2      7u
#define VOICE_INDEX_PLATFORM_P3      6u
#define VOICE_INDEX_PLATFORM_P4      5u
#define VOICE_INDEX_PLATFORM_P5      4u
#define VOICE_INDEX_PLATFORM_P6      3u
#define VOICE_INDEX_PLATFORM_P7      2u
#define VOICE_INDEX_PLATFORM_P8      1u
#define VOICE_INDEX_PLATFORM_DEFAULT VOICE_INDEX_PLATFORM_P1

HAL_StatusTypeDef VoiceModule_PlayIndex(uint16_t index);
HAL_StatusTypeDef VoiceModule_PlayReadyStart(void);

#endif /* __VOICE_MODULE_H__ */
