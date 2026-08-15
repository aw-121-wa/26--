#ifndef __VOICE_MODULE_H__
#define __VOICE_MODULE_H__

#include "main.h"

/* BY8001-16P 音频序号映射（实测确认）：
 *   1  = 丢线/侧翻（失败）
 *   2  = 准备出发（起点挡板检测后播放）
 *   3~9 = 二号~八号平台
 *   10 = 一号平台
 */
#define VOICE_INDEX_FAIL_END         1u
#define VOICE_INDEX_READY_START      2u
#define VOICE_INDEX_PLATFORM_P2      3u
#define VOICE_INDEX_PLATFORM_P3      4u
#define VOICE_INDEX_PLATFORM_P4      5u
#define VOICE_INDEX_PLATFORM_P5      6u
#define VOICE_INDEX_PLATFORM_P6      7u
#define VOICE_INDEX_PLATFORM_P7      8u
#define VOICE_INDEX_PLATFORM_P8      9u
#define VOICE_INDEX_PLATFORM_P1      10u
#define VOICE_INDEX_PLATFORM_DEFAULT VOICE_INDEX_PLATFORM_P1

HAL_StatusTypeDef VoiceModule_PlayIndex(uint16_t index);
HAL_StatusTypeDef VoiceModule_PlayReadyStart(void);
HAL_StatusTypeDef VoiceModule_PlayFailEnd(void);

#endif /* __VOICE_MODULE_H__ */
