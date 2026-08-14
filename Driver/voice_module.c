#include "voice_module.h"
#include "usart.h"

#define VOICE_MODULE_UART            huart1
#define VOICE_MODULE_TX_TIMEOUT_MS   50u

HAL_StatusTypeDef VoiceModule_PlayIndex(uint16_t index)
{
    uint8_t frame[6];

    if (index == 0u)
        return HAL_ERROR;

    frame[0] = 0x7Eu;
    frame[1] = 0x04u;
    frame[2] = 0x41u;
    frame[3] = (uint8_t)(index >> 8);
    frame[4] = (uint8_t)(index & 0xFFu);
    frame[5] = 0xEFu;

    return HAL_UART_Transmit(&VOICE_MODULE_UART, frame, sizeof(frame),
                             VOICE_MODULE_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef VoiceModule_PlayBarrierDetected(void)
{
    return VoiceModule_PlayIndex(VOICE_INDEX_BARRIER_DETECTED);
}

HAL_StatusTypeDef VoiceModule_PlayReadyStart(void)
{
    return VoiceModule_PlayIndex(VOICE_INDEX_READY_START);
}

HAL_StatusTypeDef VoiceModule_PlayFiveMountains(void)
{
    return VoiceModule_PlayIndex(VOICE_INDEX_FIVE_MOUNTAINS);
}

HAL_StatusTypeDef VoiceModule_PlayFailEnd(void)
{
    return VoiceModule_PlayIndex(VOICE_INDEX_FAIL_END);
}
