#include "voice_module.h"
#include "usart.h"

#define VOICE_MODULE_UART            huart2
#define VOICE_MODULE_TX_TIMEOUT_MS   50u

/* BY8001-16P 串口控制协议（9600 8N1，3.3V TTL）
 * 帧格式：0x7E LEN CMD PARAM... CHK 0xEF
 *   - LEN  = 操作码 + 参数个数 = 0x05（0x41 + 曲目高/低两个字节）
 *   - CHK  = LEN ^ CMD ^ 所有参数字节（异或校验）
 * 例：播放曲目 1 -> 7E 05 41 00 01 45 EF
 *   校验 = 0x05 ^ 0x41 ^ 0x00 ^ 0x01 = 0x45
 */
#define BY8001_HEADER        0x7Eu
#define BY8001_END           0xEFu
#define BY8001_CMD_PLAY      0x41u
#define BY8001_PLAY_LEN      0x05u

static uint8_t voice_module_checksum(uint8_t length, const uint8_t *cmd)
{
    uint8_t ck = length;
    uint8_t i;

    /* cmd 指向 LEN 之后的字节序列（操作码 + 参数），共 length-1 个字节 */
    for (i = 0u; i < (uint8_t)(length - 1u); i++)
        ck ^= cmd[i];

    return ck;
}

HAL_StatusTypeDef VoiceModule_PlayIndex(uint16_t index)
{
    uint8_t frame[7];
    uint8_t play[3];

    if (index == 0u)
        return HAL_ERROR;

    play[0] = BY8001_CMD_PLAY;
    play[1] = (uint8_t)(index >> 8);
    play[2] = (uint8_t)(index & 0xFFu);

    frame[0] = BY8001_HEADER;
    frame[1] = BY8001_PLAY_LEN;
    frame[2] = play[0];
    frame[3] = play[1];
    frame[4] = play[2];
    frame[5] = voice_module_checksum(BY8001_PLAY_LEN, play);
    frame[6] = BY8001_END;

    return HAL_UART_Transmit(&VOICE_MODULE_UART, frame, sizeof(frame),
                             VOICE_MODULE_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef VoiceModule_PlayReadyStart(void)
{
    return VoiceModule_PlayIndex(VOICE_INDEX_READY_START);
}

HAL_StatusTypeDef VoiceModule_PlayFailEnd(void)
{
    return VoiceModule_PlayIndex(VOICE_INDEX_FAIL_END);
}
