#include "lsc16_action.h"

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

#define LSC16_FRAME_SIZE       7u
#define LSC16_UART_TIMEOUT_MS  50u

static uint8_t lsc16_tx_frame[LSC16_FRAME_SIZE];

static void lsc16_build_run_frame(uint8_t group, uint16_t times, uint8_t *frame)
{
    frame[0] = 0x55u;
    frame[1] = 0x55u;
    frame[2] = 0x05u;
    frame[3] = 0x06u;
    frame[4] = group;
    frame[5] = (uint8_t)(times & 0xFFu);
    frame[6] = (uint8_t)(times >> 8u);
}

HAL_StatusTypeDef Lsc16_RunActionGroup(uint8_t group, uint16_t times)
{
    if (huart7.gState != HAL_UART_STATE_READY)
        return HAL_BUSY;

    lsc16_build_run_frame(group, times, lsc16_tx_frame);
    return HAL_UART_Transmit_IT(&huart7, lsc16_tx_frame, LSC16_FRAME_SIZE);
}

HAL_StatusTypeDef Lsc16_RunActionGroupBlocking(uint8_t group, uint16_t times,
                                               uint32_t wait_ms)
{
    uint8_t frame[LSC16_FRAME_SIZE];
    HAL_StatusTypeDef status;

    lsc16_build_run_frame(group, times, frame);
    status = HAL_UART_Transmit(&huart7, frame, LSC16_FRAME_SIZE,
                               LSC16_UART_TIMEOUT_MS);
    if (status == HAL_OK && wait_ms > 0u)
        vTaskDelay(pdMS_TO_TICKS(wait_ms));

    return status;
}
