#ifndef __BSP_LINEFOLLOWER_H
#define __BSP_LINEFOLLOWER_H

#include "sys.h"

/* ======================== 红外传感器引脚定义 ======================== */

#define Infrared_ahead  (uint8_t)!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)
/* 桥面辅助红外（PD9左/PD10右） */
#define Infrared_Left   (uint8_t)!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9)
#define Infrared_Right  (uint8_t)!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)

/* ======================== 全局变量声明 ======================== */

extern uint8_t infrare_open;

/* ======================== 函数声明 ======================== */

/**
 * @brief  读取红外传感器值（预留）
 */
void get_Infrared(void);

#endif /* __BSP_LINEFOLLOWER_H */
