#ifndef __BSP_LINEFOLLOWER_H
#define __BSP_LINEFOLLOWER_H

#include "sys.h"

/* ======================== 红外传感器引脚定义 ======================== */
/* 只有前方红外，用于挡板检测 */

#define Infrared_ahead  (uint8_t)!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)

/* ======================== 全局变量声明 ======================== */

extern uint8_t infrare_open;

/* ======================== 函数声明 ======================== */

/**
 * @brief  读取红外传感器值（预留）
 */
void get_Infrared(void);

#endif /* __BSP_LINEFOLLOWER_H */
