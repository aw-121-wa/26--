/**
 * @file    bsp_linefollower.c
 * @brief   红外传感器驱动模块
 * @details 只有前方红外，用于挡板检测
 */

#include "bsp_linefollower.h"
#include "main.h"

/* ======================== 全局变量定义 ======================== */

uint8_t infrare_open = 0;

/* ======================== 红外读取函数 ======================== */

/**
 * @brief  读取红外传感器值（预留）
 * @details 前方红外通过 Infrared_ahead 宏直接读取
 */
void get_Infrared(void)
{
    /* 前方红外通过宏 Infrared_ahead 直接读取，无需额外处理 */
}
