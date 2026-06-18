#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "tim.h"

/* ======================== 常量定义 ======================== */

#define ENCODER_FORWARD     1       /* 正转方向 */
#define ENCODER_BACKWARD    -1      /* 反转方向 */
#define ENCODER_MAX_PULSE   3000    /* 最大脉冲数 */

/*
 * 电机参数：
 * - 编码器线数：13
 * - 减速比：144
 * - 轮子直径：104mm
 */

/* ======================== 全局变量声明 ======================== */

extern short Speed[4];              /* 四个电机的编码器速度 */
extern uint8_t dog[4];              /* 电机看门狗（检测电机是否正常） */

/* ======================== 函数声明 ======================== */

/**
 * @brief  编码器初始化
 * @details 启动四个电机的编码器定时器（TIM1/2/3/5）和辅助定时器（TIM11）
 */
void Encoder_init(void);

/**
 * @brief  清零编码器计数和里程
 * @details 清除四个定时器的计数值和累计里程
 */
void encoder_clear(void);

#endif /* ENCODER_H */
