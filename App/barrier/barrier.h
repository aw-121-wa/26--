#ifndef __BARRIER_H
#define __BARRIER_H

#include "sys.h"

/* ======================== 障碍物函数声明 ======================== */

/**
 * @brief  准备函数 - 启动流程
 */
void zhunbei(void);

/**
 * @brief  P2平台处理
 */
void Stage_P2(void);

/**
 * @brief  通用平台处理（P1/P3/P4等）
 */
void Stage(void);

/**
 * @brief  过桥处理
 */
void Barrier_Bridge(void);

/**
 * @brief  楼梯/山地处理
 */
void Barrier_Hill(void);

/**
 * @brief  波浪板处理
 * @param  length 波浪板循线通过距离(cm)
 */
void Barrier_WavedPlate(float length);
uint8_t Barrier_Door(void);

#endif /* __BARRIER_H */
