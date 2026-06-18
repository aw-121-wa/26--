#ifndef __TURN_H
#define __TURN_H

#include "sys.h"

/* ======================== 转弯函数声明 ======================== */

/**
 * @brief  获取当前Z轴角度
 * @return float 当前yaw角度
 */
float getAngleZ(void);

/**
 * @brief  转弯到指定角度
 * @param  target 目标角度
 * @return uint8_t 1=完成, 0=进行中
 */
uint8_t Turn_Angle(float target);

/**
 * @brief  平台转弯到指定角度
 * @param  target 目标角度
 * @return uint8_t 1=完成, 0=进行中
 */
uint8_t Stage_turn_Angle(float target);

/**
 * @brief  陀螺仪直线行驶
 * @param  target_angle 目标角度
 * @param  speed        速度
 */
void runWithAngle(float target_angle, float speed);

/**
 * @brief  360度转弯步骤
 */
void Turn360Step(void);

#endif /* __TURN_H */
