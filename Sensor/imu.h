#ifndef IMU_H
#define IMU_H

#include "main.h"

/* ======================== IMU 数据结构体 ======================== */

struct Imu {
    float yaw;              /* 偏航角 (Z 轴) */
    float roll;             /* 横滚角 (X 轴) */
    float pitch;            /* 俯仰角 (Y 轴) */
    float compensateZ;      /* Z 轴补偿值 */
    float compensatePitch;  /* 俯仰角补偿值 */
};

/* ======================== 全局变量声明 ======================== */

extern struct Imu imu;      /* IMU 数据对象 */
extern float basic_p;       /* 基准俯仰角 */
extern float basic_y;       /* 基准偏航角 */

/* ======================== 函数声明 ======================== */

/**
 * @brief  IMU 接收初始化
 * @details 配置 USART3 的 DMA 接收和空闲中断，用于接收陀螺仪数据
 */
void imu_receive_init(void);

/**
 * @brief  重置 Z 轴角度补偿值
 * @param  sensorangle 当前传感器角度
 * @param  referangle  期望的参考角度
 * @note   令 imu.compensateZ = 归一化(referangle - sensorangle)
 */
void mpuZreset(float sensorangle, float referangle);

#endif /* IMU_H */
