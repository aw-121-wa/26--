#ifndef __SPEED_CTRL_H
#define __SPEED_CTRL_H

#include "sys.h"
#include "stdbool.h"

/*
 * 底盘运动状态（全局唯一实例 motor_all）
 * 说明：本结构体只承载“数据”，速度渐变结构体 Gradual 已统一到 motor_task.h，
 *       SPEED 宏统一到 map.h，相关控制函数（gradual_cal/CarBrake）在 turn.c /
 *       chassis_api.c 中实现，避免重复定义。
 */
struct Motors
{
    float Lspeed, Rspeed;       /* 左/右目标速度 */
    float Cspeed;               /* 循迹基准速度 */
    float Gspeed;               /* 陀螺仪直行基准速度 */

    float GyroT_speedMax;       /* 原地转弯最大速度 */
    float GyroG_speedMax;       /* 陀螺仪直行差速最大值 */

    float encoder_avg;          /* 编码器平均读数 */
    float Distance;             /* 累计里程 */

    float Cincrement;           /* 循迹加速步进 */
    float CDOWNincrement;       /* 循迹减速步进 */
    float Gincrement;           /* 陀螺仪加速步进 */
    float GDOWNincrement;       /* 陀螺仪减速步进 */
};

extern volatile struct Motors motor_all;

#endif /* __SPEED_CTRL_H */
