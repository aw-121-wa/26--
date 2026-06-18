#include "speed_ctrl.h"

/*
 * 底盘运动状态全局实例。
 * 仅保留数据定义；CarBrake / gradual_cal 等控制函数分别在
 * chassis_api.c 与 turn.c 中实现，速度渐变量 TC_speed/TG_speed 在 motor_task.c 中定义。
 */
volatile struct Motors motor_all = {
    .Lspeed         = 0,
    .Rspeed         = 0,
    .encoder_avg    = 0,
    .GyroG_speedMax = 220,      /* 陀螺仪直行差速上限 */
    .GyroT_speedMax = 200,      /* 原地转弯速度上限 */
    .Cincrement     = 12,       /* 循迹加速步进 */
    .CDOWNincrement = 10,       /* 循迹减速步进 */
    .Gincrement     = 10,       /* 陀螺仪加速步进 */
    .GDOWNincrement = 10,       /* 陀螺仪减速步进 */
};
