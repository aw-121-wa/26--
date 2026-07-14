#ifndef __CHASSIS_API_H
#define __CHASSIS_API_H

#include "sys.h"

/* ======================== 坡道方向枚举 ======================== */

typedef enum {
    RAMP_ASCEND = 0,    /* 上坡 */
    RAMP_DESCEND = 1    /* 下坡 */
} RampDir_t;

typedef enum {
    CHASSIS_STOP_NONE = 0,      /* 未锁存 */
    CHASSIS_STOP_LINE_LOST,     /* 巡线丢线超时 */
    CHASSIS_STOP_TIPOVER,       /* roll 侧翻 */
    CHASSIS_STOP_YAW_JUMP       /* yaw 短时累计突变 */
} Chassis_StopReason_t;

/* ======================== 坡道控制函数 ======================== */

/**
 * @brief  坡道阻塞控制（三阶段状态机）
 * @param  dir             方向：RAMP_ASCEND=上坡, RAMP_DESCEND=下坡
 * @param  init_speed      初始速度
 * @param  angle           陀螺仪目标角度
 * @param  thresh1         阶段1 pitch 阈值
 * @param  speed1          阶段1 速度
 * @param  thresh2         阶段2 pitch 阈值
 * @param  speed2          阶段2 速度
 * @param  done_thresh     完成 pitch 阈值
 * @param  GrayCorrectAngle 灰度修正角度（0=不修正）
 */
void RampCtrl_Blocking(RampDir_t dir, float init_speed, float angle,
                       float thresh1, float speed1,
                       float thresh2, float speed2,
                       float done_thresh, float GrayCorrectAngle);

/* ======================== 底盘控制函数 ======================== */

/**
 * @brief  设置工作模式（等价于 pid_mode_switch，提供统一 API）
 */
void Chassis_SetMode(uint8_t mode);
void Chassis_SetLineMode(void);

/**
 * @brief  设置PID模式和速度
 * @param  mode    PID模式
 * @param  lspeed  左轮速度
 * @param  rspeed  右轮速度
 * @param  angle   陀螺仪角度（仅is_Gyro模式有效）
 */
void Chassis_MotorControl(uint8_t mode, float lspeed, float rspeed, float angle);

/**
 * @brief  设置目标速度
 * @param  speed 目标速度
 */
void Chassis_SetTargetSpeed(float speed);

/**
 * @brief  设置陀螺仪角度目标
 * @param  angle 目标角度
 */
void Chassis_SetGyroAngle_Go(float angle);

/**
 * @brief  清零里程
 */
void Chassis_ClearMileage(void);

/**
 * @brief  获取里程
 * @return float 当前里程(cm)
 */
float Chassis_GetMileage(void);

/**
 * @brief  刹车
 */
void CarBrake(void);

/**
 * @brief  行驶指定距离（阻塞）
 * @param  mode     PID模式
 * @param  distance 目标距离(cm)
 * @param  speed    速度
 * @param  angle    陀螺仪角度
 */
void Chassis_DriveDistance_Blocking(uint8_t mode, float distance, float speed, float angle);

/**
 * @brief  原地转弯（阻塞）
 * @param  target_angle 目标角度
 * @param  current_angle 当前角度
 */
void Chassis_Turn_By_StopGyro_Blocking(float target_angle, float current_angle);
void Chassis_Turn_180_Blocking(void);

/* ======================== 辅助函数 ======================== */

/**
 * @brief  陀螺仪稳定校准
 * @param  samples    采样次数
 * @param  angle_out  输出角度
 */
void GyroStableReset(uint8_t samples, float *angle_out);

/**
 * @brief  检测是否到达坡道
 * @param  pitch_thresh pitch角阈值
 * @return uint8_t      1=检测到坡道, 0=未检测到
 */
uint8_t Stage_DetectedRamp(float pitch_thresh);

/* ======================== 游龙防护 / 丢线保护 ======================== */

void Chassis_EnableAntiSnake(void);
void Chassis_DisableAntiSnake(void);
void Chassis_EnableLineLostProtection(void);
void Chassis_DisableLineLostProtection(void);
void Chassis_EnableRollProtection(void);
void Chassis_DisableRollProtection(void);
void Chassis_EnableYawJumpProtection(void);
void Chassis_DisableYawJumpProtection(void);
/* 强制停车会锁存原因；普通 CarBrake 不锁存。 */
void Chassis_ForceStop(Chassis_StopReason_t reason);
uint8_t Chassis_IsStopLocked(void);
Chassis_StopReason_t Chassis_GetStopReason(void);
void Chassis_ClearStopLock(void);
uint8_t Chassis_IsTipoverLocked(void);
void Chassis_ClearTipoverLock(void);

/**
 * @brief  底盘 5ms 周期更新（由 motor_task 调用）
 * @details 执行强制停车、侧翻、yaw突变、游龙和丢线保护
 */
void Chassis_Periodic_Update_5ms(void);

#endif /* __CHASSIS_API_H */
