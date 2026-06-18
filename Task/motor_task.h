#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"
#include "speed_ctrl.h"     /* struct Motors + motor_all */

/* ======================== 常量定义 ======================== */

#define PI                  3.1415926535f

/* 电机方向 */
#define MOTOR_DIR_FRONT     0
#define MOTOR_DIR_BACK      1

/* 编码器缩放系数 */
#define R0_ENCODER_SCALE    1.08f
#define R1_ENCODER_SCALE    1.68f

/* 距离计算缩放系数 */
#define MOTOR_DISTANCE_SCALE 1.00f

/* ======================== PID 模式枚举 ======================== */

enum PID_Mode {
    is_No = 0,      /* 无模式（刹车） */
    is_Free,        /* 自由模式 */
    is_Line,        /* 循线模式 */
    is_Turn,        /* 转弯模式 */
    is_Gyro,        /* 陀螺仪模式 */
    is_sp,          /* 特殊模式 */
    is_Remote       /* 遥控模式 */
};

/* ======================== 角度控制结构体 ======================== */

struct Angle_Control {
    float AngleT;       /* 转弯目标角度 */
    float AngleG;       /* 陀螺仪目标角度 */
};

extern struct Angle_Control angle;

/* ======================== 速度渐变结构体 ======================== */

struct Gradual {
    float Now;          /* 当前速度 */
    float Target;       /* 目标速度 */
    float Step;         /* 步进值 */
};

/* ======================== 全局变量声明 ======================== */

extern volatile uint8_t PIDMode;    /* 当前 PID 模式 */
extern int QQB_num;                 /* QQ 编号 */
extern uint8_t Nosmall;             /* 小车标志 */
extern int MOTOR_PWM_MAX;           /* 电机 PWM 最大值 */

/* 速度控制相关 */
extern uint8_t change_speed;        /* 速度变化标志 */
extern uint8_t turnflag;            /* 转弯标志 */
extern uint8_t Turn360_Flag;        /* 360度转弯标志 */

/* 速度渐变 */
extern struct Gradual TC_speed;     /* 循线速度渐变 */
extern struct Gradual TG_speed;     /* 陀螺仪速度渐变 */

/* 流水灯/波浪板 */
extern uint8_t DownLiuShui;         /* 流水灯下降标志 */
extern float LiuShuiRate;           /* 流水灯速率 */
extern uint8_t WavePlateLeft_Flag;  /* 左波浪板标志 */
extern uint8_t WavePlateRight_Flag; /* 右波浪板标志 */

/* 红外 */
extern uint8_t infrare_open;        /* 红外开启标志 */

/* ======================== 函数声明 ======================== */

/**
 * @brief  电机任务主函数
 * @param  pvParameters 任务参数（未使用）
 */
void motor_task(void *pvParameters);

/**
 * @brief  PID 模式切换
 * @param  target_mode 目标模式
 */
void pid_mode_switch(uint8_t target_mode);

/**
 * @brief  PID 模式切换（不继承状态）
 * @param  target_mode 目标模式
 */
void pid_mode_switch_no_inherit(uint8_t target_mode);

/**
 * @brief  获取电机编码器速度
 */
void get_motor_speed(void);

/* ======================== 转弯/陀螺仪函数声明 ======================== */
/* 这些函数需要在 turn.c 中实现 */

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

/**
 * @brief  速度渐变计算
 * @param  grad   渐变结构体
 * @param  target 目标速度
 * @param  up     上升步进
 * @param  down   下降步进
 */
void gradual_cal(struct Gradual *grad, float target, float up, float down);

/**
 * @brief  获取当前Z轴角度
 * @return float 当前角度
 */
float getAngleZ(void);

/* ======================== 辅助函数声明 ======================== */

/**
 * @brief  电机PID清零
 */
void motor_pid_clear(void);

#endif /* MOTOR_TASK_H */
