#ifndef __PID_H
#define __PID_H

#include "sys.h"

/* ======================== 增量式 PID 对象结构体 ======================== */
/* 用于电机速度控制 */

struct I_pid_obj {
    float output;           /* PID 输出值 */
    int   bias;             /* 当前偏差 */
    int   last_bias;        /* 上次偏差 */
    int   last2_bias;       /* 上上次偏差 */
    float measure;          /* 测量值 */
    float target;           /* 目标值 */
};

/* ======================== 位置式 PID 对象结构体 ======================== */
/* 用于循线、转弯、陀螺仪角度控制 */

struct P_pid_obj {
    float output;               /* PID 输出值 */
    float bias;                 /* 当前偏差 */
    float measure;              /* 测量值 */
    float last_bias;            /* 上次偏差 */
    float integral;             /* 积分累积值 */
    float last_differential;    /* 上次微分值（用于滤波） */
    float target;               /* 目标值 */
};

/* ======================== PID 参数结构体 ======================== */

struct PID_param {
    float kp;                   /* 比例系数 */
    float ki;                   /* 积分系数 */
    float kd;                   /* 微分系数 */
    float differential_filterK; /* 微分滤波系数，取值范围 (0, 1]，越小滤波越强 */
    float outputMin;            /* 输出下限 */
    float outputMax;            /* 输出上限 */
    float actualMax;            /* 实际最大值（用于积分限幅） */
};

/* ======================== 电机 PID 对象 ======================== */

extern struct I_pid_obj motor_L0;       /* 左前电机 */
extern struct I_pid_obj motor_L1;       /* 左后电机 */
extern struct I_pid_obj motor_R0;       /* 右前电机 */
extern struct I_pid_obj motor_R1;       /* 右后电机 */

extern struct PID_param motor_pid_paramL0;  /* 左前电机 PID 参数 */
extern struct PID_param motor_pid_paramL1;  /* 左后电机 PID 参数 */
extern struct PID_param motor_pid_paramR0;  /* 右前电机 PID 参数 */
extern struct PID_param motor_pid_paramR1;  /* 右后电机 PID 参数 */

/* ======================== 循线 PID 对象 ======================== */

extern struct P_pid_obj line_pid_obj;       /* 循线 PID 对象 */
extern struct PID_param line_pid_param;     /* 循线 PID 参数 */
extern struct PID_param lineG_pid_param;    /* 灰度循线 PID 参数 */

/* ======================== 陀螺仪 PID 对象 ======================== */

extern struct P_pid_obj gyroT_pid;          /* 转弯 PID 对象 */
extern struct P_pid_obj gyroG_pid;          /* 平滑陀螺仪 PID 对象 */
extern struct PID_param gyroT_pid_param;    /* 转弯 PID 参数 */
extern struct PID_param gyroG_pid_param;    /* 平滑陀螺仪 PID 参数 */

extern struct P_pid_obj GyroP_pid;          /* 漂移补偿 PID 对象 */
extern struct PID_param GyroP_pid_param;    /* 漂移补偿 PID 参数 */

/* ======================== PID 算法函数 ======================== */

/**
 * @brief  增量式 PID 计算
 * @param  motor 电机 PID 对象指针
 * @param  pid   PID 参数指针
 * @note   适用于电机速度控制，输出增量累加
 */
void incremental_PID(struct I_pid_obj *motor, struct PID_param *pid);

/**
 * @brief  位置式 PID 计算（带微分滤波）
 * @param  obj PID 对象指针
 * @param  pid PID 参数指针
 * @return float PID 输出值
 * @note   适用于循线、转弯、角度控制
 */
float positional_PID(struct P_pid_obj *obj, struct PID_param *pid);

/* ======================== PID 管理函数 ======================== */

/**
 * @brief  PID 参数初始化
 * @details 初始化所有电机、循线、陀螺仪的 PID 参数
 */
void pid_init(void);

/**
 * @brief  清零电机 PID 状态
 */
void motor_pid_clear(void);

/**
 * @brief  PID 参数渐变（线性插值）
 * @param  current 当前参数指针
 * @param  target  目标参数指针
 * @param  step    插值步长 (0.0~1.0)，1.0表示立即切换
 * @details 将 current 向 target 方向移动 step 比例
 */
void pid_param_blend(struct PID_param *current, const struct PID_param *target, float step);

/* ======================== 调试函数 ======================== */

/**
 * @brief  USMART 调试接口
 * @param  val  参数值
 * @param  deno 分母
 * @param  mode 模式
 */
void usmart_pid(uint16_t val, int deno, int mode);

/**
 * @brief  修改目标值（调试用）
 * @param  targetq 目标值
 */
void chage_target(uint16_t targetq);

/**
 * @brief  修改电机 L1 的 Kp 参数（调试用）
 * @param  param 参数值（实际值 = param / 10.0）
 */
void speed_pid_kp(int param);

/**
 * @brief  修改电机 L1 的 Kd 参数（调试用）
 * @param  param 参数值（实际值 = param / 10.0）
 */
void speed_pid_kd(int param);

/**
 * @brief  修改电机 L1 的 Ki 参数（调试用）
 * @param  param 参数值（实际值 = param / 100.0）
 */
void speed_pid_ki(int param);

#endif /* __PID_H */
