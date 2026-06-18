#ifndef __SCANER_H
#define __SCANER_H

#include "sys.h"
#include "pid.h"

/* ======================== 常量定义 ======================== */

#define LAMP_MAX        16      /* 循迹灯最大数量 */
#define LAMP_HALF       8       /* 循迹灯半数 */

/* ======================== 循迹器数据结构体 ======================== */

typedef struct scaner {
    uint16_t detail;        /* 二进制灯数据（每位对应一个灯） */
    float    error;         /* 循迹误差值 */
    u8       ledNum;        /* 亮灯数量 */
    u8       lineNum;       /* 引导线数量 */
} SCANER;

/* ======================== 循迹器配置结构体 ======================== */

struct Scaner_Set {
    float   CatchsensorNum; /* 目标位置（中心点） */
    int8_t  EdgeIgnore;     /* 边缘忽略灯数（左右各忽略 x 个） */
};

/* ======================== 全局变量声明 ======================== */

extern volatile SCANER Cross_Scaner;        /* 节点间循迹器数据 */
extern volatile struct Scaner_Set scaner_set; /* 循迹器配置 */
extern volatile uint8_t LEFT_RIGHT_LINE;    /* 左右循线模式标志 */

extern float Fspeed;                        /* PID 运算后的循迹速度 */
extern volatile SCANER Scaner;              /* 主循迹器数据 */

/* 权重数组 */
extern float line_weight[16];               /* 当前使用的权重 */
extern const float line_weight_default[16]; /* 默认权重 */
extern const float line_QQBweight_default[16]; /* QQ 模式权重 */
extern const float lineG_weight_default[16];   /* 灰度模式权重 */

/* ======================== 循迹控制函数 ======================== */

/**
 * @brief  循线控制主函数
 * @param  speed 基础速度
 * @details 根据循迹误差计算左右电机速度差，实现循线功能
 */
void Go_Line(float speed);

/**
 * @brief  获取循线误差值
 * @details 读取传感器数据并进行模式处理
 */
uint8_t getline_error(void);

/**
 * @brief  节点间临时循迹值获取
 */
void Cross_getline(void);

/* ======================== 传感器数据处理函数 ======================== */

/**
 * @brief  获取各灯值（读取 GPIO）
 */
void get_detail(void);

/**
 * @brief  配置 16 路循迹灯输入引脚（全部 INPUT/NOPULL）
 */
void scaner_gpio_init(void);

/**
 * @brief  循迹器初始化（从 line_weight_default 复制权重到 line_weight）
 */
void scaner_init(void);

/**
 * @brief  循线扫描（包含各种模式处理）
 * @param  scaner      循迹器数据指针
 * @param  sensorNum   传感器数量
 * @param  edge_ignore 边缘忽略数量
 * @return uint8_t     0 表示成功
 */
uint8_t Line_Scan(volatile SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore);

/**
 * @brief  打印 u16 变量的二进制值
 * @param  data 要打印的数据
 */
void printf_byte(uint16_t data);

/* ======================== 循迹滤波函数 ======================== */

/**
 * @brief  循迹中心值和位置计算
 * @param  scaner       循迹器数据指针
 * @param  edge_ignore  边缘忽略数量
 * @param  SensorNum    传感器数量
 * @param  Error        误差输出指针
 * @param  LED_Num_Temp 亮灯数输出指针
 * @return float        位置值，负数表示错误
 */
float value_calculation(volatile SCANER *scaner, int8_t edge_ignore,
                        unsigned char SensorNum, float *Error, u8 *LED_Num_Temp);

/**
 * @brief  更新循迹值数组（递推平均滤波法）
 * @param  error_kind 错误类型：0=无错, 1=精检验错误, 2=粗略检测错误
 * @param  pos        位置值
 * @param  error      误差值
 */
void Update_line_data(uint8_t error_kind, float pos, float error);

/**
 * @brief  位置检测（判断与上次正确值是否相近）
 * @param  pos 当前位置
 * @return uint8_t 1=正确, 0=错误
 */
uint8_t pos_detect(float pos);

/**
 * @brief  获取滤波后的循迹误差值
 * @return float 循迹误差值
 */
float Get_scaner_error(void);

/**
 * @brief  粗略检测循迹值是否可用
 * @param  LED_Num  亮灯数量
 * @param  Line_Num 引导线数量
 * @return uint8_t  1=不可用, 0=可用
 */
uint8_t error_detect_one(u8 LED_Num, u8 Line_Num);

#endif /* __SCANER_H */
