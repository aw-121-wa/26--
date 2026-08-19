/**
 * @file    scaner.c
 * @brief   循迹传感器驱动模块
 * @details 包含循迹数据读取、误差计算、滤波处理等功能
 */

#include "scaner.h"
#include "map.h"
#include "../App/chassis/chassis_api.h"
#include "math.h"
#include "stdio.h"
#include "pid.h"
#include "motor_task.h"
#include "bsp_linefollower.h"
#include "string.h"

/* ======================== 常量定义 ======================== */

#define LINE_SPEED_MAX      200     /* 循线最大速度补偿 */
#define LINEG_SPEED_MAX     200     /* 灰度循线最大速度补偿 */
#define SPEED_COMPENSATE    5       /* 速度补偿系数 */
#define BLACK               0       /* 循黑线模式 */
#define WHITE               1       /* 循白线模式 */
#define LINE_DEBUG          0       /* 调试模式开关 */
#define MAX_LED             4       /* 最大有效亮灯数 */
#define POS_MAX_ERROR       1.5f    /* 位置检测最大误差阈值 */
#define NEIGHBOR_RANGE      1       /* 邻近检测范围 */

/* ======================== 全局变量定义 ======================== */

volatile uint8_t LEFT_RIGHT_LINE = 0;   /* 左右循线模式标志 */
float Fspeed;                           /* PID 运算后的循迹速度 */

/* 默认权重数组（从左到右，正负表示方向） */
const float line_weight_default[16] = {
    3.0f, 2.4f, 1.8f, 1.3f, 0.9f, 0.6f, 0.4f, 0.2f,
   -0.2f,-0.4f,-0.6f,-0.9f,-1.3f,-1.8f,-2.4f,-3.0f
};

/* QQ 模式权重 */
const float line_QQBweight_default[16] = {
    3.4f, 2.8f, 2.2f, 2.2f, 1.7f, 1.3f, 0.8f, 0.6f,
   -0.6f,-0.8f,-1.0f,-1.3f,-1.7f,-2.2f,-2.8f,-3.4f
};

/* 灰度模式权重 */
const float lineG_weight_default[16] = {
   -3.0f,-2.4f,-1.8f,-1.3f,-0.9f,-0.6f,-0.4f,-0.2f,
    0.2f, 0.4f, 0.6f, 0.9f, 1.3f, 1.8f, 2.4f, 3.0f
};

float line_weight[16];                      /* 当前使用的权重数组（由 scaner_init 从 default 复制） */
volatile struct Scaner_Set scaner_set = {0, 0}; /* 循迹器配置 */
volatile SCANER Scaner;                     /* 主循迹器数据 */
volatile SCANER Cross_Scaner;               /* 节点间循迹器数据 */

/* ======================== 私有函数声明 ======================== */

/**
 * @brief  读取 16 路 GPIO 状态（RF 模式）
 * @return uint16_t 二进制灯数据
 */
static uint16_t read_gpio_16ch(void)
{
    uint16_t data = 0xFFFF;
    data ^= ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)) << 15);
    data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))  << 14);
    data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))  << 13);
    data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))  << 12);
    data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))  << 11);
    data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))  << 10);
    data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7))  << 9);
    data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)) << 8);
    data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5))  << 7);
    data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4))  << 6);
    data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3))  << 5);
    data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1))  << 4);
    data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6))  << 3);
    data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))  << 2);
    data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2))  << 1);
    data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)) << 0);
    return ~data;
}

/* ======================== 私有变量 ======================== */

/* 五帧滤波始终开启（Travel_China next 恢复）；isFilter 常量已并入调用链，不再单独使用。 */

/* 循迹数据历史（用于滤波） */
struct Line_data {
    volatile float pos;         /* 灯的中心位置 */
    volatile float error;       /* 误差 */
    volatile uint8_t truth;     /* 数据有效性：0=正确, 1=全错误, 2=位置错误 */
};

static struct Line_data line_data[5] = {
    {0.0f, 0.0f, 1},
    {0.0f, 0.0f, 1},
    {0.0f, 0.0f, 1},
    {0.0f, 0.0f, 1},
    {0.0f, 0.0f, 1}
};

/* 错误类型枚举 */
enum Error_Type {
    NO_ERROR = 0,       /* 无错误 */
    ALL_ERROR = 1,      /* 全部错误 */
    POS_ERROR = 2       /* 位置错误 */
};

/* ======================== 循迹控制函数 ======================== */

/**
 * @brief  循线控制主函数（用五帧滤波后的误差驱动位置式 PID）
 * @param  speed 基础速度
 */
void Go_Line(float speed)
{
    line_pid_obj.measure = Get_scaner_error();   /* 滤波后误差（由 Line_ControlSampleUpdate 维护） */
    line_pid_obj.target = scaner_set.CatchsensorNum;

    /* ========== PID ========== */
    Fspeed = positional_PID(&line_pid_obj, &line_pid_param);

    if (Fspeed >= LINE_SPEED_MAX)
        Fspeed = LINE_SPEED_MAX;
    else if (Fspeed <= -LINE_SPEED_MAX)
        Fspeed = -LINE_SPEED_MAX;

    Fspeed *= fabsf(speed) / 40;

    if (Fspeed > fabsf(speed))
        Fspeed = fabsf(speed);
    else if (Fspeed < -fabsf(speed))
        Fspeed = -fabsf(speed);

    if (speed < 0.0f)
        Fspeed = -Fspeed;

    motor_all.Lspeed = speed - Fspeed;
    motor_all.Rspeed = speed + Fspeed;
}

void Line_SetTrackModeBumpless(uint8_t mode)
{
    taskENTER_CRITICAL();

    LEFT_RIGHT_LINE = mode;

    /* 模式语义改变 → 重置五帧滤波历史（全部 ALL_ERROR，不伪造 NO_ERROR） */
    Line_FilterReset();

    /* 采一帧新模式数据，让滤波历史进入新状态（本函数不被其递归调用） */
    Line_ControlSampleUpdate();

    /* 用当前 filtered error 同步位置式 PID 历史，避免模式切换微分冲击 */
    line_pid_obj.target = scaner_set.CatchsensorNum;
    line_pid_obj.measure = Get_scaner_error();
    line_pid_obj.bias = line_pid_obj.target - line_pid_obj.measure;
    line_pid_obj.last_bias = line_pid_obj.bias;
    line_pid_obj.integral = 0.0f;
    line_pid_obj.last_differential = 0.0f;
    line_pid_obj.output = line_pid_param.kp * line_pid_obj.bias;

    taskEXIT_CRITICAL();
}

/**
 * @brief  获取模式处理后的循迹值
 * @return uint8_t 0 表示成功
 */
uint8_t getline_error(void)
{
    get_detail();
    Line_Scan(&Scaner, LAMP_MAX, scaner_set.EdgeIgnore);
    return 0;
}

/**
 * @brief  节点间临时循迹值获取
 */
void Cross_getline(void)
{
    u8 linenum = 0;
    u8 lednum = 0;

    Cross_Scaner.detail = read_gpio_16ch();

    /* 统计亮灯数和引导线数（安全：i+1 越界即判定线段结束） */
    for (uint8_t i = 0; i < 16; i++)
    {
        if (Cross_Scaner.detail & (1u << i))
        {
            lednum++;
            if ((i + 1 >= 16) || ((Cross_Scaner.detail & (1u << (i + 1))) == 0u))
                linenum++;  /* 检测到从 1 变为 0 认为一条线 */
        }
    }

    Cross_Scaner.lineNum = linenum;
    Cross_Scaner.ledNum = lednum;
}

/* ======================== 滤波历史重置 / 控制采样 ======================== */

/**
 * @brief  重置五帧滤波历史（全部置 ALL_ERROR，不伪造为 NO_ERROR）
 */
void Line_FilterReset(void)
{
    for (int i = 0; i < 5; i++)
    {
        line_data[i].truth = ALL_ERROR;
        line_data[i].pos = 0.0f;
        line_data[i].error = 0.0f;
    }
}

/**
 * @brief  控制滤波采样（仅 motor_task 调用，line_data 唯一写入者）
 * @details 读 GPIO → 更新 Scaner → 粗筛 → value_calculation → pos_detect
 *          → Update_line_data。每个5ms周期只有一次控制滤波样本。
 */
void Line_ControlSampleUpdate(void)
{
    u8 led_tmp = 0;
    float error = 0.0f;
    float pos;
    uint8_t kind;

    /* 原始读取并更新 Scaner.detail/ledNum/lineNum/error（瞬时） */
    getline_error();

    /* 粗筛：不可用 → ALL_ERROR */
    if (error_detect_one(Scaner.ledNum, Scaner.lineNum))
    {
        Update_line_data(ALL_ERROR, 0.0f, 0.0f);
        return;
    }

    /* 候选位置/误差 */
    led_tmp = 0;
    error = 0.0f;
    pos = value_calculation(&Scaner, scaner_set.EdgeIgnore, LAMP_MAX, &error, &led_tmp);
    if (pos < 0.0f)
    {
        Update_line_data(ALL_ERROR, 0.0f, 0.0f);
        return;
    }

    /* 位置连续性检测（与最近一帧 NO_ERROR 比较） */
    kind = pos_detect(pos) ? NO_ERROR : POS_ERROR;

    /* 写入五帧滤波历史（唯一写入者） */
    Update_line_data(kind, pos, error);
}

/* ======================== 传感器数据读取 ======================== */

/**
 * @brief  读取 16 路循迹灯状态（直接读 GPIO）
 * @note   本工程循线固定使用 GPIO 直读，不使用 I2C / 灰度模式
 */
void get_detail(void)
{
    Scaner.detail = read_gpio_16ch();
}

/**
 * @brief  循迹器初始化：从 line_weight_default 复制权重到 line_weight
 * @note   必须在循线前调用（travel 中由 Gray_Init 完成，本工程无灰度模块，故独立出来）
 */
void scaner_init(void)
{
    memcpy(line_weight, line_weight_default, sizeof(line_weight));
    Line_FilterReset();   /* 上电：清空五帧滤波历史（ALL_ERROR） */
}

/**
 * @brief  配置 16 路循迹灯输入引脚
 * @details 与 travel 板一致：全部 INPUT / NOPULL。
 *          放在驱动里自带配置，避免 CubeMX .ioc 漏配某些脚（实测 PE14/PC2/PC3/PC14/PC15 漏配）。
 *          对 CubeMX 已配好的脚重复 Init 也无副作用。
 *          引脚映射（bit15→bit0）：
 *          E14 | B8 B7 B6 B5 B4 | D7 | C15 | D5 D4 D3 D1 D0 | C3 C2 | C14
 */
void scaner_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOD, &gpio);
}

/* ======================== 循线扫描处理 ======================== */

/**
 * @brief  左循线处理
 * @param  scaner      循迹器数据指针
 * @param  sensorNum   传感器数量
 * @param  edge_ignore 边缘忽略数量
 * @param  error       误差累加指针
 * @param  lednum_tmp  亮灯数指针
 */
static void scan_left_line(volatile SCANER *scaner, unsigned char sensorNum,
                           int8_t edge_ignore, float *error, int8_t *lednum_tmp)
{
    for (uint8_t i = edge_ignore; i < sensorNum - edge_ignore; i++)
    {
        int bit = (int)(sensorNum - 1 - i);   /* 当前 bit；先算 int，避免负移位 */
        *lednum_tmp += (scaner->detail >> bit) & 0x01;
        *error += ((scaner->detail >> bit) & 0x01) * line_weight[i];

        /* 白线且下一个灯不是白 → 线段结束退出 */
        if ((scaner->detail >> bit) & 0x01)
        {
            if ((bit - 1) < (int)edge_ignore)
                break;                          /* 到边缘边界：视为线段结束 */
            if ((scaner->detail & (1u << (bit - 1))) == 0u)
                break;
        }
    }
}

/**
 * @brief  右循线处理
 * @param  scaner      循迹器数据指针
 * @param  sensorNum   传感器数量
 * @param  edge_ignore 边缘忽略数量
 * @param  error       误差累加指针
 * @param  lednum_tmp  亮灯数指针
 */
static void scan_right_line(volatile SCANER *scaner, unsigned char sensorNum,
                            int8_t edge_ignore, float *error, int8_t *lednum_tmp)
{
    for (int8_t i = sensorNum - 1 - edge_ignore; i >= edge_ignore; i--)
    {
        *lednum_tmp += (scaner->detail >> i) & 0x01;
        *error += ((scaner->detail >> i) & 0x01) * line_weight[sensorNum - i - 1];

        if ((scaner->detail >> i) & 0x01)
        {
            if ((i - 1) < (int)edge_ignore)     /* 到边缘边界：线段结束 */
                break;
            if ((scaner->detail & (1u << (i - 1))) == 0u)
                break;
        }
    }
}

/**
 * @brief  流水循线处理（居中选择最近的线）
 * @param  scaner      循迹器数据指针
 * @param  sensorNum   传感器数量
 * @param  edge_ignore 边缘忽略数量
 * @param  error       误差累加指针
 * @param  lednum_tmp  亮灯数指针
 */
static void scan_center_line(volatile SCANER *scaner, unsigned char sensorNum,
                             int8_t edge_ignore, float *error, int8_t *lednum_tmp)
{
    float location_temp = 0.0f;   /* 用 float，避免 7.5 被截成 7 */
    float length_temp = 0.0f;
    float location_best = 0.0f;
    int last_line_location = 0;
    int length = 0;

    /* 找到最靠近中心的线段 */
    for (uint8_t i = edge_ignore; i < sensorNum - edge_ignore; i++)
    {
        if ((scaner->detail & (1u << i)) != 0u)
        {
            location_temp += (float)i;
            length_temp += 1.0f;

            /* 线段结束：i+1 越界 或下一 bit 灭（显式括号，避免运算符优先级） */
            if ((i + 1 >= sensorNum - edge_ignore) ||
                ((scaner->detail & (1u << (i + 1))) == 0u))
            {
                location_temp /= length_temp;

                /* 选择更靠近中心的线段 */
                if (fabsf(location_temp - ((float)((sensorNum - 1)) / 2)) <
                    fabsf(location_best - ((float)((sensorNum - 1)) / 2)))
                {
                    location_best = location_temp;
                    last_line_location = (int)i;
                    length = (int)length_temp;
                }
                location_temp = 0.0f;
                length_temp = 0.0f;
            }
        }
    }

    /* 无有效线：不产生误差/亮灯（避免 unsigned 下溢计算） */
    if (length == 0)
        return;

    /* 计算选中线段的误差 */
    for (int i = last_line_location - length + 1; i <= last_line_location; i++)
    {
        *lednum_tmp += (scaner->detail >> i) & 0x01;
        *error += ((scaner->detail >> i) & 0x01) * line_weight[sensorNum - i - 1];
    }
}

/**
 * @brief  流水灯模式处理（双线取最近）
 * @param  scaner    循迹器数据指针
 * @param  sensorNum 传感器数量
 * @return float     误差值
 */
static float scan_liushui_line(volatile SCANER *scaner, unsigned char sensorNum)
{
    uint8_t flag = 0;
    float error1 = 0, error2 = 0;
    uint8_t lednum1 = 0, lednum2 = 0;

    for (uint8_t i = 0; i < sensorNum; i++)
    {
        if (flag == 0)
        {
            /* 右边线 */
            error1 += ((scaner->detail >> i) & 0x01) * line_weight[sensorNum - i - 1];
            lednum1 += (scaner->detail >> i) & 0x01;
        }
        else if (flag == 1)
        {
            /* 左边线 */
            error2 += ((scaner->detail >> i) & 0x01) * line_weight[sensorNum - i - 1];
            lednum2 += (scaner->detail >> i) & 0x01;
        }

        if ((scaner->detail >> i) & 0x01)
        {
            if ((i + 1 >= sensorNum) || ((scaner->detail & (1u << (i + 1))) == 0u))
                flag = 1;   /* 检测到线结束，切换到左边线 */
        }
    }

    /* 计算平均误差 */
    if (error1 != 0)
        error1 /= lednum1;
    else
        error1 = 0;

    if (lednum2 == 0)
        error2 = 0;
    else
        error2 /= lednum2;

    /* 选择误差更小的线 */
    if (scaner->ledNum >= 6)
        return 0;
    else if (fabs(error2) > fabs(error1) || lednum2 == 0)
        return error1;
    else
        return error2;
}

/**
 * @brief  循线扫描主函数
 * @param  scaner      循迹器数据指针
 * @param  sensorNum   传感器数量
 * @param  edge_ignore 边缘忽略数量
 * @return uint8_t     0 表示成功
 */
uint8_t Line_Scan(volatile SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore)
{
    float error = 0;
    u8 linenum = 0;
    u8 lednum = 0;
    int8_t lednum_tmp = 0;

    /* 统计亮灯数和引导线数（安全：i+1 越界即判定线段结束） */
    for (uint8_t i = 0; i < sensorNum; i++)
    {
        if (scaner->detail & (1u << i))
        {
            lednum++;
            if ((i + 1 >= sensorNum) ||
                ((scaner->detail & (1u << (i + 1))) == 0u))
                ++linenum;
        }
    }
    scaner->lineNum = linenum;
    scaner->ledNum = lednum;

    /* 根据模式选择循线方式 */
    if (LEFT_RIGHT_LINE != 0)
    {
        /* 强制模式 */
        switch (LEFT_RIGHT_LINE)
        {
            case 1: /* 左循线 */
                scan_left_line(scaner, sensorNum, edge_ignore, &error, &lednum_tmp);
                break;

            case 2: /* 右循线 */
                scan_right_line(scaner, sensorNum, edge_ignore, &error, &lednum_tmp);
                break;

            case 3: /* 居中流水 */
                scan_center_line(scaner, sensorNum, edge_ignore, &error, &lednum_tmp);
                break;
        }

        if (lednum_tmp > 5)
        {
            Scaner.error = 0;
            return 0;
        }
    }
    else
    {
        /* 根据节点标志选择循线方式 */
        if ((nodesr.nowNode.flag & LEFT_LINE) == LEFT_LINE)
        {
            scan_left_line(scaner, sensorNum, edge_ignore, &error, &lednum_tmp);
        }
        else if ((nodesr.nowNode.flag & RIGHT_LINE) == RIGHT_LINE)
        {
            scan_right_line(scaner, sensorNum, edge_ignore, &error, &lednum_tmp);
        }
        else if ((nodesr.nowNode.flag & INGNORE) == INGNORE)
        {
            /* 短直立景点后退 */
            error = 0;
        }
        else if ((nodesr.nowNode.flag & LiuShui) == LiuShui)
        {
            /* 流水灯模式 */
            error = scan_liushui_line(scaner, sensorNum);
            Scaner.error = error;
            return 0;
        }
        else
        {
            /* 默认模式 */
            for (uint8_t i = edge_ignore; i < sensorNum - edge_ignore; i++)
            {
                lednum_tmp += (scaner->detail >> (sensorNum - 1 - i)) & 0x01;
                error += ((scaner->detail >> (sensorNum - 1 - i)) & 0x01) * line_weight[i];
            }

            if (scaner->ledNum >= 4 || lednum_tmp >= 4 || scaner->lineNum > 1)
            {
                Scaner.error = 0;
                return 0;
            }
        }

        if (lednum_tmp > 5)
        {
            Scaner.error = 0;
            return 0;
        }
    }

    /* 计算平均误差 */
    if (lednum == 0 || lednum_tmp == 0)
        error = 0;
    else
        error /= (float)lednum_tmp;

    Scaner.error = error;
    return 0;
}

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
                        unsigned char SensorNum, float *Error, u8 *LED_Num_Temp)
{
    float pos = 0;

    if (LEFT_RIGHT_LINE != 0)
    {
        /* 强制模式 */
        switch (LEFT_RIGHT_LINE)
        {
            case 1: /* 左循线 */
                for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
                {
                    int bit = (int)(SensorNum - 1 - i);   /* 先算 int，避免负移位 */
                    *LED_Num_Temp += (scaner->detail >> bit) & 0x01;
                    *Error += ((scaner->detail >> bit) & 0x01) * line_weight[i];

                    if ((scaner->detail >> bit) & 0x01)
                        pos += (float)i;

                    if ((scaner->detail >> bit) & 0x01)
                    {
                        if ((bit - 1) < (int)edge_ignore)
                            break;
                        if ((scaner->detail & (1u << (bit - 1))) == 0u)
                            break;
                    }
                }
                break;

            case 2: /* 右循线 */
                for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
                {
                    *LED_Num_Temp += (scaner->detail >> i) & 0x01;
                    *Error += ((scaner->detail >> i) & 0x01) * line_weight[SensorNum - 1 - i];

                    if ((scaner->detail >> i) & 0x01)
                        pos += (float)(SensorNum - 1 - i);

                    if ((scaner->detail >> i) & 0x01)
                    {
                        if ((i + 1 >= SensorNum - edge_ignore) ||
                            ((scaner->detail & (1u << (i + 1))) == 0u))
                            break;
                    }
                }
                break;

            case 3: /* 居中流水 */
            {
                float best_location = 0.0f;
                float temp_location = 0.0f;
                int   line_led_last = 0;
                int   len = 0;
                float temp_len = 0.0f;

                for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
                {
                    if (scaner->detail & (1u << i))
                    {
                        temp_location += (float)i;
                        temp_len += 1.0f;

                        if ((i + 1 >= SensorNum - edge_ignore) ||
                            ((scaner->detail & (1u << (i + 1))) == 0u))
                        {
                            temp_location /= temp_len;

                            if (fabsf(temp_location - (((float)(SensorNum - 1)) / 2)) <
                                fabsf(best_location - (((float)(SensorNum - 1)) / 2)))
                            {
                                best_location = temp_location;
                                line_led_last = (int)i;
                                len = (int)temp_len;
                            }
                            temp_location = 0;
                            temp_len = 0;
                        }
                    }
                }

                if (len > 0)
                {
                    for (int i = line_led_last - len + 1; i <= line_led_last; i++)
                    {
                        *LED_Num_Temp += (scaner->detail >> i) & 1u;
                        *Error += ((scaner->detail >> i) & 1u) * line_weight[SensorNum - 1 - i];
                        if ((scaner->detail >> i) & 1u)
                            pos += (float)(SensorNum - 1 - i);
                    }
                }
                break;
            }
        }
    }
    else
    {
        /* 根据节点标志选择循线方式 */
        if ((nodesr.nowNode.flag & LEFT_LINE) == LEFT_LINE)
        {
            for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
            {
                int bit = (int)(SensorNum - 1 - i);
                *LED_Num_Temp += (scaner->detail >> bit) & 0x01;
                *Error += ((scaner->detail >> bit) & 0x01) * line_weight[i];

                if ((scaner->detail >> bit) & 0x01)
                    pos += (float)i;

                if ((scaner->detail >> bit) & 0x01)
                {
                    if ((bit - 1) < (int)edge_ignore)
                        break;
                    if ((scaner->detail & (1u << (bit - 1))) == 0u)
                        break;
                }
            }
        }
        else if ((nodesr.nowNode.flag & RIGHT_LINE) == RIGHT_LINE)
        {
            for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
            {
                *LED_Num_Temp += (scaner->detail >> i) & 0x01;
                *Error += ((scaner->detail >> i) & 0x01) * line_weight[SensorNum - 1 - i];

                if ((scaner->detail >> i) & 0x01)
                    pos += (float)(SensorNum - 1 - i);

                if ((scaner->detail >> i) & 0x01)
                {
                    if ((i + 1 >= SensorNum - edge_ignore) ||
                        ((scaner->detail & (1u << (i + 1))) == 0u))
                        break;
                }
            }
        }
        else if ((nodesr.nowNode.flag & LiuShui) == LiuShui)
        {
            /* 居中流水（选择靠中心线；自动 edge_ignore 不适用） */
            float best_location = 0.0f;
            float temp_location = 0.0f;
            int   line_led_last = 0;
            int   len = 0;
            float temp_len = 0.0f;

            for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
            {
                if (scaner->detail & (1u << i))
                {
                    temp_location += (float)i;
                    temp_len += 1.0f;

                    if ((i + 1 >= SensorNum - edge_ignore) ||
                        ((scaner->detail & (1u << (i + 1))) == 0u))
                    {
                        temp_location /= temp_len;

                        if (fabsf(temp_location - (((float)(SensorNum - 1)) / 2)) <
                            fabsf(best_location - (((float)(SensorNum - 1)) / 2)))
                        {
                            best_location = temp_location;
                            line_led_last = (int)i;
                            len = (int)temp_len;
                        }
                        temp_location = 0;
                        temp_len = 0;
                    }
                }
            }

            if (len > 0)
            {
                for (int i = line_led_last - len + 1; i <= line_led_last; i++)
                {
                    *LED_Num_Temp += (scaner->detail >> i) & 0x01;
                    *Error += ((scaner->detail >> i) & 0x01) * line_weight[SensorNum - 1 - i];
                    if ((scaner->detail >> i) & 0x01)
                        pos += (float)(SensorNum - 1 - i);
                }
            }
        }
        else
        {
            /* 默认模式：普通循线自动屏蔽外围（仅局部变量，LEFT/RIGHT/LiuShui 不执行） */
            int8_t effective_edge = edge_ignore;
            if (scaner->ledNum >= 4 && scaner->lineNum >= 2)
                effective_edge = (edge_ignore > 4) ? edge_ignore : 4;

            for (uint8_t i = effective_edge; i < SensorNum - effective_edge; i++)
            {
                *LED_Num_Temp += (scaner->detail >> (SensorNum - 1 - i)) & 0x01;
                *Error += ((scaner->detail >> (SensorNum - 1 - i)) & 0x01) * line_weight[i];
                if ((scaner->detail >> (SensorNum - 1 - i)) & 0x01)
                    pos += (float)i;
            }
        }
    }

    /* 检查亮灯数是否过多 */
    if (*LED_Num_Temp > MAX_LED)
        return -1.0f;

    /* 除零保护：无亮灯时不可用 */
    if (*LED_Num_Temp == 0)
        return -1.0f;

    /* 计算平均位置 */
    pos /= (float)(*LED_Num_Temp);
    return pos;
}

/**
 * @brief  更新循迹值数组（递推平均滤波法）
 * @param  error_kind 错误类型
 * @param  pos        位置值
 * @param  error      误差值
 */
void Update_line_data(uint8_t error_kind, float pos, float error)
{
    /* 递推移位 */
    memmove(&line_data, &line_data[1], sizeof(struct Line_data) * 4);

    switch (error_kind)
    {
        case NO_ERROR:      /* 无错误 */
            line_data[4].error = error;
            line_data[4].pos = pos;
            line_data[4].truth = error_kind;
            break;

        case ALL_ERROR:     /* 全部错误 */
            line_data[4].error = -1;
            line_data[4].pos = -1;
            line_data[4].truth = error_kind;
            break;

        case POS_ERROR:     /* 位置错误 */
            line_data[4].error = error;
            line_data[4].pos = pos;
            line_data[4].truth = error_kind;
            break;

        default:
            break;
    }
}

/**
 * @brief  位置检测（判断与上次正确值是否相近）
 * @param  pos 当前位置
 * @return uint8_t 1=正确, 0=错误
 */
uint8_t pos_detect(float pos)
{
    uint8_t flag = 0;
    uint8_t idx = 0;

    /* 找到最近的正确值 */
    for (int i = 4; i >= 0; i--)
    {
        if (line_data[i].truth == 0)
        {
            idx = i;
            flag = 1;
            break;
        }
    }

    /* 有正确值则对比 */
    if (flag)
        return (fabs(line_data[idx].pos - pos) < POS_MAX_ERROR) ? 1 : 0;
    else
        return 1;   /* 无正确值，认为正确 */
}

/**
 * @brief  获取滤波后的循迹误差值
 * @return float 循迹误差值
 * @details 使用奖励机制从历史数据中选择最可靠的误差值
 */
float Get_scaner_error(void)
{
    uint8_t nums = 0;
    float error = 0;
    float pos_data[5] = {0};
    float pos_pos[5] = {0};
    uint8_t pos_error_nums = 0;
    uint8_t idex[5] = {0};

    /* 分类统计数据 */
    for (int i = 0; i < 5; i++)
    {
        if (line_data[i].truth == 0)
        {
            idex[nums] = i;
            nums++;
        }
        else if (line_data[i].truth == POS_ERROR)
        {
            pos_data[pos_error_nums] = line_data[i].error;
            pos_pos[pos_error_nums] = line_data[i].pos;
            pos_error_nums++;
        }
    }

    /* 情况 1：正确数据充足 */
    if (nums >= 3 || (nums >= pos_error_nums && nums != 0))
    {
        for (int i = 0; i < nums; i++)
            error += line_data[idex[i]].error;
        error /= (float)nums;
        return error;
    }

    /* 情况 2：没有正确值 */
    if (nums == 0)
    {
        if (pos_error_nums == 0)
            return 0;   /* 无法判断，保持原方向 */

        if (pos_error_nums == 1)
            return pos_data[0];

        /* 奖励机制：找邻近最多的位置 */
        uint8_t score[5] = {0};
        for (int i = 0; i < pos_error_nums; i++)
        {
            for (int j = i + 1; j < pos_error_nums; j++)
            {
                if (fabs(pos_pos[i] - pos_pos[j]) <= NEIGHBOR_RANGE)
                {
                    score[i]++;
                    score[j]++;
                }
            }
        }

        uint8_t max_score = score[0];
        uint8_t max_idx = 0;
        for (int i = 0; i < pos_error_nums; i++)
        {
            if (max_score < score[i])
            {
                max_score = score[i];
                max_idx = i;
            }
        }

        return (max_score == 0) ? 0 : pos_data[max_idx];
    }

    /* 情况 3：正确值较少，需要与错误值比较 */
    uint8_t score[5] = {0};
    for (int i = 0; i < pos_error_nums; i++)
    {
        for (int j = i + 1; j < pos_error_nums; j++)
        {
            if (fabs(pos_pos[i] - pos_pos[j]) <= NEIGHBOR_RANGE)
            {
                score[i]++;
                score[j]++;
            }
        }
    }

    uint8_t max_score = score[0];
    uint8_t max_idx = 0;
    for (int i = 0; i < pos_error_nums; i++)
    {
        if (max_score < score[i])
        {
            max_score = score[i];
            max_idx = i;
        }
    }

    if (nums >= max_score + 1)
    {
        /* 正确值偏多 */
        for (int i = 0; i < nums; i++)
            error += line_data[idex[i]].error;
        error /= (float)nums;
        return error;
    }
    else
    {
        return pos_data[max_idx];
    }
}

/**
 * @brief  粗略检测循迹值是否可用
 * @param  LED_Num  亮灯数量
 * @param  Line_Num 引导线数量
 * @return uint8_t  1=不可用, 0=可用
 */
uint8_t error_detect_one(u8 LED_Num, u8 Line_Num)
{
    /* 粗筛：保留原阈值思想(LED>=10 / Line>=4 / LED==0 / LED/Line>=4)，
     * 改为顺序判断并消除除零，禁止直接 LED_Num/Line_Num。 */
    if (LED_Num == 0)
        return 1;
    if (Line_Num == 0)
        return 1;
    if (LED_Num >= 10)
        return 1;
    if (Line_Num >= 4)
        return 1;
    if (LED_Num >= 4 * Line_Num)
        return 1;
    return 0;
}
