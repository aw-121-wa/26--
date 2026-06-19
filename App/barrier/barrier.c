/**
 * @file    barrier.c
 * @brief   障碍物处理模块
 * @details 包含zhunbei()准备函数、平台、桥、楼梯等障碍物处理
 */

#include "barrier.h"
#include "../map/map.h"
#include "../chassis/chassis_api.h"
#include "motor_task.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "imu.h"
#include "scaner.h"
#include "bsp_linefollower.h"
#include "delay.h"
#include "math.h"
#include "task_create.h"

/* ======================== 控制周期 ======================== */

#define CONTROL_CYCLE_MS        5       /* 控制周期 5ms */

/* ======================== 坡道角度阈值 ======================== */

#define BEGIN_UP     (basic_p + 5.0f)   /* 开始上坡 */
#define UP_PITCH     (basic_p + 20.0f)  /* 上坡中 */
#define AFTER_UP     (basic_p + 5.0f)   /* 上坡结束 */
#define BEGIN_DOWN   (basic_p - 5.0f)   /* 开始下坡 */
#define DOWN_PITCH   (basic_p - 20.0f)  /* 下坡中 */
#define AFTER_DOWN   (basic_p - 5.0f)   /* 下坡结束 */

/* ======================== 速度定义 ======================== */

#define GOSTAGE_SPEED           12      /* 上台速度 */
#define UPDOWN_SPEED_LOW        12      /* 坡道低速 */
#define UPDOWN_SPEED_HIGH       25      /* 坡道高速 */
#define HILL_APPROACH_SPEED     15      /* 楼梯接近速度 */

/* ======================== 延时常量 ======================== */

#define DELAY_STABLE            200     /* 稳定等待 */
#define DELAY_SHORT             100     /* 短暂等待 */
#define DELAY_TURN              50      /* 转弯后等待 */

/* ======================== 距离常量 ======================== */

#define DISTANCE_PLATFORM       20      /* 平台前进距离(cm) */
#define DISTANCE_PLATFORM_BACK  10      /* 平台转身前后退距离(cm) */
#define DISTANCE_P2_PLATFORM    75      /* P2平台前进距离(cm) */
#define DISTANCE_BRIDGE_ASCEND  15      /* 上桥后稳定距离(cm) */
#define DISTANCE_BRIDGE_TOTAL   65      /* 桥总长度(cm) */

/* ======================== 角度常量 ======================== */

#define ANGLE_TURN_180          179.0f  /* 180度转身 */

/* ======================== 检测阈值 ======================== */

#define RAMP_DETECT_STAGE       20.0f   /* 平台坡道检测阈值(度) */
#define RAMP_DETECT_BRIDGE      15.0f   /* 桥坡道检测阈值(度) */
#define RAMP_DETECT_HILL        40.0f   /* 楼梯坡道检测阈值(度) */
#define GYRO_STABLE_SAMPLES     50      /* 陀螺仪稳定采样次数 */

/* ======================== 加权融合系数 ======================== */

#define ANGLE_BLEND_CURRENT     0.2f    /* 当前角度权重 */
#define ANGLE_BLEND_ORIGIN      0.8f    /* 原始角度权重 */

/* ======================== 校正参数 ======================== */

#define RED_LINE_CORRECT_ANGLE  0.5f    /* 红线校正角度增量(度) */
#define INFRARED_CORRECT_ANGLE  0.08f   /* 红外校正角度增量(度) */

/* ======================== zhunbei() 准备函数 ======================== */

/**
 * @brief  准备函数 - 启动流程
 * @details 执行顺序：
 *          1. 停车 + 开启红外
 *          2. 等待挡板检测（Infrared_ahead 0->1->0）
 *          3. 陀螺仪模式下坡
 *          4. 切换循线模式
 */
void zhunbei(void)
{
    /* 停车 */
    pid_mode_switch(is_No);
    motor_all.Lspeed = 0;
    motor_all.Rspeed = 0;

    /* 开启红外 */
    infrare_open = 1;
    vTaskDelay(DELAY_SHORT);

    /* 等待挡板检测 - 碰到挡板 */
    while (Infrared_ahead == 0)
        vTaskDelay(5);

    /* 等待移除挡板 */
    while (Infrared_ahead == 1)
        vTaskDelay(5);

    /* 陀螺仪模式下坡 */
    mpuZreset(imu.yaw, nodesr.nowNode.angle);
    angle.AngleG = getAngleZ();
    motor_all.Gincrement = 0.5f;
    motor_all.Gspeed = GOSTAGE_SPEED;
    pid_mode_switch(is_Gyro);

    /* 等待下坡 */
    while (imu.pitch > DOWN_PITCH)
        vTaskDelay(CONTROL_CYCLE_MS);

    while (imu.pitch < AFTER_DOWN)
        vTaskDelay(CONTROL_CYCLE_MS);

    /* 切换巡线模式收尾 */
    encoder_clear();
    scaner_set.CatchsensorNum = 0;
    scaner_set.EdgeIgnore = 0;
    LEFT_RIGHT_LINE = 0;
    line_pid_obj = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
    TC_speed = (struct Gradual){0, 0, 0};
    pid_mode_switch_no_inherit(is_Line);
    motor_all.Cincrement = 0.5f;
    motor_all.Cspeed = SPEED1;
}

/* ======================== 通用平台处理（P1/P3/P4等） ======================== */

/**
 * @brief  通用平台处理函数
 * @details 执行顺序：
 *          1. 循线接近，检测坡道（20度）
 *          2. 上坡：init=25, pitch>=basic_p+5→12, pitch>=basic_p+20→12, pitch<=basic_p+5→done
 *          3. 前进20cm到平台
 *          4. 校准航向 + 后退10cm，为原地转身留空间
 *          5. 刹车 + 180度转身
 *          6. 下坡：init=12, pitch<=basic_p-5→12, pitch<=basic_p-20→25, pitch>=basic_p-5→done
 *          7. 设置到达标志
 */
void Stage(void)
{
    enum {
        STAGE_ASCEND,
        STAGE_TOP,
        STAGE_TURN,
        STAGE_DESCEND,
        STAGE_DONE
    } state = STAGE_ASCEND;

    float origin_angle = 0.0f;

    /* 循线前进 */
    Chassis_MotorControl(is_Line, SPEED1, SPEED1, 0);
    Chassis_ClearMileage();

    while (state != STAGE_DONE)
    {
        switch (state)
        {
        case STAGE_ASCEND:
            /* 检测坡道（20度） */
            GyroStableReset(GYRO_STABLE_SAMPLES, &origin_angle);

            if (Stage_DetectedRamp(RAMP_DETECT_STAGE))
            {
                if (origin_angle == 0) origin_angle = getAngleZ();
                /* 上坡：init=25, pitch>=basic_p+5→12, pitch>=basic_p+20→12, pitch<=basic_p+5→done */
                RampCtrl_Blocking(RAMP_ASCEND, UPDOWN_SPEED_HIGH, origin_angle,
                                  BEGIN_UP, UPDOWN_SPEED_LOW,
                                  UP_PITCH, UPDOWN_SPEED_LOW,
                                  AFTER_UP, 0);
                state = STAGE_TOP;
            }
            break;

        case STAGE_TOP:
            /* 到平台上，前进20cm */
            Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_PLATFORM, GOSTAGE_SPEED, getAngleZ());
            CarBrake();
            vTaskDelay(DELAY_SHORT);

            /* 校准平台航向后后退一小段，给原地转身留空间 */
            mpuZreset(imu.yaw, nodesr.nowNode.angle);
            Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_PLATFORM_BACK, -GOSTAGE_SPEED, getAngleZ());
            CarBrake();
            vTaskDelay(DELAY_STABLE);
            state = STAGE_TURN;
            break;

        case STAGE_TURN:
            /* 180度转身 */
            CarBrake();
            vTaskDelay(DELAY_SHORT);
            Chassis_Turn_By_StopGyro_Blocking(getAngleZ() + ANGLE_TURN_180, getAngleZ());
            vTaskDelay(DELAY_SHORT);
            state = STAGE_DESCEND;
            break;

        case STAGE_DESCEND:
            /* 下坡：init=12, pitch<=basic_p-5→12, pitch<=basic_p-20→25, pitch>=basic_p-5→done */
            RampCtrl_Blocking(RAMP_DESCEND, UPDOWN_SPEED_LOW, getAngleZ(),
                              BEGIN_DOWN, UPDOWN_SPEED_LOW,
                              DOWN_PITCH, UPDOWN_SPEED_HIGH,
                              AFTER_DOWN, 0);

            /* 切换回循线 */
            Chassis_MotorControl(is_Line, SPEED1, SPEED1, 0);
            state = STAGE_DONE;
            break;

        default:
            state = STAGE_DONE;
            break;
        }
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    /* 清除障碍标志，设置到达 */
    Chassis_ClearMileage();
    motor_all.Cspeed = 0;
    motor_pid_clear();
    nodesr.nowNode.function = 0;
    nodesr.flag |= 0x04;
}

/* ======================== P2平台处理 ======================== */

/**
 * @brief  P2平台处理函数
 * @details 执行顺序：
 *          1. 循线找角度
 *          2. 上坡：init=25, pitch>=basic_p+5→12, pitch>=basic_p+20→12, pitch<=basic_p+5→done
 *          3. 前进75cm到平台
 *          4. 刹车 + 180度转身
 *          5. 设置到达标志
 */
void Stage_P2(void)
{
    /* 保存原始PID参数 */
    struct PID_param origin_line = line_pid_param;
    struct PID_param origin_gyro = gyroG_pid_param;

    /* 调整PID参数用于上坡 */
    line_pid_param.kp = 35;
    line_pid_param.ki = 0.004f;
    line_pid_param.kd = 300;

    /* 循线前进，寻找合适的上坡角度 */
    Chassis_MotorControl(is_Line, UPDOWN_SPEED_LOW, UPDOWN_SPEED_LOW, 0);
    Chassis_ClearMileage();

    float tempAngle = -1;

    while (Scaner.ledNum < 8)
    {
        getline_error();
        if ((Scaner.detail & 0x0180) == 0x0180 && Scaner.ledNum < 5)
            tempAngle = getAngleZ();
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    if (tempAngle == -1)
        tempAngle = getAngleZ();

    /* 上坡：init=25, pitch>=basic_p+5→12, pitch>=basic_p+20→12, pitch<=basic_p+5→done */
    RampCtrl_Blocking(RAMP_ASCEND, UPDOWN_SPEED_HIGH, tempAngle,
                      BEGIN_UP, UPDOWN_SPEED_LOW,
                      UP_PITCH, UPDOWN_SPEED_LOW,
                      AFTER_UP, 0);

    /* 到平台上，前进75cm */
    Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_P2_PLATFORM, GOSTAGE_SPEED, tempAngle);

    /* 刹车 */
    CarBrake();
    vTaskDelay(DELAY_STABLE);

    /* 180度转身 */
    Chassis_Turn_By_StopGyro_Blocking(getAngleZ() + ANGLE_TURN_180, getAngleZ());

    /* 恢复PID参数 */
    line_pid_param = origin_line;
    gyroG_pid_param = origin_gyro;

    /* 清除障碍标志，设置到达 */
    Chassis_ClearMileage();
    motor_all.Cspeed = 0;
    motor_pid_clear();
    nodesr.nowNode.function = 0;
    nodesr.flag |= 0x04;
}

/* ======================== 过桥处理 ======================== */

/**
 * @brief  过桥处理函数
 * @details 执行顺序：
 *          1. 循线接近桥，校准陀螺仪
 *          2. 上桥：init=25, pitch>=basic_p+5→25, pitch>=basic_p+20→12, pitch<=basic_p+25→done
 *          3. 继续上坡：init=12, pitch>=basic_p+5→12, pitch<=basic_p+5→done
 *          4. 姿态校准（加权角度融合）
 *          5. 桥上直行（陀螺仪锁定+红线校正）
 *          6. 下桥：init=12, pitch<=basic_p-5→12, pitch<=basic_p-20→20, pitch>=basic_p-5→done
 *          7. 循线收尾
 */
void Barrier_Bridge(void)
{
    enum {
        BRIDGE_APPROACH,    /* 接近：循线检测坡道 */
        BRIDGE_ASCEND,      /* 上桥 */
        BRIDGE_CORRECT,     /* 姿态校准：加权角度融合 */
        BRIDGE_ACCELERATE,  /* 桥上直行 */
        BRIDGE_DESCEND,     /* 下桥 */
        BRIDGE_DONE
    } state = BRIDGE_APPROACH;

    float origin_angle = 0.0f;
    float tar_angle = 0.0f;

    Chassis_MotorControl(is_Line, SPEED0, SPEED0, 0);
    Chassis_ClearMileage();

    while (state != BRIDGE_DONE)
    {
        switch (state)
        {
        case BRIDGE_APPROACH:
            if (Stage_DetectedRamp(RAMP_DETECT_BRIDGE))
            {
                origin_angle = getAngleZ();
                /* 切换陀螺仪模式，锁定角度 */
                Chassis_MotorControl(is_Gyro, SPEED0, SPEED0, origin_angle);
                state = BRIDGE_ASCEND;
            }
            break;

        case BRIDGE_ASCEND:
            /* 上桥：init=25, pitch>=basic_p+5→25, pitch>=basic_p+20→12, pitch<=basic_p+25→done */
            RampCtrl_Blocking(RAMP_ASCEND, UPDOWN_SPEED_HIGH, origin_angle,
                              BEGIN_UP, UPDOWN_SPEED_HIGH,
                              UP_PITCH, UPDOWN_SPEED_LOW,
                              UP_PITCH + 20.0f, 0);

            /* 上桥后：前进15cm稳定 */
            Chassis_ClearMileage();
            Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_BRIDGE_ASCEND, UPDOWN_SPEED_LOW, origin_angle);

            /* 继续上坡到顶：init=12, pitch>=basic_p+5→12, pitch<=basic_p+5→done */
            RampCtrl_Blocking(RAMP_ASCEND, UPDOWN_SPEED_LOW, origin_angle,
                              0, UPDOWN_SPEED_LOW,
                              0, UPDOWN_SPEED_LOW,
                              AFTER_UP, 0);

            /* 更新基准角度 */
            origin_angle = getAngleZ();
            Chassis_ClearMileage();
            state = BRIDGE_CORRECT;
            break;

        case BRIDGE_CORRECT:
            /* 姿态校准：使用当前角度 */
            tar_angle = getAngleZ();
            Chassis_MotorControl(is_Gyro, SPEED2, SPEED2, tar_angle);
            Chassis_ClearMileage();
            state = BRIDGE_ACCELERATE;
            break;

        case BRIDGE_ACCELERATE:
            /* 桥上直行：陀螺仪锁定 */

            if (fabsf(Chassis_GetMileage()) >= DISTANCE_BRIDGE_TOTAL)
            {
                /* 接近下桥点，减速 */
                Chassis_MotorControl(is_Gyro, UPDOWN_SPEED_LOW, UPDOWN_SPEED_LOW, getAngleZ());
                state = BRIDGE_DESCEND;
            }
            break;

        case BRIDGE_DESCEND:
            /* 下桥：init=12, pitch<=basic_p-5→12, pitch<=basic_p-20→20, pitch>=basic_p-5→done */
            RampCtrl_Blocking(RAMP_DESCEND, UPDOWN_SPEED_LOW, getAngleZ(),
                              BEGIN_DOWN, UPDOWN_SPEED_LOW,
                              DOWN_PITCH, SPEED0,
                              AFTER_DOWN, 0);

            /* 切换回循线 */
            Chassis_MotorControl(is_Line, SPEED1, SPEED1, 0);

            /* 清除障碍标志 */
            nodesr.nowNode.function = 0;
            nodesr.flag |= 0x04;
            state = BRIDGE_DONE;
            break;

        default:
            state = BRIDGE_DONE;
            break;
        }
        vTaskDelay(CONTROL_CYCLE_MS);
    }
}

/* ======================== 楼梯处理 ======================== */

/**
 * @brief  楼梯/山地处理函数
 * @details 执行顺序：
 *          1. 循线接近，检测坡道（40度）
 *          2. 上坡：init=12, pitch>=basic_p+5→12, pitch>=basic_p+15→12, pitch<=basic_p+5→done
 *          3. 下坡：init=12, pitch<=basic_p→12, pitch<=basic_p-8→12, pitch>=basic_p-3→done
 *          4. 刹车 + 设置到达标志
 */
void Barrier_Hill(void)
{
    enum {
        HILL_APPROACH,
        HILL_ASCEND,
        HILL_DESCEND,
        HILL_DONE
    } state = HILL_APPROACH;

    float origin_angle = 0.0f;

    Chassis_MotorControl(is_Line, 12, 12, 0);
    vTaskDelay(10);
    Chassis_ClearMileage();

    while (state != HILL_DONE)
    {
        switch (state)
        {
        case HILL_APPROACH:
            GyroStableReset(GYRO_STABLE_SAMPLES, &origin_angle);

            if (Stage_DetectedRamp(RAMP_DETECT_HILL))
            {
                if (origin_angle == 0) origin_angle = getAngleZ();
                Chassis_MotorControl(is_Gyro, HILL_APPROACH_SPEED, HILL_APPROACH_SPEED, origin_angle);
                state = HILL_ASCEND;
            }
            break;

        case HILL_ASCEND:
            /* 上坡：init=12, pitch>=basic_p+5→12, pitch>=basic_p+15→12, pitch<=basic_p+5→done */
            RampCtrl_Blocking(RAMP_ASCEND, UPDOWN_SPEED_LOW, origin_angle,
                              basic_p + 5.0f, UPDOWN_SPEED_LOW,
                              basic_p + 15.0f, UPDOWN_SPEED_LOW,
                              basic_p + 5.0f, 0.05f);
            state = HILL_DESCEND;
            break;

        case HILL_DESCEND:
            /* 下坡：init=12, pitch<=basic_p→12, pitch<=basic_p-8→12, pitch>=basic_p-3→done */
            RampCtrl_Blocking(RAMP_DESCEND, UPDOWN_SPEED_LOW, origin_angle,
                              basic_p, UPDOWN_SPEED_LOW,
                              basic_p - 8.0f, UPDOWN_SPEED_LOW,
                              basic_p - 3.0f, 0.05f);
            state = HILL_DONE;
            break;

        default:
            state = HILL_DONE;
            break;
        }
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    /* 刹车 */
    CarBrake();

    /* 清除障碍标志 */
    nodesr.nowNode.function = 0;
    nodesr.flag |= 0x04;
}
