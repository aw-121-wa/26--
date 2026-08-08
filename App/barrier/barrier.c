/**
 * @file barrier.c
 * @brief 中国机器人大赛探险赛障碍动作。
 *
 * 所有会等待里程、姿态、红外或视觉结果的动作都有硬超时。障碍函数只在
 * 成功时设置节点到达标志；失败时保留底盘的首个停车原因。
 */

#include "barrier.h"
#include "../map/map.h"
#include "../chassis/chassis_api.h"
#include "../vision/vision_api.h"
#include "main_task.h"
#include "motor_task.h"
#include "encoder.h"
#include "pid.h"
#include "imu.h"
#include "scaner.h"
#include "bsp_linefollower.h"
#include "rudder_control.h"
#include "delay.h"
#include "math.h"

#define CONTROL_CYCLE_MS            5u
#define START_GATE_TIMEOUT_MS        180000u
#define BARRIER_APPROACH_TIMEOUT_MS  12000u
#define BARRIER_SENSOR_TIMEOUT_MS    10000u
#define BARRIER_MOTION_TIMEOUT_MS    12000u
#define BARRIER_TURN_TIMEOUT_MS      5000u

#define BEGIN_UP     (basic_p + 5.0f)
#define UP_PITCH     (basic_p + 20.0f)
#define AFTER_UP     (basic_p + 5.0f)
#define BEGIN_DOWN   (basic_p - 5.0f)
#define DOWN_PITCH   (basic_p - 20.0f)
#define AFTER_DOWN   (basic_p - 5.0f)

#define GOSTAGE_SPEED           12.0f
#define UPDOWN_SPEED_LOW        12.0f
#define UPDOWN_SPEED_HIGH       25.0f
#define HILL_APPROACH_SPEED     15.0f
#define DISTANCE_PLATFORM_FRONT 10.0f
#define DISTANCE_PLATFORM_BACK  6.0f
#define DISTANCE_P2_PLATFORM    75.0f
#define DISTANCE_BRIDGE_ASCEND  15.0f
#define DISTANCE_BRIDGE_TOTAL   65.0f
#define DISTANCE_WAVE_ENTRY_MAX 40.0f
#define DISTANCE_SEESAW_CROSS   48.0f

#define P2_DOWN_BIAS            0.0f
#define BRIDGE_RIGHT_BIAS       0.0f
#define BRIDGE_RED_ANGLE        2.0f
#define BRIDGE_RED_LEFT_MASK    0xF800u
#define BRIDGE_RED_RIGHT_MASK   0x001Fu
#define BRIDGE_RED_HOLD_TICKS   20u
#define SCANER_CENTER_MASK      0x0180u
#define NODE_ARRIVED_FLAG       0x04u
#define LEFT_LINE_MODE          1u
#define RIGHT_LINE_MODE         2u
#define CENTER_LINE_MODE        3u
#define INVALID_ANGLE           (-1000.0f)

#define RAMP_DETECT_STAGE       20.0f
#define RAMP_DETECT_BRIDGE      5.0f
#define RAMP_DETECT_HILL        15.0f
#define GYRO_STABLE_SAMPLES     50u
#define P1_STAGE_APPROACH_SPEED SPEED0
#define P1_STAGE_RAMP_DETECT    10.0f

typedef struct {
    uint8_t hold;
    uint8_t side;
    float hold_angle;
    float saved_kp;
} BridgeRedState_t;

static uint8_t barrier_timed_out(TickType_t started, uint32_t timeout_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);

    if (ticks == 0u)
        ticks = 1u;
    return ((TickType_t)(xTaskGetTickCount() - started) >= ticks) ? 1u : 0u;
}

static BarrierResult_t barrier_fail(BarrierResult_t result)
{
    if (!Chassis_IsStopLocked())
    {
        Chassis_ForceStop(result == BARRIER_RESULT_VISION_FAILED
                          ? CHASSIS_STOP_VISION_TIMEOUT
                          : CHASSIS_STOP_BARRIER_FAILED);
    }
    return result;
}

static BarrierResult_t barrier_from_action(ChassisActionResult_t result)
{
    switch (result)
    {
    case CHASSIS_ACTION_OK:
        return BARRIER_RESULT_OK;
    case CHASSIS_ACTION_TIMEOUT:
        return BARRIER_RESULT_TIMEOUT;
    case CHASSIS_ACTION_SENSOR_FAULT:
        return BARRIER_RESULT_SENSOR_FAULT;
    case CHASSIS_ACTION_STOPPED:
    default:
        return BARRIER_RESULT_STOPPED;
    }
}

static BarrierResult_t barrier_done(uint8_t stop_line, uint8_t clear_pid)
{
    Chassis_ClearMileage();
    if (stop_line)
        motor_all.Cspeed = 0.0f;
    if (clear_pid)
        motor_pid_clear();
    nodesr.nowNode.function = 0;
    nodesr.flag |= NODE_ARRIVED_FLAG;
    return BARRIER_RESULT_OK;
}

static void barrier_continue_after_wave(void)
{
    Chassis_ClearMileage();
    nodesr.nowNode.function = 0;
    nodesr.flag &= (uint8_t)(~NODE_ARRIVED_FLAG);
}

static void line_mode_reset(uint8_t mode)
{
    scaner_set.CatchsensorNum = 0;
    scaner_set.EdgeIgnore = 0;
    Line_SetTrackModeBumpless(mode);
}

static void line_mode_reset_by_flag(u32 flag)
{
    if ((flag & LEFT_LINE) == LEFT_LINE)
        line_mode_reset(LEFT_LINE_MODE);
    else if ((flag & RIGHT_LINE) == RIGHT_LINE)
        line_mode_reset(RIGHT_LINE_MODE);
    else
        line_mode_reset(CENTER_LINE_MODE);
}

static float barrier_norm_angle(float value)
{
    while (value > 180.0f)
        value -= 360.0f;
    while (value <= -180.0f)
        value += 360.0f;
    return value;
}

static BarrierResult_t wait_for_pitch_below(float threshold, uint32_t timeout_ms)
{
    TickType_t started = xTaskGetTickCount();

    while (imu.pitch > threshold)
    {
        if (Chassis_IsStopLocked())
            return BARRIER_RESULT_STOPPED;
        if (barrier_timed_out(started, timeout_ms))
            return barrier_fail(BARRIER_RESULT_TIMEOUT);
        vTaskDelay(pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }
    return BARRIER_RESULT_OK;
}

static BarrierResult_t wait_for_pitch_above(float threshold, uint32_t timeout_ms)
{
    TickType_t started = xTaskGetTickCount();

    while (imu.pitch < threshold)
    {
        if (Chassis_IsStopLocked())
            return BARRIER_RESULT_STOPPED;
        if (barrier_timed_out(started, timeout_ms))
            return barrier_fail(BARRIER_RESULT_TIMEOUT);
        vTaskDelay(pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }
    return BARRIER_RESULT_OK;
}

static BarrierResult_t wait_for_infrared(uint8_t level, uint32_t timeout_ms)
{
    TickType_t started = xTaskGetTickCount();

    while (Infrared_ahead != level)
    {
        if (Chassis_IsStopLocked())
            return BARRIER_RESULT_STOPPED;
        if (barrier_timed_out(started, timeout_ms))
            return barrier_fail(BARRIER_RESULT_SENSOR_FAULT);
        vTaskDelay(pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }
    return BARRIER_RESULT_OK;
}

static BarrierResult_t wait_for_ramp(float threshold, uint32_t timeout_ms)
{
    TickType_t started = xTaskGetTickCount();

    while (!Stage_DetectedRamp(threshold))
    {
        if (Chassis_IsStopLocked())
            return BARRIER_RESULT_STOPPED;
        if (barrier_timed_out(started, timeout_ms))
            return barrier_fail(BARRIER_RESULT_SENSOR_FAULT);
        vTaskDelay(pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }
    return BARRIER_RESULT_OK;
}

static BarrierResult_t stage_line_ramp_ctrl(RampDir_t dir, float init_speed,
                                             float thresh1, float speed1,
                                             float thresh2, float speed2,
                                             float done_thresh,
                                             uint32_t timeout_ms)
{
    enum { RAMP_INIT, RAMP_PHASE1, RAMP_PHASE2 } state = RAMP_INIT;
    TickType_t started = xTaskGetTickCount();

    Chassis_MotorControl(is_Line, init_speed, init_speed, 0.0f);
    Chassis_SetTargetSpeed(init_speed);

    while (!Chassis_IsStopLocked())
    {
        float pitch = imu.pitch;

        if (dir == RAMP_ASCEND)
        {
            if (state == RAMP_INIT && pitch >= thresh1)
            {
                Chassis_SetTargetSpeed(speed1);
                state = RAMP_PHASE1;
            }
            else if (state == RAMP_PHASE1 && pitch >= thresh2)
            {
                Chassis_SetTargetSpeed(speed2);
                state = RAMP_PHASE2;
            }
            else if (state == RAMP_PHASE2 && pitch <= done_thresh)
            {
                return BARRIER_RESULT_OK;
            }
        }
        else
        {
            if (state == RAMP_INIT && pitch <= thresh1)
            {
                Chassis_SetTargetSpeed(speed1);
                state = RAMP_PHASE1;
            }
            else if (state == RAMP_PHASE1 && pitch <= thresh2)
            {
                Chassis_SetTargetSpeed(speed2);
                state = RAMP_PHASE2;
            }
            else if (state == RAMP_PHASE2 && pitch >= done_thresh)
            {
                return BARRIER_RESULT_OK;
            }
        }

        if (barrier_timed_out(started, timeout_ms))
            return barrier_fail(BARRIER_RESULT_TIMEOUT);
        vTaskDelay(pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }

    return BARRIER_RESULT_STOPPED;
}

static BarrierResult_t barrier_recognize_scenic(void)
{
    VisionResult_t result;
    VisionStatus_t status;

    status = Vision_ScanScenicSign(VISION_DIRECTION_CENTER, &result);
    if (status != VISION_STATUS_OK)
        return barrier_fail(BARRIER_RESULT_VISION_FAILED);

    Vision_NotifyScenicSign(&result);
    return BARRIER_RESULT_OK;
}

BarrierResult_t zhunbei(void)
{
    TickType_t started;
    BarrierResult_t result;

    CarBrake();
    infrare_open = 1u;

    started = xTaskGetTickCount();
    while (Infrared_ahead == 0u)
    {
        if (barrier_timed_out(started, START_GATE_TIMEOUT_MS))
            return barrier_fail(BARRIER_RESULT_SENSOR_FAULT);
        vTaskDelay(pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }

    started = xTaskGetTickCount();
    while (Infrared_ahead != 0u)
    {
        if (barrier_timed_out(started, START_GATE_TIMEOUT_MS))
            return barrier_fail(BARRIER_RESULT_SENSOR_FAULT);
        vTaskDelay(pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }

#if LINE_DEBUG_MODE
    line_mode_reset(CENTER_LINE_MODE);
    Chassis_SetTargetSpeed(SPEED3);
    Chassis_SetMode(is_Line);
#else
    mpuZreset(imu.yaw, nodesr.nowNode.angle);
    angle.AngleG = barrier_norm_angle(getAngleZ() + P2_DOWN_BIAS);
    Chassis_MotorControl(is_Gyro, GOSTAGE_SPEED, GOSTAGE_SPEED, angle.AngleG);

    result = wait_for_pitch_below(BEGIN_DOWN, BARRIER_SENSOR_TIMEOUT_MS);
    if (result != BARRIER_RESULT_OK)
        return result;

    line_mode_reset(CENTER_LINE_MODE);
    Chassis_SetTargetSpeed(SPEED0);
    Chassis_SetMode(is_Line);
    result = wait_for_pitch_above(AFTER_DOWN, BARRIER_SENSOR_TIMEOUT_MS);
    if (result != BARRIER_RESULT_OK)
        return result;

    Chassis_ClearMileage();
    line_mode_reset(CENTER_LINE_MODE);
    motor_all.Cincrement = 0.5f;
    Chassis_SetTargetSpeed(SPEED0);
#endif
    return BARRIER_RESULT_OK;
}

BarrierResult_t Stage(void)
{
    enum {
        STAGE_ASCEND,
        STAGE_TOP,
        STAGE_TURN,
        STAGE_DESCEND,
        STAGE_DONE
    } state = STAGE_ASCEND;
    float origin_angle = 0.0f;
    float approach_speed = SPEED1;
    float ramp_detect = RAMP_DETECT_STAGE;
    BarrierResult_t result;

    if (nodesr.nowNode.nodenum == P1)
    {
        approach_speed = P1_STAGE_APPROACH_SPEED;
        ramp_detect = P1_STAGE_RAMP_DETECT;
        line_mode_reset(CENTER_LINE_MODE);
    }

    Chassis_MotorControl(is_Line, approach_speed, approach_speed, 0.0f);
    Chassis_ClearMileage();

    while (state != STAGE_DONE)
    {
        switch (state)
        {
        case STAGE_ASCEND:
            result = wait_for_ramp(ramp_detect, BARRIER_APPROACH_TIMEOUT_MS);
            if (result != BARRIER_RESULT_OK)
                return result;
            GyroStableReset(GYRO_STABLE_SAMPLES, &origin_angle);
            if (nodesr.nowNode.nodenum == P1)
            {
                result = stage_line_ramp_ctrl(RAMP_ASCEND, UPDOWN_SPEED_HIGH,
                                              BEGIN_UP, UPDOWN_SPEED_LOW,
                                              UP_PITCH, UPDOWN_SPEED_LOW,
                                              AFTER_UP, BARRIER_MOTION_TIMEOUT_MS);
                origin_angle = getAngleZ();
            }
            else
            {
                result = barrier_from_action(
                    Chassis_Ramp_Timeout(RAMP_ASCEND, UPDOWN_SPEED_HIGH,
                                         origin_angle, BEGIN_UP,
                                         UPDOWN_SPEED_LOW, UP_PITCH,
                                         UPDOWN_SPEED_LOW, AFTER_UP, 0.0f,
                                         BARRIER_MOTION_TIMEOUT_MS));
            }
            if (result != BARRIER_RESULT_OK)
                return barrier_fail(result);
            state = STAGE_TOP;
            break;

        case STAGE_TOP:
            Chassis_MotorControl(is_Gyro, GOSTAGE_SPEED, GOSTAGE_SPEED,
                                 origin_angle);
            result = wait_for_infrared(1u, BARRIER_SENSOR_TIMEOUT_MS);
            if (result != BARRIER_RESULT_OK)
                return result;
            result = barrier_from_action(
                Chassis_DriveDistance_Timeout(is_Gyro,
                                              DISTANCE_PLATFORM_FRONT,
                                              GOSTAGE_SPEED, origin_angle,
                                              BARRIER_MOTION_TIMEOUT_MS));
            if (result != BARRIER_RESULT_OK)
                return barrier_fail(result);
            result = barrier_from_action(
                Chassis_DriveDistance_Timeout(is_Gyro,
                                              DISTANCE_PLATFORM_BACK,
                                              -GOSTAGE_SPEED, origin_angle,
                                              BARRIER_MOTION_TIMEOUT_MS));
            if (result != BARRIER_RESULT_OK)
                return barrier_fail(result);
            state = STAGE_TURN;
            break;

        case STAGE_TURN:
            result = barrier_from_action(
                Chassis_Turn180_Timeout(BARRIER_TURN_TIMEOUT_MS));
            if (result != BARRIER_RESULT_OK)
                return barrier_fail(result);
            state = STAGE_DESCEND;
            break;

        case STAGE_DESCEND:
        {
            Chassis_SetMode(is_Gyro);
            motor_all.Gspeed = UPDOWN_SPEED_LOW;
            angle.AngleG = getAngleZ();

            result = wait_for_pitch_below(BEGIN_DOWN,
                                          BARRIER_SENSOR_TIMEOUT_MS);
            if (result != BARRIER_RESULT_OK)
                return result;

            encoder_clear();
            line_mode_reset(CENTER_LINE_MODE);
            Chassis_SetTargetSpeed(SPEED0);
            Chassis_SetMode(is_Line);
            result = wait_for_pitch_above(AFTER_DOWN,
                                          BARRIER_SENSOR_TIMEOUT_MS);
            if (result != BARRIER_RESULT_OK)
                return result;

            Chassis_ClearMileage();
            line_mode_reset(CENTER_LINE_MODE);
            motor_all.Cincrement = 0.5f;
            Chassis_SetTargetSpeed(SPEED2);
            state = STAGE_DONE;
            break;
        }

        default:
            return barrier_fail(BARRIER_RESULT_SENSOR_FAULT);
        }
    }

    return barrier_done(0u, 0u);
}

BarrierResult_t Stage_P2(void)
{
    struct PID_param old_line = line_pid_param;
    struct PID_param old_gyro = gyroG_pid_param;
    TickType_t started;
    float temp_angle = INVALID_ANGLE;
    BarrierResult_t result;

    line_pid_param.kp = 35.0f;
    line_pid_param.ki = 0.004f;
    line_pid_param.kd = 300.0f;
    Chassis_MotorControl(is_Line, UPDOWN_SPEED_LOW, UPDOWN_SPEED_LOW, 0.0f);
    Chassis_ClearMileage();

    started = xTaskGetTickCount();
    while (Scaner.ledNum < 8u)
    {
        getline_error();
        if ((Scaner.detail & SCANER_CENTER_MASK) == SCANER_CENTER_MASK &&
            Scaner.ledNum < 5u)
        {
            temp_angle = getAngleZ();
        }
        if (barrier_timed_out(started, BARRIER_APPROACH_TIMEOUT_MS))
        {
            line_pid_param = old_line;
            gyroG_pid_param = old_gyro;
            return barrier_fail(BARRIER_RESULT_SENSOR_FAULT);
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }

    if (temp_angle == INVALID_ANGLE)
        temp_angle = getAngleZ();

    result = barrier_from_action(
        Chassis_Ramp_Timeout(RAMP_ASCEND, UPDOWN_SPEED_HIGH, temp_angle,
                             BEGIN_UP, UPDOWN_SPEED_LOW, UP_PITCH,
                             UPDOWN_SPEED_LOW, AFTER_UP, 0.0f,
                             BARRIER_MOTION_TIMEOUT_MS));
    if (result == BARRIER_RESULT_OK)
    {
        result = barrier_from_action(
            Chassis_DriveDistance_Timeout(is_Gyro, DISTANCE_P2_PLATFORM,
                                          GOSTAGE_SPEED, temp_angle,
                                          BARRIER_MOTION_TIMEOUT_MS));
    }
    if (result == BARRIER_RESULT_OK)
    {
        result = barrier_from_action(
            Chassis_TurnTo_Timeout(getAngleZ() + 180.0f, getAngleZ(),
                                   BARRIER_TURN_TIMEOUT_MS));
    }

    line_pid_param = old_line;
    gyroG_pid_param = old_gyro;
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    return barrier_done(1u, 1u);
}

static uint8_t bridge_red_correct(float base_angle, float *target,
                                  BridgeRedState_t *state)
{
    getline_error();

    if ((Scaner.detail & BRIDGE_RED_LEFT_MASK) != 0u)
    {
        if (state->hold == 0u || state->side != 1u)
        {
            state->saved_kp = gyroG_pid_param.kp;
            gyroG_pid_param.kp = state->saved_kp * 1.8f;
        }
        state->hold = BRIDGE_RED_HOLD_TICKS;
        state->side = 1u;
        state->hold_angle = barrier_norm_angle(getAngleZ() + BRIDGE_RED_ANGLE);
    }
    else if ((Scaner.detail & BRIDGE_RED_RIGHT_MASK) != 0u)
    {
        if (state->hold == 0u || state->side != 2u)
        {
            state->saved_kp = gyroG_pid_param.kp;
            gyroG_pid_param.kp = state->saved_kp * 1.8f;
        }
        state->hold = BRIDGE_RED_HOLD_TICKS;
        state->side = 2u;
        state->hold_angle = barrier_norm_angle(getAngleZ() - BRIDGE_RED_ANGLE);
    }
    else if (state->hold > 0u)
    {
        state->hold--;
        if (state->hold == 0u)
        {
            gyroG_pid_param.kp = state->saved_kp;
            state->side = 0u;
        }
    }

    *target = state->hold > 0u ? state->hold_angle : base_angle;
    angle.AngleG = *target;
    motor_all.Gspeed = state->hold > 0u ? SPEED1 : SPEED2;
    return state->hold > 0u ? 1u : 0u;
}

static void bridge_red_restore(BridgeRedState_t *state)
{
    if (state->side != 0u)
        gyroG_pid_param.kp = state->saved_kp;
    state->hold = 0u;
    state->side = 0u;
}

BarrierResult_t Barrier_Bridge(void)
{
    enum {
        BRIDGE_APPROACH,
        BRIDGE_ASCEND,
        BRIDGE_CORRECT,
        BRIDGE_ACCELERATE,
        BRIDGE_DESCEND,
        BRIDGE_DONE
    } state = BRIDGE_APPROACH;
    BridgeRedState_t red = {0u, 0u, 0.0f, 0.0f};
    TickType_t phase_started = xTaskGetTickCount();
    float origin_angle = nodesr.nowNode.angle;
    float entry_angle = 0.0f;
    float base_angle = 0.0f;
    float target_angle = 0.0f;
    BarrierResult_t result = BARRIER_RESULT_OK;

    line_mode_reset_by_flag(nodesr.nowNode.flag);
    Chassis_MotorControl(is_Line, SPEED0, SPEED0, 0.0f);
    Chassis_ClearMileage();

    while (state != BRIDGE_DONE)
    {
        if (Chassis_IsStopLocked())
            return BARRIER_RESULT_STOPPED;

        switch (state)
        {
        case BRIDGE_APPROACH:
            if (fabsf(Chassis_GetMileage()) >= 35.0f &&
                Stage_DetectedRamp(RAMP_DETECT_BRIDGE))
            {
                mpuZreset(imu.yaw, nodesr.nowNode.angle);
                entry_angle = barrier_norm_angle(origin_angle + BRIDGE_RIGHT_BIAS);
                Chassis_MotorControl(is_Gyro, SPEED0, SPEED0, entry_angle);
                state = BRIDGE_ASCEND;
            }
            else if (barrier_timed_out(phase_started,
                                       BARRIER_APPROACH_TIMEOUT_MS))
            {
                return barrier_fail(BARRIER_RESULT_SENSOR_FAULT);
            }
            break;

        case BRIDGE_ASCEND:
            result = barrier_from_action(
                Chassis_Ramp_Timeout(RAMP_ASCEND, UPDOWN_SPEED_HIGH,
                                     entry_angle, BEGIN_UP,
                                     UPDOWN_SPEED_HIGH, UP_PITCH,
                                     UPDOWN_SPEED_LOW, UP_PITCH + 20.0f,
                                     0.0f, BARRIER_MOTION_TIMEOUT_MS));
            if (result != BARRIER_RESULT_OK)
                return barrier_fail(result);

            result = barrier_from_action(
                Chassis_DriveDistance_Timeout(is_Gyro,
                                              DISTANCE_BRIDGE_ASCEND,
                                              UPDOWN_SPEED_LOW, entry_angle,
                                              BARRIER_MOTION_TIMEOUT_MS));
            if (result != BARRIER_RESULT_OK)
                return barrier_fail(result);

            result = barrier_from_action(
                Chassis_Ramp_Timeout(RAMP_ASCEND, UPDOWN_SPEED_LOW,
                                     entry_angle - imu.pitch * 0.03f,
                                     0.0f, UPDOWN_SPEED_LOW, 0.0f,
                                     UPDOWN_SPEED_LOW, AFTER_UP, 0.0f,
                                     BARRIER_MOTION_TIMEOUT_MS));
            if (result != BARRIER_RESULT_OK)
                return barrier_fail(result);
            state = BRIDGE_CORRECT;
            break;

        case BRIDGE_CORRECT:
            base_angle = barrier_norm_angle(origin_angle + BRIDGE_RIGHT_BIAS);
            target_angle = base_angle;
            Chassis_MotorControl(is_Gyro, SPEED1, SPEED1, target_angle);
            Chassis_ClearMileage();
            state = BRIDGE_ACCELERATE;
            break;

        case BRIDGE_ACCELERATE:
            (void)bridge_red_correct(base_angle, &target_angle, &red);
            if (fabsf(Chassis_GetMileage()) >= DISTANCE_BRIDGE_TOTAL)
            {
                motor_all.Gspeed = UPDOWN_SPEED_LOW;
                state = BRIDGE_DESCEND;
            }
            break;

        case BRIDGE_DESCEND:
            result = barrier_from_action(
                Chassis_Ramp_Timeout(RAMP_DESCEND, UPDOWN_SPEED_LOW,
                                     target_angle, BEGIN_DOWN,
                                     UPDOWN_SPEED_LOW, DOWN_PITCH, SPEED0,
                                     AFTER_DOWN, 0.0f,
                                     BARRIER_MOTION_TIMEOUT_MS));
            bridge_red_restore(&red);
            if (result != BARRIER_RESULT_OK)
                return barrier_fail(result);

            line_mode_reset_by_flag(nodesr.nowNode.flag);
            Chassis_MotorControl(is_Line, SPEED1, SPEED1, 0.0f);
            motor_pid_clear();
            line_pid_obj.integral = 0.0f;
            line_pid_obj.last_bias = 0.0f;
            line_pid_obj.last_differential = 0.0f;
            state = BRIDGE_DONE;
            break;

        default:
            bridge_red_restore(&red);
            return barrier_fail(BARRIER_RESULT_SENSOR_FAULT);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }

    return barrier_done(0u, 0u);
}

BarrierResult_t Barrier_WavedPlate(float length)
{
    struct PID_param old_line = line_pid_param;
    struct PID_param old_gyro = gyroG_pid_param;
    int8_t old_ignore = scaner_set.EdgeIgnore;
    uint8_t old_mode = LEFT_RIGHT_LINE;
    TickType_t started = xTaskGetTickCount();
    BarrierResult_t result = BARRIER_RESULT_OK;

    if (length <= 0.0f)
        return barrier_fail(BARRIER_RESULT_SENSOR_FAULT);

    scaner_set.EdgeIgnore = 0;
    Line_SetTrackModeBumpless(CENTER_LINE_MODE);
    Chassis_MotorControl(is_Line, SPEED0, SPEED0, 0.0f);
    Chassis_ClearMileage();

    while (Scaner.ledNum <= 4u || Scaner.lineNum == 1u)
    {
        getline_error();
        Cross_getline();
        if ((Cross_Scaner.detail & SCANER_CENTER_MASK) == SCANER_CENTER_MASK)
            mpuZreset(imu.yaw, nodesr.nowNode.angle);
        if (fabsf(Chassis_GetMileage()) >= DISTANCE_WAVE_ENTRY_MAX)
            break;
        if (barrier_timed_out(started, BARRIER_APPROACH_TIMEOUT_MS))
        {
            result = BARRIER_RESULT_SENSOR_FAULT;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }

    if (result == BARRIER_RESULT_OK)
    {
        line_pid_param.kp = 35.0f;
        line_pid_param.ki = 0.0f;
        line_pid_param.kd = 15.0f;
        scaner_set.EdgeIgnore = 3;
        Line_SetTrackModeBumpless(CENTER_LINE_MODE);
        result = barrier_from_action(
            Chassis_DriveDistance_Timeout(is_Line, length,
                                          UPDOWN_SPEED_LOW, 0.0f,
                                          BARRIER_MOTION_TIMEOUT_MS));
    }

    WavePlateLeft_Flag = 0u;
    WavePlateRight_Flag = 0u;
    scaner_set.EdgeIgnore = old_ignore;
    line_pid_param = old_line;
    gyroG_pid_param = old_gyro;
    Line_SetTrackModeBumpless(old_mode);

    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    barrier_continue_after_wave();
    return BARRIER_RESULT_OK;
}

BarrierResult_t Barrier_Hill(void)
{
    float heading;
    BarrierResult_t result;

    Chassis_MotorControl(is_Line, HILL_APPROACH_SPEED,
                         HILL_APPROACH_SPEED, 0.0f);
    Chassis_ClearMileage();
    result = wait_for_ramp(RAMP_DETECT_HILL, BARRIER_APPROACH_TIMEOUT_MS);
    if (result != BARRIER_RESULT_OK)
        return result;

    GyroStableReset(GYRO_STABLE_SAMPLES, &heading);
    result = barrier_from_action(
        Chassis_Ramp_Timeout(RAMP_ASCEND, UPDOWN_SPEED_LOW, heading,
                             basic_p + 5.0f, UPDOWN_SPEED_LOW,
                             basic_p + 15.0f, UPDOWN_SPEED_LOW,
                             basic_p + 5.0f, 0.05f,
                             BARRIER_MOTION_TIMEOUT_MS));
    if (result == BARRIER_RESULT_OK)
    {
        result = barrier_from_action(
            Chassis_Ramp_Timeout(RAMP_DESCEND, UPDOWN_SPEED_LOW, heading,
                                 basic_p, UPDOWN_SPEED_LOW,
                                 basic_p - 8.0f, UPDOWN_SPEED_LOW,
                                 basic_p - 3.0f, 0.05f,
                                 BARRIER_MOTION_TIMEOUT_MS));
    }
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    return barrier_done(0u, 0u);
}

BarrierResult_t Barrier_DoubleHill(void)
{
    uint8_t hill;
    float heading = getAngleZ();
    BarrierResult_t result;

    for (hill = 0u; hill < 2u; hill++)
    {
        result = barrier_from_action(
            Chassis_Ramp_Timeout(RAMP_ASCEND, UPDOWN_SPEED_LOW, heading,
                                 BEGIN_UP, UPDOWN_SPEED_LOW, UP_PITCH,
                                 UPDOWN_SPEED_LOW, AFTER_UP, 0.05f,
                                 BARRIER_MOTION_TIMEOUT_MS));
        if (result != BARRIER_RESULT_OK)
            return barrier_fail(result);
        result = barrier_from_action(
            Chassis_Ramp_Timeout(RAMP_DESCEND, UPDOWN_SPEED_LOW, heading,
                                 BEGIN_DOWN, UPDOWN_SPEED_LOW, DOWN_PITCH,
                                 UPDOWN_SPEED_LOW, AFTER_DOWN, 0.05f,
                                 BARRIER_MOTION_TIMEOUT_MS));
        if (result != BARRIER_RESULT_OK)
            return barrier_fail(result);
    }
    return barrier_done(0u, 0u);
}

BarrierResult_t Barrier_SwordMountain(void)
{
    float heading;
    BarrierResult_t result;

    line_mode_reset_by_flag(nodesr.nowNode.flag);
    result = barrier_from_action(
        Chassis_DriveDistance_Timeout(is_Line, 10.0f, SPEED0, 0.0f,
                                      BARRIER_MOTION_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    mpuZreset(imu.yaw, nodesr.nowNode.angle);
    heading = getAngleZ();

    result = barrier_from_action(
        Chassis_Ramp_Timeout(RAMP_ASCEND, SPEED0, heading, BEGIN_UP,
                             SPEED0, UP_PITCH, UPDOWN_SPEED_LOW,
                             AFTER_UP, 0.0f, BARRIER_MOTION_TIMEOUT_MS));
    if (result == BARRIER_RESULT_OK)
    {
        result = barrier_from_action(
            Chassis_Ramp_Timeout(RAMP_DESCEND, UPDOWN_SPEED_LOW, heading,
                                 BEGIN_DOWN, UPDOWN_SPEED_LOW, DOWN_PITCH,
                                 SPEED0, AFTER_DOWN, 0.0f,
                                 BARRIER_MOTION_TIMEOUT_MS));
    }
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    return barrier_done(0u, 0u);
}

BarrierResult_t Barrier_View(uint8_t short_marker)
{
    BarrierResult_t result;
    float clear_distance = short_marker ? 14.0f : 12.0f;

    Chassis_SetTargetSpeed(SPEED0);
    Chassis_SetMode(is_Line);
    result = wait_for_infrared(1u, BARRIER_SENSOR_TIMEOUT_MS);
    if (result != BARRIER_RESULT_OK)
        return result;

    result = barrier_from_action(
        Chassis_DriveDistance_Timeout(is_Line, clear_distance, SPEED0, 0.0f,
                                      BARRIER_MOTION_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    CarBrake();

    result = barrier_recognize_scenic();
    if (result != BARRIER_RESULT_OK)
        return result;
    return barrier_done(1u, 0u);
}

BarrierResult_t Barrier_Back(void)
{
    BarrierResult_t result;

    result = barrier_from_action(
        Chassis_DriveDistance_Timeout(is_Gyro, 12.0f, -SPEED0,
                                      getAngleZ(),
                                      BARRIER_MOTION_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    result = barrier_from_action(
        Chassis_TurnTo_Timeout(nodesr.nextNode.angle, getAngleZ(),
                               BARRIER_TURN_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    Chassis_SetTargetSpeed(SPEED25);
    Chassis_SetMode(is_Line);
    return barrier_done(0u, 0u);
}

BarrierResult_t Barrier_SouthPole(void)
{
    BarrierResult_t result;

    result = wait_for_ramp(RAMP_DETECT_HILL, BARRIER_APPROACH_TIMEOUT_MS);
    if (result != BARRIER_RESULT_OK)
        return result;
    result = barrier_from_action(
        Chassis_Ramp_Timeout(RAMP_ASCEND, SPEED0, nodesr.nowNode.angle,
                             BEGIN_UP, SPEED0, UP_PITCH,
                             UPDOWN_SPEED_LOW, AFTER_UP, 0.0f,
                             BARRIER_MOTION_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);

    Rudder_control(170u, 0u);
    result = wait_for_infrared(1u, BARRIER_SENSOR_TIMEOUT_MS);
    if (result != BARRIER_RESULT_OK)
        return result;
    result = barrier_recognize_scenic();
    if (result != BARRIER_RESULT_OK)
        return result;
    result = barrier_from_action(Chassis_Turn180_Timeout(BARRIER_TURN_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);

    Rudder_control(270u, 0u);
    result = barrier_from_action(
        Chassis_Ramp_Timeout(RAMP_DESCEND, UPDOWN_SPEED_LOW, getAngleZ(),
                             BEGIN_DOWN, UPDOWN_SPEED_LOW, DOWN_PITCH,
                             SPEED0, AFTER_DOWN, 0.0f,
                             BARRIER_MOTION_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    line_mode_reset(CENTER_LINE_MODE);
    Chassis_SetTargetSpeed(SPEED0);
    Chassis_SetMode(is_Line);
    return barrier_done(0u, 0u);
}

BarrierResult_t Barrier_Seesaw(void)
{
    float heading = getAngleZ();
    BarrierResult_t result;

    Chassis_SetTargetSpeed(SPEED0);
    Chassis_SetMode(is_Line);
    result = wait_for_pitch_above(BEGIN_UP, BARRIER_APPROACH_TIMEOUT_MS);
    if (result != BARRIER_RESULT_OK)
        return result;

    result = barrier_from_action(
        Chassis_DriveDistance_Timeout(is_Gyro, DISTANCE_SEESAW_CROSS,
                                      UPDOWN_SPEED_LOW, heading,
                                      BARRIER_MOTION_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    result = wait_for_pitch_above(AFTER_DOWN, BARRIER_SENSOR_TIMEOUT_MS);
    if (result != BARRIER_RESULT_OK)
        return result;

    line_mode_reset_by_flag(nodesr.nowNode.flag);
    Chassis_SetTargetSpeed(SPEED0);
    Chassis_SetMode(is_Line);
    return barrier_done(0u, 0u);
}

BarrierResult_t Barrier_Door(uint8_t alternate_camera)
{
    VisionDirection_t direction = alternate_camera
                                ? VISION_DIRECTION_RIGHT
                                : VISION_DIRECTION_CENTER;
    VisionResult_t result;
    VisionStatus_t status;
    uint8_t outbound = (map.routetime == 0u) ? 1u : 0u;

    CarBrake();
    status = Vision_ScanTrafficSign(direction, &result);
    if (status != VISION_STATUS_OK)
        return barrier_fail(BARRIER_RESULT_VISION_FAILED);
    if (!Vision_TrafficAllows((VisionTrafficColor_t)result.value, outbound))
        return barrier_fail(BARRIER_RESULT_BLOCKED);

    Chassis_SetTargetSpeed(nodesr.nowNode.speed);
    Chassis_SetMode(is_Line);
    return barrier_done(0u, 0u);
}

BarrierResult_t Barrier_HighMountain(void)
{
    BarrierResult_t result;
    float heading = nodesr.nowNode.angle;

    result = wait_for_ramp(RAMP_DETECT_HILL, BARRIER_APPROACH_TIMEOUT_MS);
    if (result != BARRIER_RESULT_OK)
        return result;
    result = barrier_from_action(
        Chassis_Ramp_Timeout(RAMP_ASCEND, SPEED0, heading, BEGIN_UP,
                             SPEED0, UP_PITCH, UPDOWN_SPEED_LOW,
                             AFTER_UP, 0.0f, BARRIER_MOTION_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);

    result = barrier_from_action(
        Chassis_DriveDistance_Timeout(is_Line, 80.0f, SPEED0, 0.0f,
                                      BARRIER_MOTION_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    result = wait_for_pitch_above(UP_PITCH, BARRIER_SENSOR_TIMEOUT_MS);
    if (result != BARRIER_RESULT_OK)
        return result;

    Rudder_control(170u, 0u);
    result = wait_for_infrared(1u, BARRIER_SENSOR_TIMEOUT_MS);
    if (result != BARRIER_RESULT_OK)
        return result;
    result = barrier_recognize_scenic();
    if (result != BARRIER_RESULT_OK)
        return result;
    result = barrier_from_action(Chassis_Turn180_Timeout(BARRIER_TURN_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    Rudder_control(270u, 0u);

    result = barrier_from_action(
        Chassis_Ramp_Timeout(RAMP_DESCEND, UPDOWN_SPEED_LOW, getAngleZ(),
                             BEGIN_DOWN, UPDOWN_SPEED_LOW, DOWN_PITCH,
                             SPEED0, AFTER_DOWN, 0.0f,
                             BARRIER_MOTION_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    line_mode_reset(CENTER_LINE_MODE);
    Chassis_SetTargetSpeed(SPEED0);
    Chassis_SetMode(is_Line);
    return barrier_done(0u, 0u);
}

BarrierResult_t Barrier_Under(void)
{
    BarrierResult_t result;

    result = wait_for_pitch_below(basic_p - 3.0f,
                                  BARRIER_APPROACH_TIMEOUT_MS);
    if (result != BARRIER_RESULT_OK)
        return result;
    result = barrier_from_action(
        Chassis_DriveDistance_Timeout(is_Line, 50.0f,
                                      nodesr.nowNode.speed, 0.0f,
                                      BARRIER_MOTION_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    return barrier_done(0u, 0u);
}

BarrierResult_t Barrier_SpecialNode(void)
{
    BarrierResult_t result;

    line_mode_reset_by_flag(nodesr.nowNode.flag);
    result = barrier_from_action(
        Chassis_DriveDistance_Timeout(is_Line, 10.0f,
                                      nodesr.nowNode.speed, 0.0f,
                                      BARRIER_MOTION_TIMEOUT_MS));
    if (result != BARRIER_RESULT_OK)
        return barrier_fail(result);
    return barrier_done(0u, 0u);
}

BarrierResult_t Barrier_Ignore(void)
{
    return barrier_done(0u, 0u);
}
