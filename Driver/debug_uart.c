#include "debug_uart.h"
#include "chassis_api.h"
#include "scaner.h"
#include "pid.h"
#include "imu.h"
#include "turn.h"
#include "speed_ctrl.h"
#include "motor_task.h"
#include "map.h"
#include "stdio.h"
#include "usart.h"

/* DAPLink virtual COM monitor: one non-blocking frame every 100 ms. */
#define DEBUG_MONITOR_PERIOD_MS 100u
#define DEBUG_MONITOR_BUFFER_SIZE 320u

static char monitor_buffer[DEBUG_MONITOR_BUFFER_SIZE];
static TickType_t last_monitor_tick;

static long scaled_tenth(float value)
{
    return (long)(value * 10.0f);
}

void debug_uart_init(void)
{
    last_monitor_tick = xTaskGetTickCount();
}

void debug_uart_tick(void)
{
    TickType_t now = xTaskGetTickCount();
    float target_speed;
    int length;

    if ((now - last_monitor_tick) < pdMS_TO_TICKS(DEBUG_MONITOR_PERIOD_MS))
        return;

    last_monitor_tick = now;

    /* Never wait for UART here: this runs in the 5 ms motor-control task. */
    if (huart2.gState != HAL_UART_STATE_READY)
        return;

    target_speed = PIDMode == is_Gyro ? motor_all.Gspeed : motor_all.Cspeed;

    length = snprintf(monitor_buffer, sizeof(monitor_buffer),
        "MON,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%04lX,%ld,%u,%u,%04lX,%u,%u,"
        "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
        (unsigned long)now,
        (unsigned int)map.routetime,
        (unsigned int)map.point,
        (unsigned int)nodesr.lastNode.nodenum,
        (unsigned int)nodesr.nowNode.nodenum,
        (unsigned int)nodesr.nextNode.nodenum,
        (unsigned int)nodesr.nowNode.function,
        (unsigned int)PIDMode,
        (unsigned int)Chassis_GetStopReason(),
        (unsigned long)Scaner.detail,
        scaled_tenth(Scaner.error),
        (unsigned int)Scaner.lineNum,
        (unsigned int)Scaner.ledNum,
        (unsigned long)Cross_Scaner.detail,
        (unsigned int)Cross_Scaner.lineNum,
        (unsigned int)Cross_Scaner.ledNum,
        scaled_tenth(getAngleZ()),
        scaled_tenth(imu.pitch),
        scaled_tenth(imu.roll),
        scaled_tenth(angle.AngleG),
        scaled_tenth(gyroG_pid.output),
        scaled_tenth(gyroT_pid.output),
        scaled_tenth(line_pid_obj.output),
        scaled_tenth(target_speed),
        scaled_tenth(motor_all.encoder_avg),
        scaled_tenth(motor_all.Lspeed),
        scaled_tenth(motor_all.Rspeed),
        scaled_tenth(Chassis_GetMileage()));

    if (length <= 0 || (size_t)length >= sizeof(monitor_buffer))
        return;

    /* A busy/error return intentionally drops this sample; control keeps running. */
    (void)HAL_UART_Transmit_IT(&huart2, (uint8_t *)monitor_buffer,
                               (uint16_t)length);
}
