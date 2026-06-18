#include "debug_uart.h"
#include "scaner.h"
#include "pid.h"
#include "imu.h"
#include "turn.h"
#include "speed_ctrl.h"
#include "motor_task.h"
#include "map.h"
#include "stdio.h"
#include "chassis_api.h"

#define DBG_ERR     0
#define DBG_L0      0
#define DBG_L1      0
#define DBG_R0      0
#define DBG_R1      0
#define DBG_YAW     0
#define DBG_PITCH   0
#define DBG_LINEPID 0
#define DBG_GYROPID 0
#define DBG_TURNPID 0
#define DBG_CSPD    0
#define DBG_GSPD    0
#define DBG_LSPD    0
#define DBG_RSPD    0
#define DBG_DIST    0   
#define DBG_NODE    0
#define DBG_MODE    0

void debug_uart_init(void)
{
}

void debug_uart_tick(void)
{
    static uint8_t cnt = 0;
    if (++cnt < 10)
        return;
    cnt = 0;

#if DBG_ERR
    printf("err:%.2f\n", (double)Scaner.error);
#endif
#if DBG_L0
    printf("L0:%.2f\n", (double)motor_L0.measure);
#endif
#if DBG_L1
    printf("L1:%.2f\n", (double)motor_L1.measure);
#endif
#if DBG_R0
    printf("R0:%.2f\n", (double)motor_R0.measure);
#endif
#if DBG_R1
    printf("R1:%.2f\n", (double)motor_R1.measure);
#endif
#if DBG_YAW
    printf("yaw:%.2f tgt:%.2f\n", (double)getAngleZ(), (double)angle.AngleG);
#endif
#if DBG_PITCH
    printf("pitch:%.2f\n", (double)imu.pitch);
#endif
#if DBG_LINEPID
    printf("linePID:%.2f\n", (double)line_pid_obj.output);
#endif
#if DBG_GYROPID
    printf("gyroPID:%.2f\n", (double)gyroG_pid.output);
#endif
#if DBG_TURNPID
    printf("turnPID:%.2f\n", (double)gyroT_pid.output);
#endif
#if DBG_CSPD
    printf("Cspd:%.2f\n", (double)motor_all.Cspeed);
#endif
#if DBG_GSPD
    printf("Gspd:%.2f\n", (double)motor_all.Gspeed);
#endif
#if DBG_LSPD
    printf("Lspd:%.2f\n", (double)motor_all.Lspeed);
#endif
#if DBG_RSPD
    printf("Rspd:%.2f\n", (double)motor_all.Rspeed);
#endif
#if DBG_DIST
    printf("dist:%.2f\n", (double)motor_all.Distance);
#endif
#if DBG_NODE
    printf("node:%.0f\n", (double)nodesr.nowNode.nodenum);
#endif
#if DBG_MODE
    printf("mode:%.0f\n", (double)PIDMode);
#endif
}
