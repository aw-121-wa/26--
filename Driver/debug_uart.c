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
#include "string.h"
#include "usart.h"

#define DBG_ERR     0
#define DBG_L0      0
#define DBG_L1      0
#define DBG_R0      0
#define DBG_R1      0
#define DBG_YAW     0
#define DBG_PITCH   0
#define DBG_ROLL    0
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
#define DBG_STAGE_TURN 0
#define DBG_ARRIVE   0

#define DBG_BUF_SIZE 128

/* 节点编号 → 名称（与 map.h enum MapNode 顺序一致） */
static const char *node_name(uint8_t n)
{
    static const char *const names[] = {
        [0]  = "S1",  [1]  = "P1",  [2]  = "N1",  [3]  = "B1",
        [4]  = "B2",  [5]  = "B3",  [6]  = "N2",  [7]  = "P2",
        [8]  = "S2",  [9]  = "P3",  [10] = "N3",  [11] = "N4",
        [12] = "N5",  [13] = "N6",  [14] = "P4",  [15] = "N7",
        [16] = "P5",  [17] = "B8",  [18] = "B9",  [19] = "N8",
        [20] = "C1",  [21] = "C2",  [22] = "C3",  [23] = "N9",
        [24] = "N10", [25] = "N12", [26] = "N13", [27] = "P6",
        [28] = "N14", [29] = "S3",  [30] = "S4",  [31] = "N15",
        [32] = "S5",  [33] = "C4",  [34] = "C5",  [35] = "B4",
        [36] = "B5",  [37] = "B6",  [38] = "B7",  [39] = "N16",
        [40] = "N18", [41] = "N19", [42] = "P7",  [43] = "N20",
        [44] = "N22", [45] = "C6",  [46] = "C7",  [47] = "C8",
        [48] = "C9",  [49] = "P8",  [50] = "N11", [51] = "C10",
    };
    return (n < 52u) ? names[n] : "??";
}

static void dbg_send(const char *s)
{
    extern UART_HandleTypeDef huart2;
    HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), 0xffff);
}

void debug_uart_init(void)
{
    dbg_send("=== IMU Debug Start ===\r\n");
}

void debug_uart_tick(void)
{
    static uint8_t cnt = 0;
    char buf[DBG_BUF_SIZE];

    if (Chassis_IsStopLocked())
        return;

    if (++cnt < 20)
        return;
    cnt = 0;

#if DBG_STAGE_TURN
    if (StageTurn_Flag)
    {
        snprintf(buf, DBG_BUF_SIZE,
                 "T,%d,%d,%d,%d,%d,%d,%d\r\n",
                 (int)(getAngleZ() * 10.0f),
                 (int)(gyroT_pid.measure * 10.0f),
                 (int)(gyroT_pid.output * 10.0f),
                 (int)(motor_all.Lspeed * 10.0f),
                 (int)(motor_all.Rspeed * 10.0f),
                 (int)(motor_L0.measure * 10.0f),
                 (int)(motor_R0.measure * 10.0f));
        dbg_send(buf);
    }
#endif

#if DBG_ERR
    snprintf(buf, DBG_BUF_SIZE, "err:%.2f\r\n", (double)Scaner.error);
    dbg_send(buf);
#endif
#if DBG_L0
    snprintf(buf, DBG_BUF_SIZE, "L0:%.2f\r\n", (double)motor_L0.measure);
    dbg_send(buf);
#endif
#if DBG_L1
    snprintf(buf, DBG_BUF_SIZE, "L1:%.2f\r\n", (double)motor_L1.measure);
    dbg_send(buf);
#endif
#if DBG_R0
    snprintf(buf, DBG_BUF_SIZE, "R0:%.2f\r\n", (double)motor_R0.measure);
    dbg_send(buf);
#endif
#if DBG_R1
    snprintf(buf, DBG_BUF_SIZE, "R1:%.2f\r\n", (double)motor_R1.measure);
    dbg_send(buf);
#endif
#if DBG_YAW
    snprintf(buf, DBG_BUF_SIZE, "yaw:%.2f tgt:%.2f\r\n", (double)getAngleZ(), (double)angle.AngleG);
    dbg_send(buf);
#endif
#if DBG_PITCH
    snprintf(buf, DBG_BUF_SIZE, "pitch:%.2f\r\n", (double)imu.pitch);
    dbg_send(buf);
#endif
#if DBG_ROLL
    snprintf(buf, DBG_BUF_SIZE, "roll:%.2f\r\n", (double)imu.roll);
    dbg_send(buf);
#endif
#if DBG_LINEPID
    snprintf(buf, DBG_BUF_SIZE, "linePID:%.2f\r\n", (double)line_pid_obj.output);
    dbg_send(buf);
#endif
#if DBG_GYROPID
    snprintf(buf, DBG_BUF_SIZE, "gyroPID:%.2f\r\n", (double)gyroG_pid.output);
    dbg_send(buf);
#endif
#if DBG_TURNPID
    snprintf(buf, DBG_BUF_SIZE, "turnPID:%.2f\r\n", (double)gyroT_pid.output);
    dbg_send(buf);
#endif
#if DBG_CSPD
    snprintf(buf, DBG_BUF_SIZE, "Cspd:%.2f\r\n", (double)motor_all.Cspeed);
    dbg_send(buf);
#endif
#if DBG_GSPD
    snprintf(buf, DBG_BUF_SIZE, "Gspd:%.2f\r\n", (double)motor_all.Gspeed);
    dbg_send(buf);
#endif
#if DBG_LSPD
    snprintf(buf, DBG_BUF_SIZE, "Lspd:%.2f\r\n", (double)motor_all.Lspeed);
    dbg_send(buf);
#endif
#if DBG_RSPD
    snprintf(buf, DBG_BUF_SIZE, "Rspd:%.2f\r\n", (double)motor_all.Rspeed);
    dbg_send(buf);
#endif
#if DBG_DIST
    snprintf(buf, DBG_BUF_SIZE, "dist:%.2f\r\n", (double)motor_all.Distance);
    dbg_send(buf);
#endif
#if DBG_NODE
    snprintf(buf, DBG_BUF_SIZE, "node:%.0f\r\n", (double)nodesr.nowNode.nodenum);
    dbg_send(buf);
#endif
#if DBG_MODE
    snprintf(buf, DBG_BUF_SIZE, "mode:%.0f\r\n", (double)PIDMode);
    dbg_send(buf);
#endif
#if DBG_ARRIVE
    {
        static uint8_t last_arrive_node = 0xFF;
        uint8_t cur = g_last_arrived_node;
        if (cur != last_arrive_node)
        {
            last_arrive_node = cur;
            snprintf(buf, DBG_BUF_SIZE, "reach %s\r\n", node_name(cur));
            dbg_send(buf);
        }
    }
#endif
}
