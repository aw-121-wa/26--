#include "debug_snapshot.h"

#include "map.h"
#include "motor_task.h"
#include "speed_ctrl.h"
#include "scaner.h"
#include "imu.h"

#define DEBUG_SNAPSHOT_PERIOD_MS 100u

static DebugSnapshot_t snapshot;
static TickType_t last_update;

void DebugSnapshot_Update(void)
{
    TickType_t now = xTaskGetTickCount();
    const VisionDiagnostics_t *vision;

    if ((now - last_update) < pdMS_TO_TICKS(DEBUG_SNAPSHOT_PERIOD_MS))
        return;
    last_update = now;
    vision = Vision_GetDiagnostics();

    snapshot.match_state = Match_GetState();
    snapshot.round = map.routetime < 1u ? 1u : 2u;
    snapshot.route_index = map.point;
    snapshot.last_node = nodesr.lastNode.nodenum;
    snapshot.current_node = nodesr.nowNode.nodenum;
    snapshot.next_node = nodesr.nextNode.nodenum;
    snapshot.cross_state = Cross_GetState();
    snapshot.barrier_type = nodesr.nowNode.function;
    snapshot.chassis_mode = PIDMode;
    snapshot.target_speed = PIDMode == is_Gyro ? motor_all.Gspeed : motor_all.Cspeed;
    snapshot.actual_speed = motor_all.encoder_avg;
    snapshot.mileage = Chassis_GetMileage();
    snapshot.yaw = imu.yaw;
    snapshot.pitch = imu.pitch;
    snapshot.roll = imu.roll;
    snapshot.line_detail = Scaner.detail;
    snapshot.line_count = Scaner.lineNum;
    snapshot.led_count = Scaner.ledNum;
    snapshot.vision_status = vision->last_status;
    snapshot.vision_sequence = vision->last_sequence;
    snapshot.stop_reason = Chassis_GetStopReason();
}

const DebugSnapshot_t *DebugSnapshot_Get(void)
{
    return &snapshot;
}
