#ifndef DEBUG_SNAPSHOT_H
#define DEBUG_SNAPSHOT_H

#include "sys.h"
#include "main_task.h"
#include "chassis_api.h"
#include "vision_api.h"

typedef struct {
    MatchState_t match_state;
    uint8_t round;
    uint8_t route_index;
    uint8_t last_node;
    uint8_t current_node;
    uint8_t next_node;
    uint8_t cross_state;
    uint8_t barrier_type;
    uint8_t chassis_mode;
    float target_speed;
    float actual_speed;
    float mileage;
    float yaw;
    float pitch;
    float roll;
    uint16_t line_detail;
    uint8_t line_count;
    uint8_t led_count;
    VisionStatus_t vision_status;
    uint8_t vision_sequence;
    Chassis_StopReason_t stop_reason;
} DebugSnapshot_t;

void DebugSnapshot_Update(void);
const DebugSnapshot_t *DebugSnapshot_Get(void);

#endif
