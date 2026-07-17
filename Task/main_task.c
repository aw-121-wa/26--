#include "main_task.h"

#include "map.h"
#include "route_builder.h"
#include "route_catalog.h"
#include "chassis_api.h"
#include "vision_api.h"
#include "encoder.h"
#include "imu.h"
#include "debug_snapshot.h"
#include <string.h>

#define MATCH_CONTROL_PERIOD_MS 5u

static volatile MatchState_t match_state = MATCH_INIT;
static volatile uint8_t reset_requested;
static BarrierMissionData_t match_mission;

static uint8_t treasure_value_to_node(uint8_t value)
{
    switch (value)
    {
    case 1u: return P1;
    case 3u: return P3;
    case 4u: return P4;
    case 5u: return P6;  /* legacy clue sum 5 */
    case 6u: return P5;  /* legacy clue sum 6 */
    default: return value;
    }
}

static uint8_t match_read_vision(VisionMode_t mode, VisionResult_t *result)
{
    if (Vision_Request(mode, VISION_DIRECTION_CENTER) != VISION_STATUS_OK ||
        Vision_WaitResult(result, VISION_RESULT_TIMEOUT_MS) != VISION_STATUS_OK ||
        result->mode != mode || result->confidence < VISION_MIN_CONFIDENCE)
    {
        Chassis_ForceStop(CHASSIS_STOP_VISION_TIMEOUT);
        return 0u;
    }
    return 1u;
}

static uint8_t match_collect_mission(void)
{
    VisionResult_t result;

    Barrier_GetMissionData(&match_mission);
    if (match_mission.clue_a == 0u)
    {
        if (!match_read_vision(VISION_MODE_CLUE, &result)) return 0u;
        match_mission.clue_a = result.value;
    }
    if (match_mission.clue_b == 0u)
    {
        if (!match_read_vision(VISION_MODE_CLUE, &result)) return 0u;
        match_mission.clue_b = result.value;
    }
    if (match_mission.treasure_node == 0u)
    {
        if (!match_read_vision(VISION_MODE_TREASURE, &result)) return 0u;
        match_mission.treasure_node = treasure_value_to_node(result.value);
    }
    match_mission.valid = 1u;
    Barrier_SetMissionData(&match_mission);
    Barrier_GetMissionData(&match_mission);
    return 1u;
}

void Match_SetMissionData(const BarrierMissionData_t *data)
{
    if (data == NULL)
        return;
    match_mission = *data;
    match_mission.valid = 1u;
    Barrier_SetMissionData(&match_mission);
    Barrier_GetMissionData(&match_mission);
}

void Match_RequestReset(void)
{
    memset(&match_mission, 0, sizeof(match_mission));
    Barrier_ClearMissionData();
    Vision_ClearResults();
    reset_requested = 1u;
}

MatchState_t Match_GetState(void)
{
    return match_state;
}

static uint8_t match_build_round2_route(void)
{
    RouteBuilder_t builder;
    MissionRouteInput_t input;
    const uint8_t *return_segment;
    uint8_t storage[ROUTE_CAPACITY];
    uint8_t green_count = 0u;

    if (!match_mission.valid)
    {
        Chassis_ForceStop(CHASSIS_STOP_VISION_TIMEOUT);
        return 0u;
    }

    input.clue_a = match_mission.clue_a;
    input.clue_b = match_mission.clue_b;
    input.green_gate = 0u;
    input.treasure_node = match_mission.treasure_node;
    for (uint8_t i = 0u; i < 4u; i++)
        if (match_mission.gate_color[i] == VISION_COLOR_GREEN)
        {
            input.green_gate = (uint8_t)(i + 1u);
            green_count++;
        }

    if (green_count != 1u)
    {
        Chassis_ForceStop(CHASSIS_STOP_VISION_TIMEOUT);
        return 0u;
    }

    return_segment = RouteCatalog_SelectReturn(&input);
    if (return_segment == NULL)
    {
        Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
        return 0u;
    }

    RouteBuilder_Init(&builder, storage, ROUTE_CAPACITY);
    if (RouteBuilder_AppendShortestPath(&builder, nodesr.nowNode.nodenum,
                                        return_segment[0]) != ROUTE_BUILD_OK ||
        RouteBuilder_AppendSegment(&builder, &return_segment[1]) != ROUTE_BUILD_OK ||
        RouteBuilder_Commit(&builder, nodesr.nowNode.nodenum) != ROUTE_BUILD_OK)
        return 0u;
    return 1u;
}

void main_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    (void)pvParameters;

    for (;;)
    {
        Vision_Poll();
        DebugSnapshot_Update();

        if (Chassis_IsStopLocked() && match_state != MATCH_FAULT)
            match_state = MATCH_FAULT;

        switch (match_state)
        {
        case MATCH_INIT:
            reset_requested = 0u;
            Chassis_ClearStopLock();
            match_state = ROUND1_PREPARE;
            break;

        case ROUND1_PREPARE:
            mapInit();
            if (!Chassis_IsStopLocked())
            {
                zhunbei();
                encoder_clear();
            }
            match_state = Chassis_IsStopLocked() ? MATCH_FAULT : ROUND1_RUNNING;
            break;

        case ROUND1_RUNNING:
            Cross();
            if (map.routetime >= 1u)
                match_state = ROUND1_FINISH;
            break;

        case ROUND1_FINISH:
            CarBrake();
            match_state = match_collect_mission() ? ROUND2_ROUTE_BUILD : MATCH_FAULT;
            break;

        case ROUND2_ROUTE_BUILD:
            match_state = match_build_round2_route() ? ROUND2_PREPARE : MATCH_FAULT;
            break;

        case ROUND2_PREPARE:
            mapInit1();
            encoder_clear();
            match_state = Chassis_IsStopLocked() ? MATCH_FAULT : ROUND2_RUNNING;
            break;

        case ROUND2_RUNNING:
            Cross();
            if (map.routetime >= 2u)
                match_state = RETURN_HOME;
            break;

        case RETURN_HOME:
            CarBrake();
            match_state = MATCH_FINISH;
            break;

        case MATCH_FINISH:
            CarBrake();
            if (reset_requested)
                match_state = MATCH_INIT;
            break;

        case MATCH_FAULT:
            CarBrake();
            (void)Chassis_GetStopReason();
            if (reset_requested)
            {
                Chassis_ClearStopLock();
                match_state = MATCH_INIT;
            }
            break;

        default:
            Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
            match_state = MATCH_FAULT;
            break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MATCH_CONTROL_PERIOD_MS));
    }
}
