/**
 * @file main_task.c
 * @brief 探险赛双轮比赛状态机。
 */

#include "main_task.h"
#include "../App/map/map.h"
#include "../App/map/route_builder.h"
#include "../App/map/route_catalog.h"
#include "../App/barrier/barrier.h"
#include "../App/chassis/chassis_api.h"
#include "../App/vision/vision_api.h"
#include "motor_task.h"
#include "encoder.h"
#include <string.h>

static MatchState_t match_state = MATCH_INIT;
static MissionRouteInput_t mission_data;
static uint8_t mission_data_valid;
static TickType_t round_started;
static RouteBuilder_t round2_builder;
static uint8_t round2_storage[ROUTE_CAPACITY];

void Match_SetMissionData(const MissionRouteInput_t *data)
{
    if (data == NULL)
    {
        Match_ClearMissionData();
        return;
    }

    mission_data = *data;
    mission_data_valid = 1u;
}

void Match_ClearMissionData(void)
{
    memset(&mission_data, 0, sizeof(mission_data));
    mission_data_valid = 0u;
}

MatchState_t Match_GetState(void)
{
    return match_state;
}

static uint8_t match_faulted(void)
{
    return Chassis_GetStopReason() != CHASSIS_STOP_NONE ? 1u : 0u;
}

static uint8_t match_round_timed_out(void)
{
    return ((TickType_t)(xTaskGetTickCount() - round_started) >=
            pdMS_TO_TICKS(MATCH_ROUND_TIMEOUT_MS)) ? 1u : 0u;
}

static RouteBuildStatus_t match_append_selected_return(void)
{
    const uint8_t *selected = RouteCatalog_SelectReturn(&mission_data);
    RouteBuildStatus_t status;
    uint8_t current = nodesr.nowNode.nodenum;

    if (selected == NULL || selected[0] == ROUTE_END)
        return ROUTE_BUILD_INVALID_ARG;

    if (selected[0] != current)
    {
        status = RouteBuilder_AppendShortestPath(&round2_builder,
                                                 current, selected[0]);
        if (status != ROUTE_BUILD_OK)
            return status;
        selected++;
    }

    return RouteBuilder_AppendSegment(&round2_builder, selected);
}

static RouteBuildStatus_t match_build_round2_route(void)
{
    RouteBuildStatus_t status;
    uint8_t start_node = nodesr.nowNode.nodenum;

    RouteBuilder_Init(&round2_builder, round2_storage, ROUTE_CAPACITY);
    if (mission_data_valid)
        status = match_append_selected_return();
    else
        status = RouteBuilder_AppendSegment(&round2_builder, route);

    if (status != ROUTE_BUILD_OK)
        return status;
    return RouteBuilder_Commit(&round2_builder, start_node);
}

static void match_enter_fault(Chassis_StopReason_t fallback_reason)
{
    if (!Chassis_IsStopLocked())
        Chassis_ForceStop(fallback_reason);
    match_state = MATCH_FAULT;
}

void main_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    BarrierResult_t barrier_result;

    (void)pvParameters;
    for (;;)
    {
        switch (match_state)
        {
        case MATCH_INIT:
            if (!Map_ValidateData() || !RouteCatalog_ValidateAll())
            {
                match_enter_fault(CHASSIS_STOP_ROUTE_INVALID);
                break;
            }
            if (Vision_Init() != VISION_STATUS_OK)
            {
                match_enter_fault(CHASSIS_STOP_VISION_TIMEOUT);
                break;
            }
            match_state = ROUND1_PREPARE;
            break;

        case ROUND1_PREPARE:
            Chassis_ClearStopLock();
            mapInit();
            if (match_faulted())
            {
                match_state = MATCH_FAULT;
                break;
            }
            barrier_result = zhunbei();
            if (barrier_result != BARRIER_RESULT_OK)
            {
                match_enter_fault(CHASSIS_STOP_BARRIER_FAILED);
                break;
            }
            encoder_clear();
            round_started = xTaskGetTickCount();
            match_state = ROUND1_RUNNING;
            break;

        case ROUND1_RUNNING:
            Cross();
            if (match_faulted())
                match_state = MATCH_FAULT;
            else if (match_round_timed_out())
                match_enter_fault(CHASSIS_STOP_MOTION_TIMEOUT);
            else if (map.routetime >= 1u)
                match_state = ROUND1_FINISH;
            break;

        case ROUND1_FINISH:
            CarBrake();
            match_state = ROUND2_ROUTE_BUILD;
            break;

        case ROUND2_ROUTE_BUILD:
            if (match_build_round2_route() != ROUTE_BUILD_OK)
            {
                match_enter_fault(CHASSIS_STOP_ROUTE_INVALID);
                break;
            }
            match_state = ROUND2_PREPARE;
            break;

        case ROUND2_PREPARE:
            mapInit1();
            if (match_faulted())
            {
                match_state = MATCH_FAULT;
                break;
            }
            barrier_result = zhunbei();
            if (barrier_result != BARRIER_RESULT_OK)
            {
                match_enter_fault(CHASSIS_STOP_BARRIER_FAILED);
                break;
            }
            encoder_clear();
            round_started = xTaskGetTickCount();
            match_state = ROUND2_RUNNING;
            break;

        case ROUND2_RUNNING:
            Cross();
            if (match_faulted())
                match_state = MATCH_FAULT;
            else if (match_round_timed_out())
                match_enter_fault(CHASSIS_STOP_MOTION_TIMEOUT);
            else if (map.routetime >= 2u)
                match_state = RETURN_HOME;
            break;

        case RETURN_HOME:
            CarBrake();
            if (nodesr.nowNode.nodenum != P2)
                match_enter_fault(CHASSIS_STOP_ROUTE_INVALID);
            else
                match_state = MATCH_FINISH;
            break;

        case MATCH_FINISH:
            CarBrake();
            break;

        case MATCH_FAULT:
            CarBrake();
            break;

        default:
            match_enter_fault(CHASSIS_STOP_ROUTE_INVALID);
            break;
        }

        Vision_Poll();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5u));
    }
}
