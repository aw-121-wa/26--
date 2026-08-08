#ifndef MAIN_TASK_H
#define MAIN_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "task_create.h"
#include "temporary_task.h"
#include "route_catalog.h"

#define LINE_DEBUG_MODE 0
#define MATCH_ROUND_TIMEOUT_MS 150000u

typedef enum {
    MATCH_INIT = 0,
    ROUND1_PREPARE,
    ROUND1_RUNNING,
    ROUND1_FINISH,
    ROUND2_ROUTE_BUILD,
    ROUND2_PREPARE,
    ROUND2_RUNNING,
    RETURN_HOME,
    MATCH_FINISH,
    MATCH_FAULT
} MatchState_t;

void Match_SetMissionData(const MissionRouteInput_t *data);
void Match_ClearMissionData(void);
MatchState_t Match_GetState(void);
void main_task(void *pvParameters);

#endif
