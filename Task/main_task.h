#ifndef MAIN_TASK_H
#define MAIN_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "task_create.h"
#include "temporary_task.h"
#include "barrier.h"

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

void main_task(void *pvParameters);
void Match_SetMissionData(const BarrierMissionData_t *data);
void Match_RequestReset(void);
MatchState_t Match_GetState(void);

#endif
