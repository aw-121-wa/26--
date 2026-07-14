#include "barrier_door.h"
#include "../map/map.h"
#include "../map/door_route.h"
#include "../chassis/chassis_api.h"
#include "../chassis/line_sensor_api.h"
#include "FreeRTOS.h"
#include "task.h"

#define DOOR_STOP_LED_NUM         8u
#define DOOR_APPROACH_TIMEOUT_MS  5000u
#define DOOR_WAIT_PERIOD_MS       5u
#define DOOR_STOP_STABLE_MS       200u
#define DOOR_NODE_ARRIVED_FLAG    0x04u

static uint8_t Door_WaitStopLine(uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while (LineSensor_GetMainLedNum() < DOOR_STOP_LED_NUM)
    {
        LineSensor_UpdateMain();

        if ((xTaskGetTickCount() - start) >= timeout_ticks)
            return 0;

        if (Chassis_IsStopLocked())
            return 0;

        vTaskDelay(pdMS_TO_TICKS(DOOR_WAIT_PERIOD_MS));
    }

    return 1;
}

uint8_t Barrier_Door(void)
{
    DoorId_t door_id;

    door_id = Door_GetIdByEdge(nodesr.lastNode.nodenum, nodesr.nowNode.nodenum);
    door_ctx.current = door_id;

    Chassis_SetTargetSpeed(nodesr.nowNode.speed);
    Chassis_SetLineMode();

    if (door_id == DOOR_ID_INVALID)
    {
        CarBrake();
        return 0;
    }

    if (!Door_WaitStopLine(DOOR_APPROACH_TIMEOUT_MS))
    {
        CarBrake();
        return 0;
    }

    CarBrake();
    vTaskDelay(pdMS_TO_TICKS(DOOR_STOP_STABLE_MS));

    if (!Door_IsPassAllowed(door_id))
        return 0;

    Door_RecordPass(door_id);
    nodesr.nowNode.function = NONE;
    nodesr.flag |= DOOR_NODE_ARRIVED_FLAG;
    Chassis_SetTargetSpeed(nodesr.nowNode.speed);
    Chassis_SetLineMode();

    return 1;
}
