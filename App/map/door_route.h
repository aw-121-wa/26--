#ifndef __DOOR_ROUTE_H
#define __DOOR_ROUTE_H

#include "sys.h"

typedef enum
{
    DOOR_ID_INVALID = 0,
    DOOR_D1,
    DOOR_D2,
    DOOR_D3,
    DOOR_D4,
    DOOR_D5,
    DOOR_ID_COUNT
} DoorId_t;

typedef enum
{
    LIGHT_UNKNOWN = 0,
    LIGHT_GREEN,
    LIGHT_YELLOW,
    LIGHT_RED
} LightColor_t;

typedef struct
{
    LightColor_t color[DOOR_ID_COUNT];
    DoorId_t current;
    uint8_t detected_count;
} DoorContext_t;

extern DoorContext_t door_ctx;

DoorId_t Door_GetIdByEdge(uint8_t from_node, uint8_t to_node);
DoorId_t Door_GetIdByEdgeBidirectional(uint8_t from_node, uint8_t to_node);
uint8_t Door_IsPassAllowed(DoorId_t door_id);
void Door_RecordPass(DoorId_t door_id);

#endif /* __DOOR_ROUTE_H */
