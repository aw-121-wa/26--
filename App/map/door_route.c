#include "door_route.h"
#include "map.h"

typedef struct
{
    uint8_t from_node;
    uint8_t to_node;
    DoorId_t door_id;
} DoorEdgeMap_t;

DoorContext_t door_ctx =
{
    {LIGHT_UNKNOWN},
    DOOR_ID_INVALID,
    0
};

static const DoorEdgeMap_t door_edge_map[] =
{
    {C1, N13, DOOR_D1},
    {N5, N12, DOOR_D2},
    {N5, N8,  DOOR_D3},
    {N3, N8,  DOOR_D4},
    {N3, N10, DOOR_D5}
};

DoorId_t Door_GetIdByEdge(uint8_t from_node, uint8_t to_node)
{
    uint32_t i;
    uint32_t count = sizeof(door_edge_map) / sizeof(door_edge_map[0]);

    for (i = 0; i < count; i++)
    {
        if (door_edge_map[i].from_node == from_node &&
            door_edge_map[i].to_node == to_node)
        {
            return door_edge_map[i].door_id;
        }
    }

    return DOOR_ID_INVALID;
}

DoorId_t Door_GetIdByEdgeBidirectional(uint8_t from_node, uint8_t to_node)
{
    DoorId_t id = Door_GetIdByEdge(from_node, to_node);

    if (id != DOOR_ID_INVALID)
        return id;

    return Door_GetIdByEdge(to_node, from_node);
}

uint8_t Door_IsPassAllowed(DoorId_t door_id)
{
    switch (door_id)
    {
    case DOOR_D3:
        return 1;

    case DOOR_D1:
    case DOOR_D2:
    case DOOR_D4:
    case DOOR_D5:
    default:
        return 0;
    }
}

void Door_RecordPass(DoorId_t door_id)
{
    if (door_id <= DOOR_ID_INVALID || door_id >= DOOR_ID_COUNT)
        return;

    door_ctx.current = door_id;
    door_ctx.detected_count++;
}
