#ifndef ROUTE_CATALOG_H
#define ROUTE_CATALOG_H

#include <stdint.h>
#include "route_builder.h"

#ifdef __cplusplus
extern "C" {
#endif
#define ROUTE_CATALOG_DOOR_COUNT 14u
#define ROUTE_CATALOG_RETURN_COUNT 80u

typedef struct
{
    uint8_t clue_a;
    uint8_t clue_b;
    uint8_t green_gate;
    uint8_t treasure_node;
} MissionRouteInput_t;

const uint8_t *RouteCatalog_GetDoor(uint8_t route_number);
const uint8_t *RouteCatalog_GetReturn(uint8_t route_number);
const uint8_t *RouteCatalog_SelectReturn(const MissionRouteInput_t *input);
RouteBuildStatus_t RouteCatalog_AppendDoor(RouteBuilder_t *builder, uint8_t route_number);
RouteBuildStatus_t RouteCatalog_AppendReturn(RouteBuilder_t *builder,
                                              const MissionRouteInput_t *input);
uint8_t RouteCatalog_ValidateAll(void);

#ifdef __cplusplus
}
#endif

#endif
