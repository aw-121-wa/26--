#include "route_builder.h"

#include "chassis_api.h"
#include "map.h"
#include <string.h>

void RouteBuilder_Init(RouteBuilder_t *builder, uint8_t *storage, uint8_t capacity)
{
    if (builder == NULL)
        return;
    builder->storage = storage;
    builder->capacity = capacity;
    builder->length = 0;
    if (storage != NULL && capacity != 0u)
        memset(storage, ROUTE_END, capacity);
}

void RouteBuilder_Clear(RouteBuilder_t *builder)
{
    if (builder == NULL)
        return;
    builder->length = 0;
    if (builder->storage != NULL && builder->capacity != 0u)
        memset(builder->storage, ROUTE_END, builder->capacity);
}

RouteBuildStatus_t RouteBuilder_AppendNode(RouteBuilder_t *builder, uint8_t node)
{
    if (builder == NULL || builder->storage == NULL || builder->capacity < 2u)
        return ROUTE_BUILD_INVALID_ARG;
    if (node >= MAP_NODE_COUNT)
        return ROUTE_BUILD_INVALID_NODE;
    if (builder->length >= (uint8_t)(builder->capacity - 1u))
        return ROUTE_BUILD_FULL;
    builder->storage[builder->length++] = node;
    builder->storage[builder->length] = ROUTE_END;
    return ROUTE_BUILD_OK;
}

RouteBuildStatus_t RouteBuilder_AppendSegment(RouteBuilder_t *builder, const uint8_t *segment)
{
    uint8_t i;
    RouteBuildStatus_t status;

    if (builder == NULL || segment == NULL)
        return ROUTE_BUILD_INVALID_ARG;

    for (i = 0; i < ROUTE_CAPACITY; i++)
    {
        if (segment[i] == ROUTE_END)
            return ROUTE_BUILD_OK;
        status = RouteBuilder_AppendNode(builder, segment[i]);
        if (status != ROUTE_BUILD_OK)
            return status;
    }
    return ROUTE_BUILD_MALFORMED;
}

RouteBuildStatus_t RouteBuilder_AppendShortestPath(RouteBuilder_t *builder,
                                                    uint8_t start_node,
                                                    uint8_t target_node)
{
    uint8_t queue[MAP_NODE_COUNT];
    uint8_t previous[MAP_NODE_COUNT];
    uint8_t reverse_path[MAP_NODE_COUNT];
    uint8_t head = 0u;
    uint8_t tail = 0u;
    uint8_t path_length = 0u;
    uint8_t node;
    uint8_t i;

    if (builder == NULL || start_node >= MAP_NODE_COUNT || target_node >= MAP_NODE_COUNT)
        return ROUTE_BUILD_INVALID_ARG;
    if (start_node == target_node)
        return ROUTE_BUILD_OK;

    memset(previous, MAP_NODE_INDEX_INVALID, sizeof(previous));
    previous[start_node] = start_node;
    queue[tail++] = start_node;

    while (head < tail && previous[target_node] == MAP_NODE_INDEX_INVALID)
    {
        node = queue[head++];
        for (i = 0u; i < ConnectionNum[node]; i++)
        {
            uint8_t next = Node[Address[node] + i].nodenum;
            if (previous[next] == MAP_NODE_INDEX_INVALID)
            {
                previous[next] = node;
                queue[tail++] = next;
            }
        }
    }
    if (previous[target_node] == MAP_NODE_INDEX_INVALID)
        return ROUTE_BUILD_DISCONNECTED;

    node = target_node;
    while (node != start_node && path_length < MAP_NODE_COUNT)
    {
        reverse_path[path_length++] = node;
        node = previous[node];
    }
    while (path_length > 0u)
    {
        RouteBuildStatus_t status = RouteBuilder_AppendNode(builder,
                                                             reverse_path[--path_length]);
        if (status != ROUTE_BUILD_OK)
            return status;
    }
    return ROUTE_BUILD_OK;
}

RouteBuildStatus_t RouteBuilder_Validate(const RouteBuilder_t *builder, uint8_t start_node)
{
    uint8_t current;
    uint8_t i;

    if (builder == NULL || builder->storage == NULL || builder->capacity < 2u ||
        builder->length == 0u || builder->length >= builder->capacity ||
        start_node >= MAP_NODE_COUNT)
        return ROUTE_BUILD_INVALID_ARG;

    current = start_node;
    for (i = 0; i < builder->length; i++)
    {
        if (builder->storage[i] >= MAP_NODE_COUNT)
            return ROUTE_BUILD_INVALID_NODE;
        if (getNextConnectNode(current, builder->storage[i]) == MAP_NODE_INDEX_INVALID)
            return ROUTE_BUILD_DISCONNECTED;
        current = builder->storage[i];
    }
    if (builder->storage[builder->length] != ROUTE_END)
        return ROUTE_BUILD_MALFORMED;
    return ROUTE_BUILD_OK;
}

RouteBuildStatus_t RouteBuilder_Commit(const RouteBuilder_t *builder, uint8_t start_node)
{
    RouteBuildStatus_t status = RouteBuilder_Validate(builder, start_node);

    if (status != ROUTE_BUILD_OK)
    {
        Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
        return status;
    }
    memset(route, ROUTE_END, ROUTE_CAPACITY);
    memcpy(route, builder->storage, builder->length);
    route[builder->length] = ROUTE_END;
    map.point = 0;
    return ROUTE_BUILD_OK;
}
