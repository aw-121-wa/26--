#ifndef ROUTE_BUILDER_H
#define ROUTE_BUILDER_H

#include "sys.h"

#define ROUTE_CAPACITY 100u

typedef enum {
    ROUTE_BUILD_OK = 0,
    ROUTE_BUILD_FULL,
    ROUTE_BUILD_INVALID_NODE,
    ROUTE_BUILD_DISCONNECTED,
    ROUTE_BUILD_MALFORMED,
    ROUTE_BUILD_INVALID_ARG
} RouteBuildStatus_t;

typedef struct {
    uint8_t *storage;
    uint8_t capacity;
    uint8_t length;
} RouteBuilder_t;

void RouteBuilder_Init(RouteBuilder_t *builder, uint8_t *storage, uint8_t capacity);
void RouteBuilder_Clear(RouteBuilder_t *builder);
RouteBuildStatus_t RouteBuilder_AppendNode(RouteBuilder_t *builder, uint8_t node);
RouteBuildStatus_t RouteBuilder_AppendSegment(RouteBuilder_t *builder, const uint8_t *segment);
RouteBuildStatus_t RouteBuilder_AppendShortestPath(RouteBuilder_t *builder,
                                                    uint8_t start_node,
                                                    uint8_t target_node);
RouteBuildStatus_t RouteBuilder_Validate(const RouteBuilder_t *builder, uint8_t start_node);
RouteBuildStatus_t RouteBuilder_Commit(const RouteBuilder_t *builder, uint8_t start_node);

#endif
