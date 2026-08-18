#ifndef ROUTE_CATALOG_H
#define ROUTE_CATALOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#define ROUTE_CATALOG_DOOR_COUNT 14u

const uint8_t *RouteCatalog_GetDoor(uint8_t route_number);

#ifdef __cplusplus
}
#endif

#endif
