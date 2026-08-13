#ifndef LSC16_ACTION_H
#define LSC16_ACTION_H

#include "main.h"

typedef enum {
    LSC16_ACTION_INIT_LIE_DOWN = 0u,
    LSC16_ACTION_STAND_WAVE_LIE_DOWN = 1u,
    LSC16_ACTION_STAND_UP = 2u,
    LSC16_ACTION_CAMERA_RIGHT = 3u,
    LSC16_ACTION_CAMERA_LEFT = 4u
} Lsc16ActionGroup_t;

#define LSC16_ACTION_BARRIER_DETECTED LSC16_ACTION_INIT_LIE_DOWN
#define LSC16_ACTION_TURN_DONE        LSC16_ACTION_STAND_WAVE_LIE_DOWN
#define LSC16_ACTION_CAMERA_CENTER    LSC16_ACTION_STAND_UP

#define LSC16_ACTION_RUN_ONCE        1u
#define LSC16_WAIT_INIT_MS           1000u
#define LSC16_WAIT_STAND_MS          2500u
#define LSC16_WAIT_PLATFORM_MS       6000u
#define LSC16_WAIT_CAMERA_MS         500u

HAL_StatusTypeDef Lsc16_RunActionGroup(uint8_t group, uint16_t times);
HAL_StatusTypeDef Lsc16_RunActionGroupBlocking(uint8_t group, uint16_t times,
                                               uint32_t wait_ms);

#endif /* LSC16_ACTION_H */
