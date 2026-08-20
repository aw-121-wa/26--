#ifndef LSC16_ACTION_H
#define LSC16_ACTION_H

#include "main.h"

typedef enum {
    LSC16_ACTION_LIE_DOWN       = 0u,
    LSC16_ACTION_STAND_UP      = 1u,
    LSC16_ACTION_WAVE_LEFT     = 2u,
    LSC16_ACTION_WAVE_RIGHT    = 3u,
    LSC16_ACTION_WAVE_STOP     = 4u,
    LSC16_ACTION_CAMERA_LEFT   = 5u,
    LSC16_ACTION_CAMERA_RIGHT  = 6u,
    LSC16_ACTION_CAMERA_CENTER = 7u
} Lsc16ActionGroup_t;

#define LSC16_ACTION_RUN_ONCE        1u
#define LSC16_WAIT_STAND_MS          200u
#define LSC16_WAIT_LIE_MS            150u
#define LSC16_WAIT_GESTURE_MS        250u
#define LSC16_WAIT_CAMERA_MS         150u

HAL_StatusTypeDef Lsc16_RunActionGroup(uint8_t group, uint16_t times);
HAL_StatusTypeDef Lsc16_RunActionGroupBlocking(uint8_t group, uint16_t times,
                                               uint32_t wait_ms);

#endif /* LSC16_ACTION_H */
