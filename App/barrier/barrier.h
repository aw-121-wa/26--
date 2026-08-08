#ifndef __BARRIER_H
#define __BARRIER_H

#include "sys.h"

typedef enum {
    BARRIER_RESULT_OK = 0,
    BARRIER_RESULT_TIMEOUT,
    BARRIER_RESULT_STOPPED,
    BARRIER_RESULT_SENSOR_FAULT,
    BARRIER_RESULT_VISION_FAILED,
    BARRIER_RESULT_BLOCKED
} BarrierResult_t;

BarrierResult_t zhunbei(void);
BarrierResult_t Stage_P2(void);
BarrierResult_t Stage(void);
BarrierResult_t Barrier_Bridge(void);
BarrierResult_t Barrier_Hill(void);
BarrierResult_t Barrier_DoubleHill(void);
BarrierResult_t Barrier_SwordMountain(void);
BarrierResult_t Barrier_View(uint8_t short_marker);
BarrierResult_t Barrier_Back(void);
BarrierResult_t Barrier_SouthPole(void);
BarrierResult_t Barrier_Seesaw(void);
BarrierResult_t Barrier_WavedPlate(float length);
BarrierResult_t Barrier_Door(uint8_t alternate_camera);
BarrierResult_t Barrier_HighMountain(void);
BarrierResult_t Barrier_Under(void);
BarrierResult_t Barrier_SpecialNode(void);
BarrierResult_t Barrier_Ignore(void);

#endif
