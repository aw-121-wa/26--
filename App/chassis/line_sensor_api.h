#ifndef __LINE_SENSOR_API_H
#define __LINE_SENSOR_API_H

#include "sys.h"

typedef struct
{
    uint16_t detail;
    uint8_t ledNum;
    uint8_t lineNum;
} LineSensorSnapshot_t;

void LineSensor_UpdateMain(void);
void LineSensor_GetMain(LineSensorSnapshot_t *snapshot);
uint8_t LineSensor_GetMainLedNum(void);
uint8_t LineSensor_GetMainLineNum(void);
uint16_t LineSensor_GetMainDetail(void);

#endif /* __LINE_SENSOR_API_H */
