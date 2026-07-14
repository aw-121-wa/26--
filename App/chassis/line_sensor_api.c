#include "line_sensor_api.h"
#include "scaner.h"

void LineSensor_UpdateMain(void)
{
    getline_error();
}

void LineSensor_GetMain(LineSensorSnapshot_t *snapshot)
{
    if (snapshot == 0)
        return;

    snapshot->detail = Scaner.detail;
    snapshot->ledNum = Scaner.ledNum;
    snapshot->lineNum = Scaner.lineNum;
}

uint8_t LineSensor_GetMainLedNum(void)
{
    return Scaner.ledNum;
}

uint8_t LineSensor_GetMainLineNum(void)
{
    return Scaner.lineNum;
}

uint16_t LineSensor_GetMainDetail(void)
{
    return Scaner.detail;
}
