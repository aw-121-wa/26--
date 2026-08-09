#ifndef HMI_DISPLAY_H
#define HMI_DISPLAY_H

#include <stdint.h>

typedef struct {
    uint16_t upright_spots;
    uint16_t platforms_1_to_5;
    uint16_t platform_6;
    uint16_t platform_7;
    uint16_t platform_8;
    uint16_t home_returns;
    uint32_t total_score;
} HmiDisplayScores_t;

void HmiDisplay_Init(void);
void HmiDisplay_ResetScores(void);
void HmiDisplay_RecordArrival(uint8_t node, uint8_t function);
void HmiDisplay_Tick(void);
HmiDisplayScores_t HmiDisplay_GetScores(void);

#endif /* HMI_DISPLAY_H */
