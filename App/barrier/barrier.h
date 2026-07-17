#ifndef __BARRIER_H
#define __BARRIER_H

#include "sys.h"
#include "chassis_api.h"

typedef struct {
    float sword_distance_cm;
    float south_pole_distance_cm;
    float view_distance_cm;
    float back_distance_cm;
    float under_distance_cm;
    float seesaw_pitch_enter;
    float seesaw_pitch_leave;
    float high_mountain_pitch;
    float low_speed;
    float normal_speed;
    uint32_t motion_timeout_ms;
    uint32_t pitch_timeout_ms;
} BarrierConfig_t;

typedef struct {
    uint8_t clue_a;
    uint8_t clue_b;
    uint8_t treasure_node;
    uint8_t gate_color[4];
    uint8_t valid;
} BarrierMissionData_t;

extern BarrierConfig_t barrier_config;

void Barrier_SetMissionData(const BarrierMissionData_t *data);
void Barrier_GetMissionData(BarrierMissionData_t *data);
void Barrier_ClearMissionData(void);

/* ======================== 障碍物函数声明 ======================== */

/**
 * @brief  准备函数 - 启动流程
 */
void zhunbei(void);

/**
 * @brief  P2平台处理
 */
void Stage_P2(void);

/**
 * @brief  通用平台处理（P1/P3/P4等）
 */
void Stage(void);

/**
 * @brief  过桥处理
 */
void Barrier_Bridge(void);

/**
 * @brief  楼梯/山地处理
 */
void Barrier_Hill(void);

/**
 * @brief  波浪板处理
 * @param  length 波浪板循线通过距离(cm)
 */
void Barrier_WavedPlate(float length);

ChassisActionResult_t Barrier_DoubleHill(void);
ChassisActionResult_t Barrier_SwordMountain(void);
ChassisActionResult_t Barrier_View(uint8_t turn_after);
ChassisActionResult_t Barrier_Back(void);
ChassisActionResult_t Barrier_SouthPole(void);
ChassisActionResult_t Barrier_Seesaw(void);
ChassisActionResult_t Barrier_HighMountain(void);
ChassisActionResult_t Barrier_Under(void);
ChassisActionResult_t Barrier_SpecialNode(void);
ChassisActionResult_t Barrier_Door(void);

#endif /* __BARRIER_H */
