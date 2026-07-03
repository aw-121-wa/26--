/**
 * @file    map_message.c
 * @brief   地图节点数据表
 * @details 定义所有节点的连接关系、标志位、速度等信息
 *          节点格式：{目标节点, 标志位, 角度, 长度(cm), 速度, 功能}
 */

#include "map.h"

/* ======================== 节点数据表 ======================== */
/*
 * 索引顺序：S1, P1, N1(3邻), B1(2邻), B2(2邻), B3(2邻), N2(3邻), P2(1邻), ...
 * 每个节点的所有邻居连续存储
 */

NODE Node[126] = {
    /* ---- S1 (0) ---- */
    /*S1 -> N3*/  {N3, CLEFT|DLEFT|MUL2MUL, 160, 180, SPEED4, NONE},

    /* ---- P1 (1) ---- */
    /*P1 -> N1*/  {N1, CRIGHT|LEFT_LINE, 180, 10, SPEED2, NONE},

    /* ---- N1 (2) - 3个邻居：P1, B2, B1 ---- */
    /*N1 -> P1*/  {P1, RIGHT_LINE, 0, 30, SPEED2, UpStage},
    /*N1 -> B2*/  {B2, NO, 142, 30, SPEED2, Hill},
    /*N1 -> B1*/  {B1, RESTMPUZ|LEFT_LINE, 180, 25, SPEED1, Bridge},

    /* ---- B1 (3) - 2个邻居：N2, N1 ---- */
    /*B1 -> N2*/  {N2, LEFT_LINE|CRIGHT|MUL2SING, 180, 33, SPEED1, NONE},
    /*B1 -> N1*/  {N1, RIGHT_LINE|MCLEFT|CLEFT|DLEFT|STOPTURN, 0, 5, SPEED2, NONE},

    /* ---- B2 (4) - 2个邻居：N1, N4 ---- */
    /*B2 -> N1*/  {N1, LEFT_LINE|CRIGHT, -40, 15, SPEED0, NONE},
    /*B2 -> N4*/  {N4, CLEFT|MCLEFT|LEFT_LINE, 140, 6, SPEED2, NONE},

    /* ---- B3 (5) - 2个邻居：N2, N4 ---- */
    /*B3 -> N2*/  {N2, RIGHT_LINE|CLEFT|STOPTURN, -150, 30, SPEED1, NONE},
    /*B3 -> N4*/  {N4, CLEFT, 30, 43, SPEED1, NONE},

    /* ---- N2 (6) - 3个邻居：B3, P2, B1 ---- */
    /*N2 -> B3*/  {B3, NO, 30, 30, SPEED1, BLBS},
    /*N2 -> P2*/  {P2, LEFT_LINE, 180, 10, SPEED0, UpStageP2},
    /*N2 -> B1*/  {B1, RESTMPUZ|RIGHT_LINE, 0, 30, SPEED2, Bridge},

    /* ---- P2 (7) - 1个邻居：N2 ---- */
    /*P2 -> N2*/  {N2, CLEFT|RIGHT_LINE, 0, 25, SPEED1, NONE},

    /* ---- 后续节点预留 (8~51) ---- */
    /* S2 */  {N6, MUL2MUL|RIGHT_LINE|CLEFT|STOPTURN, 45, 100, SPEED4, NONE},
    /* P3 */  {N3, DRIGHT|RIGHT_LINE, 180, 205, SPEED4, NONE},
    /* N3 (5邻居) */
    {S1, NO, -25, 180, SPEED4, View},
    {P3, LEFT_LINE, 0, 205, SPEED4, UpStage},
    {N10, DLEFT|RIGHT_LINE, 90, 90, SPEED3, DOOR},
    {N8, DRIGHT|DLEFT, 140, 75, SPEED0, DOOR},
    {N4, LEFT_LINE|Temp_R|CLEFT, 180, 100, SPEED3, NONE},
    /* N4 (4邻居) */
    {B2, NO, -40, 20, SPEED1, Hill},
    {N5, LiuShui|MUL2SING|RIGHT_LINE|Temp_L, 180, 100, SPEED3, NONE},
    {N3, DLEFT|Temp_R|LEFT_LINE, 0, 100, SPEED3, NONE},
    {B3, LEFT_LINE, -144, 43, SPEED1, BLBS},
    /* N5 (4邻居) */
    {N4, RIGHT_LINE|Temp_L|MUL2SING, 0, 120, SPEED3, NONE},
    {N8, CLEFT|DLEFT, 35, 80, SPEED0, DOOR},
    {N12, AWHITE|RESTMPUZ, 90, 90, SPEED1, DOOR},
    {N6, LEFT_LINE|MUL2SING, 180, 104, SPEED25, NONE},
    /* N6 (4邻居) */
    {N5, DLEFT|RIGHT_LINE, 0, 95, SPEED3, NONE},
    {C1, CLEFT|DLEFT, 50, 150, SPEED1, NONE},
    {P4, LiuShui|RIGHT_LINE, 180, 55, SPEED3, UpStage},
    {S2, NO, -140, 100, SPEED4, View},
    /* P4 */  {N6, LEFT_LINE|MUL2SING|NOTURN, 0, 55, SPEED3, NONE},
    /* N7 (3邻居) */
    {P6, NO, 90, 5, SPEED1, UpStage},
    {B9, NO, 0, 0, SPEED1, NONE},
    {B8, LEFT_LINE|NOTURN, 10, 1, SPEED1, QQB},
    /* P5 */  {N13, MUL2SING|CLEFT|CRIGHT|LEFT_LINE, 0, 85, SPEED2, NONE},
    /* B8 (2邻居) */
    {N7, NO, 0, 0, SPEED1, NONE},
    {N9, LEFT_LINE|MUL2MUL|MUL2SING|STOPTURN, 160, 40, SPEED0-7, NONE},
    /* B9 (2邻居) */
    {N7, DLEFT|MORELED|STOPTURN, 10, 55, SPEED1, NONE},
    {N9, NO, 0, 0, SPEED1, NONE},
    /* N8 (4邻居) */
    {N3, CLEFT|LEFT_LINE|MUL2MUL, -45, 60, SPEED0, DOOR},
    {N10, MUL2MUL, 33, 140, SPEED3, NONE},
    {N12, MUL2MUL, 140, 150, SPEED3, NONE},
    {N5, STOPTURN|CLEFT, -140, 150, SPEED0, DOOR},
    /* C1 (2邻居) */
    {N6, CRIGHT, -50, 150, SPEED1, NONE},
    {C2, DRIGHT|DLEFT, 125, 30, SPEED1, NONE},
    /* C2 (2邻居) */
    {C1, CLEFT, 170, 154, SPEED1, NONE},
    {N13, DRIGHT|DLEFT|CLEFT|CRIGHT|DRIFT, 120, 20, SPEED1, NONE},
    /* C3 (2邻居) */
    {N14, DLEFT|STOPTURN, 90, 30, SPEED2, NONE},
    {N9, RIGHT_LINE|MUL2SING|STOPTURN, 180, 40, SPEED2, NONE},
    /* N9 (4邻居) */
    {C3, DLEFT|CLEFT|STOPTURN|LEFT_LINE, 10, 40, SPEED2, NONE},
    {N10, DLEFT|DRIGHT|RIGHT_LINE, 180, 110, SPEED3, NONE},
    {B8, NO, 0, 0, SPEED1, NONE},
    {B9, LEFT_LINE|NOTURN, -155, 1, SPEED1, QQB},
    /* N10 (6邻居) */
    {N9, LEFT_LINE|CRIGHT|MUL2SING|STOPTURN, 0, 140, SPEED3, NONE},
    {N15, DRIGHT|STOPTURN, 90, 20, SPEED2, NONE},
    {N12, DRIGHT|RIGHT_LINE, -180, 220, SPEED1, NONE},
    {N8, LEFT_LINE|CLEFT|CRIGHT|DLEFT|DRIGHT, -160, 140, SPEED3, NONE},
    {N3, DRIGHT|DLEFT, -90, 95, SPEED0, DOOR},
    {N11, NO, 180, 80, SPEED0, SM},
    /* N12 (6邻居) */
    {N11, NO, 0, 50, SPEED1, SM},
    {N16, DRIGHT|STOPTURN, 90, 20, SPEED2, NONE},
    {N13, CLEFT|CRIGHT|MUL2SING|LEFT_LINE, 180, 70, SPEED3, NONE},
    {N5, AWHITE|RIGHT_LINE|RESTMPUZ, -90, 185, SPEED4, NONE},
    {N8, CRIGHT|DLEFT, -43, 150, SPEED3, NONE},
    {P5, LiuShui, 180, 240, SPEED4, UpStage},
    /* N13 (4邻居) */
    {N12, DLEFT|DRIGHT|LiuShui|RIGHT_LINE, 0, 90, SPEED3, NONE},
    {N18, CRIGHT|CLEFT, 45, 190, SPEED3, NONE},
    {P5, LiuShui|RIGHT_LINE, 180, 85, SPEED3, UpStage},
    {C2, NO, 0, 0, SPEED1, NONE},
    /* P6 */  {N7, DLEFT|DRIGHT|AWHITE|STOPTURN, -90, 15, SPEED1, NONE},
    /* N14 (3邻居) */
    {C3, DRIGHT|CRIGHT, -90, 30, SPEED0, NONE},
    {C7, CLEFT|DLEFT, 90, 90, SPEED2, UNDER},
    {S3, NO, -180, 2, SPEED1, View1},
    /* S3 */  {N14, INGNORE, 180, 2, -SPEED0, BACK},
    /* S4 */  {N15, INGNORE, 0, 1, -SPEED0, BACK},
    /* N15 (3邻居) */
    {S4, NO, 0, 1, SPEED1, View1},
    {C5, DLEFT, 90, 30, SPEED2, NONE},
    {N10, DLEFT|DRIGHT, -90, 20, SPEED2, NONE},
    /* S5 */  {N16, INGNORE, 0, 1, -SPEED0, BACK},
    /* C4 (2邻居) */
    {C8, DRIGHT|CRIGHT, 90, 10, SPEED0, UNDER},
    {N20, MUL2SING, 155, 180, SPEED4, NONE},
    /* C5 (2邻居) */
    {N15, DLEFT|STOPTURN, -90, 30, SPEED2, NONE},
    {N18, DLEFT|CLEFT, 180, 270, SPEED4, NONE},
    /* B4 (2邻居) */
    {C5, DRIGHT, 0, 145, SPEED1, NONE},
    {N18, DLEFT|CLEFT|RESTMPUZ, 180, 100, SPEED1, NONE},
    /* B5 (2邻居) */
    {N18, DRIGHT, 0, 70, SPEED3, NONE},
    {N19, DRIGHT, 180, 45, SPEED3, NONE},
    /* B6 (2邻居) */
    {N20, CRIGHT|DRIGHT|RESTMPUZ|LEFT_LINE, 0, 50, SPEED2, NONE},
    {N22, DRIGHT|RESTMPUZ, 180, 30, SPEED3, NONE},
    /* B7 (2邻居) */
    {N22, DLEFT|RESTMPUZ, 0, 70, SPEED3, NONE},
    {C6, DLEFT, 180, 45, SPEED3, NONE},
    /* N16 (3邻居) */
    {S5, NO, 0, 1, SPEED1, View1},
    {N12, DLEFT|DRIGHT, -90, 20, SPEED2, NONE},
    {N18, DRIGHT|RIGHT_LINE, 90, 25, SPEED2, NONE},
    /* N18 (3邻居) */
    {C5, DRIGHT|CRIGHT, 0, 270, SPEED3, NONE},
    {B5, RIGHT_LINE, 180, 88, SPEED25, Hill},
    {N16, DLEFT|CLEFT|STOPTURN, -90, 25, SPEED2, NONE},
    /* N19 (3邻居) */
    {B5, NO, 0, 45, SPEED3, Hill},
    {C6, DRIGHT|CRIGHT, 90, 20, SPEED2, NONE},
    {N13, CRIGHT, 0, 0, SPEED1, NONE},
    /* P7 */  {N20, MCLEFT|RIGHT_LINE|STOPTURN, 180, 50, SPEED1, NONE},
    /* N20 (3邻居) */
    {C4, CLEFT|MCLEFT|STOPTURN, -42, 180, SPEED4, NONE},
    {P7, LEFT_LINE|RESTMPUZ, 0, 10, SPEED0, BHM},
    {B6, NO, 180, 20, SPEED1, SM},
    /* N22 (3邻居) */
    {C9, DLEFT, 90, 15, SPEED0, NONE},
    {B7, NO, 180, 115, SPEED25, Hill},
    {B6, RESTMPUZ, 0, 30, SPEED1, SM},
    /* C6 (2邻居) */
    {B7, RESTMPUZ, 0, 60, SPEED25, Hill},
    {N19, DLEFT, -90, 25, SPEED3, NONE},
    /* C7 (2邻居) */
    {N14, DRIGHT|CRIGHT|STOPTURN, -90, 1, SPEED3, UNDER},
    {C8, DLEFT|STOPTURN, 180, 100, SPEED2, NONE},
    /* C8 (2邻居) */
    {C7, STOPTURN|DRIGHT|CRIGHT|RESTMPUZ, 0, 100, SPEED4, NONE},
    {C4, MCRIGHT|CRIGHT|RESTMPUZ, -90, 0, SPEED1, UNDER},
    /* C9 (3邻居) */
    {N22, DLEFT|DRIGHT|AWHITE, -90, 15, SPEED0, NONE},
    {P8, NO, 180, 200, SPEED5, BSoutPole},
    {G1, NO, 180, 120, SPEED3, NONE},
    /* P8 (2邻居) */
    {C9, DRIGHT, 0, 220, SPEED4, NONE},
    {G1, NO, 0, 120, SPEED3, NONE},
    /* N11 (2邻居) */
    {N10, LEFT_LINE|DLEFT, 0, 80, SPEED25, NONE},
    {N12, RIGHT_LINE|AWHITE|DRIGHT, 180, 50, SPEED25, NONE},
    /* G1 (2邻居) */
    {C9, DRIGHT|CRIGHT, 0, 120, SPEED25, NONE},
    {P8, RESTMPUZ, 180, 120, SPEED3, BSoutPole}
};

/* ======================== 连接数表 ======================== */

uint8_t ConnectionNum[52] = {
    1, 1, 3, 2, 2, 2, 3, 1, 1, 1,
    5, 4, 4, 4, 1, 3, 1, 2, 2, 4,
    2, 2, 2, 4, 6, 6, 4, 1, 3, 1,
    1, 3, 1, 2, 2, 2, 2, 2, 2, 3,
    3, 3, 1, 3, 3, 2, 2, 2, 3, 2,
    2, 2
};

/* ======================== 地址表（每个节点在Node[]中的起始下标） ======================== */

uint8_t Address[53] = {
    0, 1, 2, 5, 7, 9, 11, 14, 15, 16,
    17, 22, 26, 30, 34, 35, 38, 39, 41, 43,
    47, 49, 51, 53, 57, 63, 69, 73, 74, 77,
    78, 79, 82, 83, 85, 87, 89, 91, 93, 95,
    98, 101, 104, 105, 108, 111, 113, 115, 117, 120,
    122, 124, 126
};
