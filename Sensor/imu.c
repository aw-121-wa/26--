/**
 * @file    imu.c
 * @brief   IMU 惯性测量单元驱动模块
 * @details 通过 USART3 + DMA 接收陀螺仪数据，解析欧拉角
 */

#include "imu.h"
#include "usart.h"
#include "main.h"
#include "string.h"
#include "filter.h"

/* USART3 RX DMA 句柄定义在 Core/Src/usart.c，但未在 usart.h 中声明 */
extern DMA_HandleTypeDef hdma_usart3_rx;

/* ======================== 全局变量定义 ======================== */

struct Imu imu;             /* IMU 数据对象 */
float basic_p = 0;          /* 基准俯仰角 */
float basic_y = 0;          /* 基准偏航角 */
float basic_r = 0;

/* ======================== 私有变量 ======================== */

#define BUFFER_SIZE 132     /* DMA 接收缓冲区大小 */

static uint8_t imu_rx_buf[BUFFER_SIZE] = {0};  /* DMA 接收缓冲区 */
static uint8_t imu_rx_len = 0;                  /* 接收数据长度 */

/* ======================== IMU 初始化 ======================== */

void imu_receive_init(void)
{
    HAL_UART_Receive_DMA(&huart3, imu_rx_buf, BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

/* ======================== Z 轴角度补偿 ======================== */

void mpuZreset(float sensorangle, float referangle)
{
    float diff = referangle - sensorangle;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    imu.compensateZ = diff;
}

/* ======================== USART3 中断处理 ======================== */

void USART3_IRQHandler(void)
{
    uint32_t flag_idle = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE);

    if (flag_idle != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);

        HAL_UART_DMAStop(&huart3);
        uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
        imu_rx_len = BUFFER_SIZE - temp;

        /* 解析数据帧：帧头 0x55，数据标识 0x51 和 0x53 */
        if (imu_rx_len >= 33 &&
            imu_rx_buf[0] == 0x55 && imu_rx_buf[1] == 0x51 &&
            imu_rx_buf[22] == 0x55 && imu_rx_buf[23] == 0x53)
        {
            uint8_t sum = 0;
            for (int i = 22; i < 32; i++)
                sum += imu_rx_buf[i];

            if (sum == imu_rx_buf[32])
            {
                imu.pitch = -((short)(((short)imu_rx_buf[25] << 8) | imu_rx_buf[24])) / 32768.0f * 180.0f;
                imu.roll  =  ((short)(((short)imu_rx_buf[27] << 8) | imu_rx_buf[26])) / 32768.0f * 180.0f;
                imu.yaw   =  ((short)(((short)imu_rx_buf[29] << 8) | imu_rx_buf[28])) / 32768.0f * 180.0f;

                if (filter_Open)
                {
                    imu.pitch = filter(imu.pitch);
                    imu.roll  = filter(imu.roll);
                    imu.yaw   = filter(imu.yaw);
                }
            }
        }

        memset(imu_rx_buf, 0, imu_rx_len);
        imu_rx_len = 0;
    }

    HAL_UART_Receive_DMA(&huart3, imu_rx_buf, BUFFER_SIZE);
    HAL_UART_IRQHandler(&huart3);
}
