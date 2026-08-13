#include "rudder_control.h"

#define RUDDER_MODE1 0x00

/**
 * @brief  初始化 PCA9685 舵机板
 * @note   PA11/PA12 为软件 I2C，PA15 为 OE 使能脚，低电平使能输出。
 */
void Rudder_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t old_mode;
    uint8_t new_mode;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    IIC_Init();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    /* OE 低有效；初始化后保持低电平，让 PCA9685 输出 PWM。 */
    gpio.Pin = GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    /* 进入 sleep 后写分频，再退出 sleep，流程来自参考工程。 */
    Rudder_WriteOneByte(RUDDER_MODE1, 0x01);
    old_mode = Rudder_ReadOneByte(RUDDER_MODE1);
    new_mode = (old_mode & 0x7F) | 0x10;
    Rudder_WriteOneByte(RUDDER_MODE1, new_mode);
    Rudder_WriteOneByte(PRE_SCALE, 132);
    Rudder_WriteOneByte(RUDDER_MODE1, 0x01);
    delay_ms(10);
}

/**
 * @brief  向舵机板写 1 字节寄存器
 * @param  addr 寄存器地址
 * @param  data 写入数据
 */
void Rudder_WriteOneByte(uint8_t addr, uint8_t data)
{
    IIC_Start();
    IIC_Send_Byte(Rudder << 1);
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    IIC_Wait_Ack();
    IIC_Stop();
    delay_ms(10);
}

/**
 * @brief  从舵机板读 1 字节寄存器
 * @param  addr 寄存器地址
 * @return 读出的寄存器值
 */
uint8_t Rudder_ReadOneByte(uint8_t addr)
{
    uint8_t data;

    IIC_Start();
    IIC_Send_Byte(Rudder << 1);
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((Rudder << 1) + 1);
    IIC_Wait_Ack();
    data = IIC_Read_Byte(0);
    IIC_Stop();

    return data;
}

/**
 * @brief  设置指定通道 PWM 位置
 * @param  aim PCA9685 OFF 计数值，参考工程中 0 度约 350，270 度约 2100
 * @param  id  舵机通道号
 */
void Rudder_control(uint16_t aim, uint8_t id)
{
    Rudder_WriteOneByte(LED0_ON_L + (4 * id), 0);
    Rudder_WriteOneByte(LED0_ON_H + (4 * id), 0);
    Rudder_WriteOneByte(LED0_OFF_L + (4 * id), (uint8_t)(aim & 0xFF));
    Rudder_WriteOneByte(LED0_OFF_H + (4 * id), (uint8_t)(aim >> 8));
}
