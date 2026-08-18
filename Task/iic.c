#include "iic.h"

/* SDA 释放为开漏输出高电平，允许从设备拉低 ACK。 */
static void iic_sda_output(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/* 读 ACK/数据时切为输入，避免主机强推总线。 */
static void iic_sda_input(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/**
 * @brief  初始化软件 I2C 总线
 * @note   CubeMX 已把 PA11/PA12 配成 GPIO 输出，这里按舵机板通信需要重配为开漏上拉。
 */
void IIC_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    IIC_SDA_OUT;
    IIC_SCK_OUT;
}

/**
 * @brief  设置 SDA 方向
 * @param  param 1=开漏输出，0=输入
 */
void SDA(uint8_t param)
{
    if (param)
        iic_sda_output();
    else
        iic_sda_input();
}

/**
 * @brief  产生 I2C 起始信号
 */
void IIC_Start(void)
{
    SDA(1);
    IIC_SDA_OUT;
    IIC_SCK_OUT;
    delay_us(4);
    IIC_SDA_DOWN;
    delay_us(4);
    IIC_SCK_DOWN;
}

/**
 * @brief  产生 I2C 停止信号
 */
void IIC_Stop(void)
{
    SDA(1);
    IIC_SCK_DOWN;
    IIC_SDA_DOWN;
    delay_us(4);
    IIC_SCK_OUT;
    IIC_SDA_OUT;
    delay_us(4);
}

/**
 * @brief  等待从设备 ACK
 * @return 0=收到 ACK，1=超时未收到 ACK
 */
uint8_t IIC_Wait_Ack(void)
{
    uint8_t err = 0;

    SDA(0);
    delay_us(1);
    IIC_SCK_OUT;
    delay_us(1);

    while (READ_SDA)
    {
        err++;
        if (err > 250)
        {
            IIC_Stop();
            return 1;
        }
    }

    IIC_SCK_DOWN;
    return 0;
}

/**
 * @brief  主机发送 ACK
 */
void IIC_Ack(void)
{
    IIC_SCK_DOWN;
    SDA(1);
    IIC_SDA_DOWN;
    delay_us(2);
    IIC_SCK_OUT;
    delay_us(2);
    IIC_SCK_DOWN;
}

/**
 * @brief  主机发送 NACK
 */
void IIC_NAck(void)
{
    IIC_SCK_DOWN;
    SDA(1);
    IIC_SDA_OUT;
    delay_us(2);
    IIC_SCK_OUT;
    delay_us(2);
    IIC_SCK_DOWN;
}

/**
 * @brief  发送 1 字节数据，MSB 先发
 * @param  txd 待发送字节
 */
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t i;

    SDA(1);
    IIC_SCK_DOWN;

    for (i = 0; i < 8; i++)
    {
        if (txd & 0x80)
            IIC_SDA_OUT;
        else
            IIC_SDA_DOWN;

        txd <<= 1;
        delay_us(2);
        IIC_SCK_OUT;
        delay_us(2);
        IIC_SCK_DOWN;
        delay_us(2);
    }
}

/**
 * @brief  读取 1 字节数据
 * @param  ack 1=读完发送 ACK，0=读完发送 NACK
 * @return 读取到的字节
 */
uint8_t IIC_Read_Byte(unsigned char ack)
{
    uint8_t i;
    uint8_t data = 0;

    SDA(0);

    for (i = 0; i < 8; i++)
    {
        IIC_SCK_DOWN;
        delay_us(2);
        IIC_SCK_OUT;
        data <<= 1;
        if (READ_SDA)
            data++;
        delay_us(1);
    }

    if (ack)
        IIC_Ack();
    else
        IIC_NAck();

    return data;
}
