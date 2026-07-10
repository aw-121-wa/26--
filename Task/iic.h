#ifndef __IIC_H__
#define __IIC_H__

#include "main.h"
#include "delay.h"

/* 软件 I2C 引脚：SDA=PA11，SCL=PA12。 */
#define IIC_SDA_OUT   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
#define IIC_SDA_DOWN  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)
#define READ_SDA      HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)

#define IIC_SCK_OUT   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET)
#define IIC_SCK_DOWN  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)

void SDA(uint8_t param);
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
/* 返回 0=收到 ACK，1=超时未收到 ACK。 */
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(unsigned char ack);

#endif
