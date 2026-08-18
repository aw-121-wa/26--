#ifndef __RUDDER_CONTROL_H__
#define __RUDDER_CONTROL_H__

#include "main.h"
#include "iic.h"

/* PCA9685 舵机驱动板 7 位 I2C 地址。 */
#define Rudder      0x70
#define PRE_SCALE   0xFE

/* PCA9685 通道 0~4 的寄存器基地址；更高通道通过 LED0 + 4 * id 计算。 */
#define LED0_ON_L   0x06
#define LED0_ON_H   0x07
#define LED0_OFF_L  0x08
#define LED0_OFF_H  0x09

#define LED1_ON_L   0x0A
#define LED1_ON_H   0x0B
#define LED1_OFF_L  0x0C
#define LED1_OFF_H  0x0D

#define LED2_ON_L   0x0E
#define LED2_ON_H   0x0F
#define LED2_OFF_L  0x10
#define LED2_OFF_H  0x11

#define LED3_ON_L   0x12
#define LED3_ON_H   0x13
#define LED3_OFF_L  0x14
#define LED3_OFF_H  0x15

#define LED4_ON_L   0x16
#define LED4_ON_H   0x17
#define LED4_OFF_L  0x18
#define LED4_OFF_H  0x19

void Rudder_Init(void);
/* aim 是 PCA9685 OFF 计数值，不是角度；id 是舵机通道号。 */
void Rudder_control(uint16_t aim, uint8_t id);
void Rudder_WriteOneByte(uint8_t addr, uint8_t data);
uint8_t Rudder_ReadOneByte(uint8_t addr);

#endif
