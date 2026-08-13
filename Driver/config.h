#ifndef CONFIG_H
#define CONFIG_H

#include "sys.h"

/*轮径参数（距离计算用）*/
#define WHEEL_DIAMETER_CM       10.4f
#define ENCODER_TICKS_PER_REV   5720.0f
#define GEAR_RATIO              0.362f
#define RIGHT_REAR_COMPENSATE   1.65f

/*舵机*/
#define SERVO_NEUTRAL_PULSE     299

/*nodesr.flag 位定义*/
#define NODE_FLAG_CLEAR_REQ     0x01
#define NODE_FLAG_START_DETECT  0x02
#define NODE_FLAG_ARRIVED       0x04
#define NODE_FLAG_TEMP_CLEAR    0x08
#define NODE_FLAG_Z_RESET       0x10
#define NODE_FLAG_ROUTE_RESET   0x20
#define NODE_FLAG_NO_DOOR       0x40
#define NODE_FLAG_RED_LIGHT     0x80

/*扫描器位掩码（Cross_Scaner.detail）*/
#define SCAN_LED_ALL_ON         0x1FF8
#define SCAN_LEFT_BRANCH        0x4000
#define SCAN_RIGHT_BRANCH       0x2000
#define SCAN_CROSS_FULL         0x0180
#define SCAN_EDGE_MASK          0xC003

/*路由结束标记*/
#define ROUTE_END               0xFF

/*颜色值*/
#define COLOR_GREEN             0x01
#define COLOR_BLUE              0x02
#define COLOR_BLACK             0x03

/*主循环周期*/
#define MAIN_TASK_PERIOD_MS     5

/*PWM*/
#define BRAKE_PWM               1500

/*音频命令*/
#define AUDIO_CMD_ARRIVED       0x0A

#endif
