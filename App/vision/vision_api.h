#ifndef VISION_API_H
#define VISION_API_H

#include "sys.h"

#define VISION_SOF_0             0xAA
#define VISION_SOF_1             0x55
#define VISION_PROTOCOL_VERSION  0x01
#define VISION_MAX_PAYLOAD       16
#define VISION_RX_BUFFER_SIZE    128
#define VISION_INJECT_QUEUE_SIZE 8

#define VISION_SERVO_CHANNEL     1
#define VISION_SERVO_LEFT        100
#define VISION_SERVO_CENTER      170
#define VISION_SERVO_RIGHT       250
#define VISION_SERVO_SETTLE_MS   350
#define VISION_RESULT_TIMEOUT_MS 800
#define VISION_SCAN_TIMEOUT_MS   6000
#define VISION_MIN_CONFIDENCE    60
#define VISION_SIDE_SAMPLES      3

typedef enum {
    VISION_MSG_HEARTBEAT = 0x01,
    VISION_MSG_SET_MODE  = 0x10,
    VISION_MSG_RECOGNIZE = 0x11,
    VISION_MSG_CANCEL    = 0x12,
    VISION_MSG_ACK       = 0x80,
    VISION_MSG_RESULT    = 0x81,
    VISION_MSG_ERROR     = 0x82
} VisionMessageType_t;

typedef enum {
    VISION_MODE_IDLE = 0,
    VISION_MODE_TRAFFIC_LIGHT,
    VISION_MODE_CLUE,
    VISION_MODE_TREASURE
} VisionMode_t;

typedef enum {
    VISION_DIRECTION_CENTER = 0,
    VISION_DIRECTION_LEFT,
    VISION_DIRECTION_RIGHT
} VisionDirection_t;

typedef enum {
    VISION_COLOR_NONE = 0,
    VISION_COLOR_GREEN,
    VISION_COLOR_YELLOW,
    VISION_COLOR_RED
} VisionTrafficColor_t;

typedef enum {
    VISION_STATUS_OK = 0,
    VISION_STATUS_BUSY,
    VISION_STATUS_TIMEOUT,
    VISION_STATUS_PROTOCOL_ERROR,
    VISION_STATUS_IO_ERROR,
    VISION_STATUS_NO_RESULT,
    VISION_STATUS_INVALID_ARG
} VisionStatus_t;

typedef struct {
    VisionMode_t mode;
    VisionDirection_t direction;
    uint8_t value;
    uint8_t confidence;
    uint8_t sequence;
} VisionResult_t;

typedef struct {
    VisionResult_t left;
    VisionResult_t right;
} VisionPairResult_t;

typedef struct {
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t protocol_errors;
    uint32_t rx_overflows;
    uint32_t last_heartbeat_tick;
    uint8_t last_sequence;
    VisionStatus_t last_status;
} VisionDiagnostics_t;

VisionStatus_t Vision_Init(void);
void Vision_Poll(void);
VisionStatus_t Vision_Request(VisionMode_t mode, VisionDirection_t direction);
VisionStatus_t Vision_WaitResult(VisionResult_t *result, uint32_t timeout_ms);
uint8_t Vision_TakeResult(VisionResult_t *result);
void Vision_InjectResult(const VisionResult_t *result);
VisionStatus_t Vision_ScanTrafficPair(VisionPairResult_t *result);
void Vision_ClearResults(void);
const VisionDiagnostics_t *Vision_GetDiagnostics(void);
uint8_t Vision_Crc8(const uint8_t *data, uint8_t length);

#endif
