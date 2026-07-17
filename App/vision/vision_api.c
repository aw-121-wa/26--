#include "vision_api.h"

#include "chassis_api.h"
#include "rudder_control.h"
#include "task.h"
#include "usart.h"
#include <string.h>

#define VISION_FRAME_FIXED_BODY 4u
#define VISION_TX_FRAME_MAX     (2u + VISION_FRAME_FIXED_BODY + VISION_MAX_PAYLOAD + 1u)
#define VISION_RX_MASK          (VISION_RX_BUFFER_SIZE - 1u)
#define VISION_UART_TIMEOUT_MS  50u
#define VISION_POLL_DELAY_MS    5u

#if (VISION_RX_BUFFER_SIZE & (VISION_RX_BUFFER_SIZE - 1u)) != 0
#error VISION_RX_BUFFER_SIZE_must_be_power_of_two
#endif

typedef enum {
    PARSER_SOF0 = 0,
    PARSER_SOF1,
    PARSER_VERSION,
    PARSER_TYPE,
    PARSER_SEQUENCE,
    PARSER_LENGTH,
    PARSER_PAYLOAD,
    PARSER_CRC
} VisionParserState_t;

typedef struct {
    VisionParserState_t state;
    uint8_t type;
    uint8_t sequence;
    uint8_t length;
    uint8_t payload_index;
    uint8_t payload[VISION_MAX_PAYLOAD];
    uint8_t crc;
} VisionParser_t;

static uint8_t rx_buffer[VISION_RX_BUFFER_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static uint8_t uart_rx_byte;
static volatile uint8_t rx_overflow_pending;

static VisionParser_t parser;
static VisionResult_t pending_result;
static volatile uint8_t result_pending;
static VisionResult_t injected_results[VISION_INJECT_QUEUE_SIZE];
static uint8_t inject_head;
static uint8_t inject_tail;
static uint8_t request_pending;
static uint8_t expected_sequence;
static uint8_t tx_sequence;
static VisionDiagnostics_t diagnostics;

uint8_t Vision_Crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;
    uint8_t i;
    uint8_t bit;

    if (data == NULL && length != 0u)
        return 0;

    for (i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8u; bit++)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1u) ^ 0x07u) : (uint8_t)(crc << 1u);
    }
    return crc;
}

static void parser_reset(void)
{
    memset(&parser, 0, sizeof(parser));
    parser.state = PARSER_SOF0;
}

static void parser_crc_add(uint8_t value)
{
    uint8_t bit;
    parser.crc ^= value;
    for (bit = 0; bit < 8u; bit++)
        parser.crc = (parser.crc & 0x80u) ? (uint8_t)((parser.crc << 1u) ^ 0x07u) : (uint8_t)(parser.crc << 1u);
}

static uint8_t rx_pop(uint8_t *value)
{
    if (rx_tail == rx_head)
        return 0;

    *value = rx_buffer[rx_tail & VISION_RX_MASK];
    rx_tail = (uint8_t)((rx_tail + 1u) & VISION_RX_MASK);
    return 1;
}

static VisionStatus_t send_frame(uint8_t type, uint8_t sequence,
                                 const uint8_t *payload, uint8_t length)
{
    uint8_t frame[VISION_TX_FRAME_MAX];
    uint8_t body_length;

    if (length > VISION_MAX_PAYLOAD || (payload == NULL && length != 0u))
        return VISION_STATUS_INVALID_ARG;

    frame[0] = VISION_SOF_0;
    frame[1] = VISION_SOF_1;
    frame[2] = VISION_PROTOCOL_VERSION;
    frame[3] = type;
    frame[4] = sequence;
    frame[5] = length;
    if (length != 0u)
        memcpy(&frame[6], payload, length);

    body_length = (uint8_t)(VISION_FRAME_FIXED_BODY + length);
    frame[6u + length] = Vision_Crc8(&frame[2], body_length);
    if (HAL_UART_Transmit(&huart5, frame, (uint16_t)(7u + length), VISION_UART_TIMEOUT_MS) != HAL_OK)
    {
        diagnostics.last_status = VISION_STATUS_IO_ERROR;
        return VISION_STATUS_IO_ERROR;
    }
    return VISION_STATUS_OK;
}

static void accept_frame(void)
{
    diagnostics.valid_frames++;
    diagnostics.last_sequence = parser.sequence;

    if (parser.type == VISION_MSG_HEARTBEAT)
    {
        diagnostics.last_heartbeat_tick = (uint32_t)xTaskGetTickCount();
        diagnostics.last_status = VISION_STATUS_OK;
        return;
    }

    if (parser.type == VISION_MSG_ERROR)
    {
        request_pending = 0;
        diagnostics.last_status = VISION_STATUS_PROTOCOL_ERROR;
        return;
    }

    if (parser.type != VISION_MSG_RESULT)
        return;

    if (parser.length != 4u || parser.payload[0] > VISION_MODE_TREASURE ||
        parser.payload[1] > VISION_DIRECTION_RIGHT || parser.payload[3] > 100u)
    {
        diagnostics.protocol_errors++;
        diagnostics.last_status = VISION_STATUS_PROTOCOL_ERROR;
        return;
    }

    if (request_pending && parser.sequence != expected_sequence)
        return;

    pending_result.mode = (VisionMode_t)parser.payload[0];
    pending_result.direction = (VisionDirection_t)parser.payload[1];
    pending_result.value = parser.payload[2];
    pending_result.confidence = parser.payload[3];
    pending_result.sequence = parser.sequence;
    result_pending = 1;
    request_pending = 0;
    diagnostics.last_status = VISION_STATUS_OK;
}

static void parse_byte(uint8_t value)
{
    switch (parser.state)
    {
        case PARSER_SOF0:
            if (value == VISION_SOF_0)
                parser.state = PARSER_SOF1;
            break;
        case PARSER_SOF1:
            if (value == VISION_SOF_1)
                parser.state = PARSER_VERSION;
            else
                parser.state = (value == VISION_SOF_0) ? PARSER_SOF1 : PARSER_SOF0;
            break;
        case PARSER_VERSION:
            parser.crc = 0;
            parser_crc_add(value);
            if (value != VISION_PROTOCOL_VERSION)
            {
                diagnostics.protocol_errors++;
                parser_reset();
            }
            else
                parser.state = PARSER_TYPE;
            break;
        case PARSER_TYPE:
            parser.type = value;
            parser_crc_add(value);
            parser.state = PARSER_SEQUENCE;
            break;
        case PARSER_SEQUENCE:
            parser.sequence = value;
            parser_crc_add(value);
            parser.state = PARSER_LENGTH;
            break;
        case PARSER_LENGTH:
            parser.length = value;
            parser.payload_index = 0;
            parser_crc_add(value);
            if (value > VISION_MAX_PAYLOAD)
            {
                diagnostics.protocol_errors++;
                parser_reset();
            }
            else
                parser.state = (value == 0u) ? PARSER_CRC : PARSER_PAYLOAD;
            break;
        case PARSER_PAYLOAD:
            parser.payload[parser.payload_index++] = value;
            parser_crc_add(value);
            if (parser.payload_index >= parser.length)
                parser.state = PARSER_CRC;
            break;
        case PARSER_CRC:
            if (value == parser.crc)
                accept_frame();
            else
            {
                diagnostics.crc_errors++;
                diagnostics.last_status = VISION_STATUS_PROTOCOL_ERROR;
            }
            parser_reset();
            break;
        default:
            parser_reset();
            break;
    }
}

VisionStatus_t Vision_Init(void)
{
    rx_head = 0;
    rx_tail = 0;
    rx_overflow_pending = 0;
    result_pending = 0;
    inject_head = 0u;
    inject_tail = 0u;
    request_pending = 0;
    expected_sequence = 0;
    tx_sequence = 0;
    memset(&diagnostics, 0, sizeof(diagnostics));
    parser_reset();

    if (HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1u) != HAL_OK)
    {
        diagnostics.last_status = VISION_STATUS_IO_ERROR;
        return VISION_STATUS_IO_ERROR;
    }
    diagnostics.last_status = VISION_STATUS_OK;
    return VISION_STATUS_OK;
}

void Vision_Poll(void)
{
    uint8_t value;

    if (rx_overflow_pending)
    {
        rx_overflow_pending = 0;
        diagnostics.rx_overflows++;
        diagnostics.last_status = VISION_STATUS_PROTOCOL_ERROR;
        parser_reset();
    }

    while (rx_pop(&value))
        parse_byte(value);
}

VisionStatus_t Vision_Request(VisionMode_t mode, VisionDirection_t direction)
{
    uint8_t payload[3];
    VisionStatus_t status;

    if (mode == VISION_MODE_IDLE || mode > VISION_MODE_TREASURE || direction > VISION_DIRECTION_RIGHT)
        return VISION_STATUS_INVALID_ARG;
    if (request_pending)
        return VISION_STATUS_BUSY;
    if (inject_head != inject_tail)
        return VISION_STATUS_OK;

    tx_sequence++;
    expected_sequence = tx_sequence;
    payload[0] = (uint8_t)mode;
    payload[1] = (uint8_t)direction;
    payload[2] = 1u;
    result_pending = 0;
    status = send_frame(VISION_MSG_RECOGNIZE, expected_sequence, payload, sizeof(payload));
    if (status == VISION_STATUS_OK)
        request_pending = 1;
    return status;
}

uint8_t Vision_TakeResult(VisionResult_t *result)
{
    if (result == NULL)
        return 0;
    if (inject_head != inject_tail)
    {
        *result = injected_results[inject_tail];
        inject_tail = (uint8_t)((inject_tail + 1u) % VISION_INJECT_QUEUE_SIZE);
        return 1;
    }
    if (!result_pending)
        return 0;
    *result = pending_result;
    result_pending = 0;
    return 1;
}

VisionStatus_t Vision_WaitResult(VisionResult_t *result, uint32_t timeout_ms)
{
    TickType_t start;
    TickType_t timeout_ticks;

    if (result == NULL || timeout_ms == 0u)
        return VISION_STATUS_INVALID_ARG;

    start = xTaskGetTickCount();
    timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    do
    {
        Vision_Poll();
        if (Vision_TakeResult(result))
            return VISION_STATUS_OK;
        vTaskDelay(pdMS_TO_TICKS(VISION_POLL_DELAY_MS));
    } while ((xTaskGetTickCount() - start) < timeout_ticks);

    request_pending = 0;
    (void)send_frame(VISION_MSG_CANCEL, expected_sequence, NULL, 0u);
    diagnostics.last_status = VISION_STATUS_TIMEOUT;
    return VISION_STATUS_TIMEOUT;
}

void Vision_InjectResult(const VisionResult_t *result)
{
    uint8_t next;

    if (result == NULL || result->mode > VISION_MODE_TREASURE ||
        result->direction > VISION_DIRECTION_RIGHT || result->confidence > 100u)
        return;
    next = (uint8_t)((inject_head + 1u) % VISION_INJECT_QUEUE_SIZE);
    if (next == inject_tail)
        inject_tail = (uint8_t)((inject_tail + 1u) % VISION_INJECT_QUEUE_SIZE);
    injected_results[inject_head] = *result;
    inject_head = next;
    request_pending = 0;
    diagnostics.last_sequence = result->sequence;
    diagnostics.last_status = VISION_STATUS_OK;
}

void Vision_ClearResults(void)
{
    result_pending = 0;
    request_pending = 0;
    inject_head = 0u;
    inject_tail = 0u;
    memset(&pending_result, 0, sizeof(pending_result));
}

const VisionDiagnostics_t *Vision_GetDiagnostics(void)
{
    return &diagnostics;
}

static VisionStatus_t scan_side(VisionDirection_t direction, VisionResult_t *result)
{
    uint8_t votes[4] = {0, 0, 0, 0};
    uint16_t confidence_sum[4] = {0, 0, 0, 0};
    uint8_t sample;
    uint8_t winner = VISION_COLOR_NONE;
    VisionResult_t current;
    VisionStatus_t status;

    for (sample = 0; sample < VISION_SIDE_SAMPLES; sample++)
    {
        status = Vision_Request(VISION_MODE_TRAFFIC_LIGHT, direction);
        if (status != VISION_STATUS_OK)
            return status;
        status = Vision_WaitResult(&current, VISION_RESULT_TIMEOUT_MS);
        if (status != VISION_STATUS_OK)
            return status;
        if (current.mode != VISION_MODE_TRAFFIC_LIGHT || current.direction != direction ||
            current.value > VISION_COLOR_RED || current.confidence < VISION_MIN_CONFIDENCE)
            continue;
        votes[current.value]++;
        confidence_sum[current.value] += current.confidence;
    }

    for (sample = VISION_COLOR_GREEN; sample <= VISION_COLOR_RED; sample++)
    {
        if (votes[sample] >= 2u && votes[sample] > votes[winner])
            winner = sample;
    }
    if (winner == VISION_COLOR_NONE)
        return VISION_STATUS_NO_RESULT;

    result->mode = VISION_MODE_TRAFFIC_LIGHT;
    result->direction = direction;
    result->value = winner;
    result->confidence = (uint8_t)(confidence_sum[winner] / votes[winner]);
    result->sequence = diagnostics.last_sequence;
    return VISION_STATUS_OK;
}

VisionStatus_t Vision_ScanTrafficPair(VisionPairResult_t *result)
{
    TickType_t start;
    VisionStatus_t status;

    if (result == NULL)
        return VISION_STATUS_INVALID_ARG;

    start = xTaskGetTickCount();
    Rudder_control(VISION_SERVO_LEFT, VISION_SERVO_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(VISION_SERVO_SETTLE_MS));
    status = scan_side(VISION_DIRECTION_LEFT, &result->left);
    if (status != VISION_STATUS_OK)
        goto cleanup;

    Rudder_control(VISION_SERVO_RIGHT, VISION_SERVO_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(VISION_SERVO_SETTLE_MS));
    status = scan_side(VISION_DIRECTION_RIGHT, &result->right);

cleanup:
    Rudder_control(VISION_SERVO_CENTER, VISION_SERVO_CHANNEL);
    if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(VISION_SCAN_TIMEOUT_MS))
        status = VISION_STATUS_TIMEOUT;
    if (status != VISION_STATUS_OK)
        Chassis_ForceStop(CHASSIS_STOP_VISION_TIMEOUT);
    return status;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t next;

    if (huart != &huart5)
        return;

    next = (uint8_t)((rx_head + 1u) & VISION_RX_MASK);
    if (next == rx_tail)
        rx_overflow_pending = 1;
    else
    {
        rx_buffer[rx_head & VISION_RX_MASK] = uart_rx_byte;
        rx_head = next;
    }
    (void)HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1u);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart5)
        return;
    diagnostics.last_status = VISION_STATUS_IO_ERROR;
    (void)HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1u);
}
