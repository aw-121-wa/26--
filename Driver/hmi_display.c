#include "hmi_display.h"

#include <stdio.h>
#include <string.h>

#include "map.h"
#include "usart.h"

#ifndef HMI_DISPLAY_UART
#define HMI_DISPLAY_UART huart8
#endif

/*
 * The current route starts from P2 and returns to P2. If the final rules route
 * uses P1 as home, override this macro to P1.
 */
#ifndef HMI_HOME_NODE
#define HMI_HOME_NODE P2
#endif

#define HMI_CMD_BUF_SIZE          160u
#define HMI_COMMAND_GAP_MS        25u
#define HMI_FULL_REFRESH_MS       2000u

#define HMI_COLOR_WHITE           65535u
#define HMI_COLOR_YELLOW          65504u
#define HMI_COLOR_GREEN           2016u
#define HMI_COLOR_CYAN            2047u

static HmiDisplayScores_t hmi_scores;
static uint32_t hmi_round_base_score;
static uint8_t hmi_dirty = 1u;
static uint8_t hmi_draw_index;
static uint8_t hmi_initialized;
static uint32_t hmi_last_send_ms;
static uint32_t hmi_last_full_refresh_ms;
static uint8_t hmi_tx_buf[HMI_CMD_BUF_SIZE];

static const char label_title[] = "\xBB\xFA\xC6\xF7\xC8\xCB\xC2\xC3\xD3\xCE\xBC\xC7\xC2\xBC";
static const char label_upright[] = "\xD6\xB1\xC1\xA2\xBE\xB0\xB5\xE3";
static const char label_platform_1_5[] = "1-5\xBA\xC5\xC6\xBD\xCC\xA8";
static const char label_platform_6[] = "6\xBA\xC5\xC6\xBD\xCC\xA8";
static const char label_platform_7[] = "7\xBA\xC5\xC6\xBD\xCC\xA8";
static const char label_platform_8[] = "8\xBA\xC5\xC6\xBD\xCC\xA8";
static const char label_home[] = "\xBB\xD8\xBC\xD2";
static const char label_total[] = "\xD7\xDC\xB7\xD6";
static const char unit_count[] = "\xB8\xF6";
static const char unit_times[] = "\xB4\xCE";
static const char unit_score[] = "\xB7\xD6";

static uint8_t hmi_uart_ready(void)
{
    return (HMI_DISPLAY_UART.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}

static void hmi_mark_dirty(void)
{
    hmi_dirty = 1u;
    hmi_draw_index = 0u;
}

static void hmi_send_command(const char *cmd)
{
    size_t len = strlen(cmd);

    if (len + 3u > HMI_CMD_BUF_SIZE)
        len = HMI_CMD_BUF_SIZE - 3u;

    memcpy(hmi_tx_buf, cmd, len);
    hmi_tx_buf[len++] = 0xffu;
    hmi_tx_buf[len++] = 0xffu;
    hmi_tx_buf[len++] = 0xffu;

    (void)HAL_UART_Transmit_IT(&HMI_DISPLAY_UART, hmi_tx_buf, (uint16_t)len);
}

static void hmi_format_text(char *buf, size_t size, uint16_t y,
                            uint16_t color, const char *text)
{
    (void)snprintf(buf, size, "xstr 0,%u,240,30,0,%u,0,0,1,1,\"%s\"",
                   (unsigned int)y, (unsigned int)color, text);
}

static void hmi_format_value(char *buf, size_t size, uint16_t y,
                             uint16_t color, const char *label,
                             uint32_t value, const char *unit)
{
    (void)snprintf(buf, size, "xstr 0,%u,240,30,0,%u,0,0,1,1,\"%s:%lu%s\"",
                   (unsigned int)y, (unsigned int)color, label,
                   (unsigned long)value, unit);
}

static uint8_t hmi_make_draw_command(uint8_t index, char *buf, size_t size)
{
    switch (index)
    {
    case 0u:
        (void)snprintf(buf, size, "cls 0");
        return 1u;
    case 1u:
        hmi_format_text(buf, size, 4u, HMI_COLOR_YELLOW, label_title);
        return 1u;
    case 2u:
        hmi_format_value(buf, size, 42u, HMI_COLOR_WHITE, label_upright,
                         hmi_scores.upright_spots, unit_count);
        return 1u;
    case 3u:
        hmi_format_value(buf, size, 76u, HMI_COLOR_WHITE, label_platform_1_5,
                         hmi_scores.platforms_1_to_5, unit_count);
        return 1u;
    case 4u:
        hmi_format_value(buf, size, 110u, HMI_COLOR_WHITE, label_platform_6,
                         hmi_scores.platform_6, unit_count);
        return 1u;
    case 5u:
        hmi_format_value(buf, size, 144u, HMI_COLOR_WHITE, label_platform_7,
                         hmi_scores.platform_7, unit_count);
        return 1u;
    case 6u:
        hmi_format_value(buf, size, 178u, HMI_COLOR_WHITE, label_platform_8,
                         hmi_scores.platform_8, unit_count);
        return 1u;
    case 7u:
        hmi_format_value(buf, size, 212u, HMI_COLOR_CYAN, label_home,
                         hmi_scores.home_returns, unit_times);
        return 1u;
    case 8u:
        hmi_format_value(buf, size, 258u, HMI_COLOR_GREEN, label_total,
                         hmi_scores.total_score, unit_score);
        return 1u;
    default:
        return 0u;
    }
}

static uint16_t hmi_record_platform(uint8_t node)
{
    switch (node)
    {
    case P1:
    case P2:
    case P3:
    case P4:
    case P5:
        hmi_scores.platforms_1_to_5++;
        return 30u;
    case P6:
        hmi_scores.platform_6++;
        return 30u;
    case P7:
        hmi_scores.platform_7++;
        return 90u;
    case P8:
        hmi_scores.platform_8++;
        return 150u;
    default:
        return 0u;
    }
}

void HmiDisplay_Init(void)
{
    hmi_initialized = 1u;
    hmi_mark_dirty();
}

void HmiDisplay_ResetScores(void)
{
    memset(&hmi_scores, 0, sizeof(hmi_scores));
    hmi_round_base_score = 0u;
    hmi_mark_dirty();
}

void HmiDisplay_RecordArrival(uint8_t node, uint8_t function)
{
    uint16_t base_score = 0u;

    if (function == View || function == View1)
    {
        hmi_scores.upright_spots++;
        base_score = (uint16_t)(base_score + 11u);
    }

    base_score = (uint16_t)(base_score + hmi_record_platform(node));

    if (base_score > 0u)
    {
        hmi_scores.total_score += base_score;
        hmi_round_base_score += base_score;
    }

    if (node == HMI_HOME_NODE)
    {
        hmi_scores.home_returns++;
        hmi_scores.total_score += hmi_round_base_score / 5u;
        hmi_round_base_score = 0u;
    }

    if (base_score > 0u || node == HMI_HOME_NODE)
        hmi_mark_dirty();
}

void HmiDisplay_Tick(void)
{
    uint32_t now = HAL_GetTick();
    char cmd[HMI_CMD_BUF_SIZE];

    if (!hmi_initialized)
        return;

    if (!hmi_dirty && (now - hmi_last_full_refresh_ms) >= HMI_FULL_REFRESH_MS)
        hmi_mark_dirty();

    if (!hmi_dirty)
        return;

    if (!hmi_uart_ready())
        return;

    if ((now - hmi_last_send_ms) < HMI_COMMAND_GAP_MS)
        return;

    if (!hmi_make_draw_command(hmi_draw_index, cmd, sizeof(cmd)))
    {
        hmi_dirty = 0u;
        hmi_draw_index = 0u;
        hmi_last_full_refresh_ms = now;
        return;
    }

    hmi_send_command(cmd);
    hmi_draw_index++;
    hmi_last_send_ms = now;
}

HmiDisplayScores_t HmiDisplay_GetScores(void)
{
    return hmi_scores;
}
