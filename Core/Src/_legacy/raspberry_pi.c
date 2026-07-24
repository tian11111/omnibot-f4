#include "raspberry_pi.h"
#include "usart.h"
#include "oled.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

extern UART_HandleTypeDef huart1;

uint8_t  g_rpi_data_ready = 0;
char     g_rpi_line1_colors[2][16] = {0};
char     g_rpi_line2_colors[3][16] = {0};
uint8_t  g_rpi_line1_count = 0;
uint8_t  g_rpi_line2_count = 0;
uint16_t g_rpi_code1 = 0;
uint16_t g_rpi_code2 = 0;
uint8_t  s_rpi_rx_byte = 0;

/* live diagnostics */
volatile uint32_t g_rpi_rx_count = 0;
volatile uint32_t g_rpi_rx_last_byte = 0;
volatile uint32_t g_rpi_err_count = 0;

static char    s_rpi_rx_buf[RPI_RX_PACKET_MAX_LEN];
static uint8_t s_rpi_rx_index = 0;
static char    s_rpi_last_raw[RPI_RX_PACKET_MAX_LEN];
static uint32_t s_ui_tick = 0;
static uint8_t  s_rpi_hold_display = 0; /* 1=keep color result on OLED */
static uint8_t  s_rpi_first_display = 0; /* 1=already switched from READY to color screen */

/* 帧队列：OpenMV 可能连续发多个二维码，防止后一帧覆盖前一帧 */
#define RPI_FRAME_QUEUE_SIZE  2
typedef struct {
    char data[RPI_RX_PACKET_MAX_LEN];
} RpiFrame_t;
static RpiFrame_t s_frame_queue[RPI_FRAME_QUEUE_SIZE];
static volatile uint8_t s_frame_q_w = 0;
static volatile uint8_t s_frame_q_r = 0;

/* echo queue: fill in ISR, flush in main loop (no blocking TX in IRQ) */
#define RPI_ECHO_Q_SIZE  128U
static volatile uint8_t  s_echo_q[RPI_ECHO_Q_SIZE];
static volatile uint16_t s_echo_w = 0;
static volatile uint16_t s_echo_r = 0;

/* Chinese 16x16 font index in Hzk[] (oledfont.c) */
#define RPI_CHN_RED     14
#define RPI_CHN_BLUE    15
#define RPI_CHN_GREEN   16
#define RPI_CHN_YELLOW  17
#define RPI_CHN_ORANGE  18
#define RPI_CHN_PURPLE  19
#define RPI_CHN_WHITE   20
#define RPI_CHN_BLACK   21

static void RaspberryPi_EchoPush(uint8_t byte);
static void RaspberryPi_EchoFlush(void);
static void RaspberryPi_ForceHwInit(void);
static void RaspberryPi_ClearErrors(void);
static void RaspberryPi_ColorToUpper(char *dst, const char *src, size_t max_len);
static int  RaspberryPi_ColorToChineseIndex(const char *color);
static uint8_t RaspberryPi_ParseFrame(const char *frame);
static void RaspberryPi_ProcessByte(uint8_t byte);
static void RaspberryPi_RestartReceiveIT(void);
static void RaspberryPi_ShowColor(uint8_t x, uint8_t y, const char *color);
static void RaspberryPi_ClearLine(uint8_t y);
static void RaspberryPi_DisplayLine(uint8_t y, uint8_t count, char colors[][16]);

static void RaspberryPi_EchoPush(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_echo_w + 1U) % RPI_ECHO_Q_SIZE);
    if (next != s_echo_r)
    {
        s_echo_q[s_echo_w] = byte;
        s_echo_w = next;
    }
}

static void RaspberryPi_EchoFlush(void)
{
    while (s_echo_r != s_echo_w)
    {
        uint8_t byte = s_echo_q[s_echo_r];
        s_echo_r = (uint16_t)((s_echo_r + 1U) % RPI_ECHO_Q_SIZE);
        (void)HAL_UART_Transmit(&huart1, &byte, 1, 20);
    }
}

static void RaspberryPi_ForceHwInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* re-apply AF7 on PA9(TX)/PA10(RX) */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* no DMA request bits; keep UE/TE/RE on */
    CLEAR_BIT(huart1.Instance->CR3, USART_CR3_DMAR | USART_CR3_DMAT);
    SET_BIT(huart1.Instance->CR1, USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);

    /* clear sticky error by SR+DR read sequence */
    {
        volatile uint32_t sr = huart1.Instance->SR;
        volatile uint32_t dr = huart1.Instance->DR;
        (void)sr;
        (void)dr;
    }
}

static void RaspberryPi_ClearErrors(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE)  ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_PE))
    {
        g_rpi_err_count++;
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        __HAL_UART_CLEAR_NEFLAG(&huart1);
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        {
            volatile uint32_t dr = huart1.Instance->DR;
            (void)dr;
        }
    }
}

static void RaspberryPi_ColorToUpper(char *dst, const char *src, size_t max_len)
{
    size_t i;
    for (i = 0; i < max_len - 1 && src[i] != '\0'; i++)
    {
        dst[i] = (char)toupper((unsigned char)src[i]);
    }
    dst[i] = '\0';
}

static int RaspberryPi_ColorToChineseIndex(const char *color)
{
    if (color == NULL || color[0] == '\0')
    {
        return -1;
    }

    if (strcmp(color, "RED") == 0)    return RPI_CHN_RED;
    if (strcmp(color, "BLUE") == 0)   return RPI_CHN_BLUE;
    if (strcmp(color, "GREEN") == 0)  return RPI_CHN_GREEN;
    if (strcmp(color, "YELLOW") == 0) return RPI_CHN_YELLOW;
    if (strcmp(color, "ORANGE") == 0) return RPI_CHN_ORANGE;
    if (strcmp(color, "PURPLE") == 0) return RPI_CHN_PURPLE;
    if (strcmp(color, "WHITE") == 0)  return RPI_CHN_WHITE;
    if (strcmp(color, "BLACK") == 0)  return RPI_CHN_BLACK;

    return -1;
}

static uint8_t RaspberryPi_ParseFrame(const char *frame)
{
    char tmp[RPI_RX_PACKET_MAX_LEN];
    char *tokens[RPI_MAX_COLORS];
    size_t len;
    uint8_t i;
    uint8_t color_count = 0;
    uint8_t refresh_mask = 0U;

    if (frame == NULL || *frame == '\0')
    {
        return 0U;
    }

    strncpy(tmp, frame, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';

    /* 去掉 \r\n */
    len = strlen(tmp);
    while (len > 0 && (tmp[len - 1] == '\r' || tmp[len - 1] == '\n'))
    {
        tmp[--len] = '\0';
    }

    /* 按逗号分割，最多取 3 个颜色 */
    tokens[0] = tmp;
    for (i = 1; i < RPI_MAX_COLORS; i++)
    {
        tokens[i] = NULL;
    }

    for (i = 0; i < RPI_MAX_COLORS; i++)
    {
        if (tokens[i] == NULL)
        {
            break;
        }

        char *comma = strchr(tokens[i], ',');
        if (comma != NULL)
        {
            *comma = '\0';
            if (i + 1 < RPI_MAX_COLORS)
            {
                tokens[i + 1] = comma + 1;
            }
        }

        /* 去掉前后空格 */
        char *p = tokens[i];
        while (*p == ' ' || *p == '\t') p++;
        char *end = p + strlen(p);
        while (end > p && (*(end - 1) == ' ' || *(end - 1) == '\t')) end--;
        *end = '\0';

        tokens[i] = p;
    }

    /* 统计有效颜色 */
    for (i = 0; i < RPI_MAX_COLORS; i++)
    {
        if (tokens[i] != NULL && tokens[i][0] != '\0')
        {
            color_count++;
        }
        else
        {
            break;
        }
    }

    /* 根据颜色数量分流到上行（2 个）或下行（3 个），保持另一行不变 */
    if (color_count == 2)
    {
        memset(g_rpi_line1_colors, 0, sizeof(g_rpi_line1_colors));
        for (i = 0; i < color_count && i < 2; i++)
        {
            RaspberryPi_ColorToUpper(g_rpi_line1_colors[i], tokens[i], sizeof(g_rpi_line1_colors[i]));
        }
        g_rpi_line1_count = color_count;
        refresh_mask = 0x01U;
    }
    else if (color_count == 3)
    {
        memset(g_rpi_line2_colors, 0, sizeof(g_rpi_line2_colors));
        for (i = 0; i < color_count && i < 3; i++)
        {
            RaspberryPi_ColorToUpper(g_rpi_line2_colors[i], tokens[i], sizeof(g_rpi_line2_colors[i]));
        }
        g_rpi_line2_count = color_count;
        refresh_mask = 0x02U;
    }
    else
    {
        /* 非 2/3 个颜色的帧保持原显示不变 */
    }

    g_rpi_code1 = 0;
    g_rpi_code2 = 0;

    return refresh_mask;
}

static void RaspberryPi_ProcessByte(uint8_t byte)
{
    g_rpi_rx_count++;
    g_rpi_rx_last_byte = byte;

    /* queue echo for main loop (ISR-safe) */
    RaspberryPi_EchoPush(byte);

    if (byte == '\r')
    {
        /* 忽略 \r，等 \n 再结算 */
        return;
    }

    if (byte == '\n')
    {
        if (s_rpi_rx_index > 0)
        {
            s_rpi_rx_buf[s_rpi_rx_index] = '\0';
            strncpy(s_rpi_last_raw, s_rpi_rx_buf, sizeof(s_rpi_last_raw) - 1);
            s_rpi_last_raw[sizeof(s_rpi_last_raw) - 1] = '\0';

            /* 把完整帧存入队列 */
            uint8_t next = (uint8_t)((s_frame_q_w + 1U) % RPI_FRAME_QUEUE_SIZE);
            if (next != s_frame_q_r)
            {
                strncpy(s_frame_queue[s_frame_q_w].data, s_rpi_rx_buf, sizeof(s_frame_queue[s_frame_q_w].data) - 1);
                s_frame_queue[s_frame_q_w].data[sizeof(s_frame_queue[s_frame_q_w].data) - 1] = '\0';
                s_frame_q_w = next;
            }

            g_rpi_data_ready = 1;
        }
        s_rpi_rx_index = 0;
        memset(s_rpi_rx_buf, 0, sizeof(s_rpi_rx_buf));
    }
    else
    {
        if (s_rpi_rx_index < RPI_RX_PACKET_MAX_LEN - 1)
        {
            s_rpi_rx_buf[s_rpi_rx_index++] = (char)byte;
        }
        else
        {
            /* 长度超了，复位 */
            s_rpi_rx_index = 0;
            memset(s_rpi_rx_buf, 0, sizeof(s_rpi_rx_buf));
        }
    }
}

static void RaspberryPi_RestartReceiveIT(void)
{
    CLEAR_BIT(huart1.Instance->CR3, USART_CR3_DMAR | USART_CR3_DMAT);
    SET_BIT(huart1.Instance->CR1, USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);

    if (huart1.RxState != HAL_UART_STATE_READY)
    {
        (void)HAL_UART_AbortReceive(&huart1);
    }

    if (HAL_UART_Receive_IT(&huart1, &s_rpi_rx_byte, 1) != HAL_OK)
    {
        (void)HAL_UART_AbortReceive(&huart1);
        huart1.RxState = HAL_UART_STATE_READY;
        (void)HAL_UART_Receive_IT(&huart1, &s_rpi_rx_byte, 1);
    }
}

void RaspberryPi_Init(void)
{
    memset(s_rpi_rx_buf, 0, sizeof(s_rpi_rx_buf));
    memset(s_rpi_last_raw, 0, sizeof(s_rpi_last_raw));
    s_rpi_rx_index = 0;
    s_frame_q_w = 0;
    s_frame_q_r = 0;
    g_rpi_data_ready = 0;
    memset(g_rpi_line1_colors, 0, sizeof(g_rpi_line1_colors));
    memset(g_rpi_line2_colors, 0, sizeof(g_rpi_line2_colors));
    g_rpi_line1_count = 0;
    g_rpi_line2_count = 0;
    g_rpi_code1 = 0;
    g_rpi_code2 = 0;
    g_rpi_rx_count = 0;
    g_rpi_rx_last_byte = 0;
    g_rpi_err_count = 0;
    s_rpi_hold_display = 0;
    s_rpi_first_display = 0;
    s_ui_tick = 0;
    s_echo_w = 0;
    s_echo_r = 0;

    (void)HAL_UART_AbortReceive(&huart1);
    (void)HAL_UART_AbortTransmit(&huart1);
    RaspberryPi_ForceHwInit();
    RaspberryPi_ClearErrors();
}

void RaspberryPi_StartReceiveIT(void)
{
    RaspberryPi_ForceHwInit();
    RaspberryPi_ClearErrors();
    RaspberryPi_RestartReceiveIT();
}

void RaspberryPi_RxCallback(void)
{
    RaspberryPi_ProcessByte(s_rpi_rx_byte);

    if (HAL_UART_Receive_IT(&huart1, &s_rpi_rx_byte, 1) != HAL_OK)
    {
        RaspberryPi_RestartReceiveIT();
    }
}

void RaspberryPi_ShowStatus(void)
{
    char line[22];
    uint8_t pa10 = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);

    snprintf(line, sizeof(line), "RX:%lu E:%lu",
             (unsigned long)g_rpi_rx_count,
             (unsigned long)g_rpi_err_count);
    OLED_ShowString(0, 0, "READY   ", 16, 0);
    OLED_ShowString(0, 2, line, 16, 0);
    snprintf(line, sizeof(line), "P10:%u L:%02lX",
             (unsigned)pa10,
             (unsigned long)g_rpi_rx_last_byte);
    OLED_ShowString(0, 4, line, 16, 0);
}

void RaspberryPi_Task(void)
{
    uint32_t now = HAL_GetTick();

    /* flush per-byte echo from ISR queue (only received content) */
    RaspberryPi_EchoFlush();

    /* keep receiver alive if HAL dropped out of BUSY_RX */
    if (huart1.RxState != HAL_UART_STATE_BUSY_RX)
    {
        RaspberryPi_RestartReceiveIT();
    }

    /* safety: if RXNE stuck without IRQ, drain a few bytes */
    {
        uint32_t guard = 0;
        while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) &&
               (huart1.RxState != HAL_UART_STATE_BUSY_RX) &&
               guard < 16U)
        {
            uint8_t byte = (uint8_t)(huart1.Instance->DR & 0xFFU);
            RaspberryPi_ProcessByte(byte);
            guard++;
        }
    }

    /* do not ClearErrors here: reading DR would race with RX IRQ */
    SET_BIT(huart1.Instance->CR1, USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
    CLEAR_BIT(huart1.Instance->CR3, USART_CR3_DMAR | USART_CR3_DMAT);

    /* OLED live status ~5Hz if no frame parsed yet */
    if ((now - s_ui_tick) >= 200U)
    {
        s_ui_tick = now;
        if (s_rpi_hold_display == 0U)
        {
            RaspberryPi_ShowStatus();
        }
    }

    /* no periodic DBG; only echo received payload */
}

static void RaspberryPi_ShowColor(uint8_t x, uint8_t y, const char *color)
{
    int idx = RaspberryPi_ColorToChineseIndex(color);
    if (idx >= 0)
    {
        OLED_ShowCHinese(x, y, (uint8_t)idx, 0);
    }
    else
    {
        /* 未知颜色：用 ASCII 显示原始文本 */
        OLED_ShowString(x, y, color, 16, 0);
    }
}

static void RaspberryPi_ClearLine(uint8_t y)
{
    uint8_t page, col;
    for (page = 0; page < 2; page++)
    {
        OLED_Set_Pos(0, (uint8_t)(y + page));
        for (col = 0; col < 128; col++)
        {
            OLED_WR_DATA(0);
        }
    }
}

static void RaspberryPi_DisplayLine(uint8_t y, uint8_t count, char colors[][16])
{
    uint8_t x;
    uint8_t i;

    if (count == 0U)
    {
        return;
    }

    x = (uint8_t)((128 - count * 16) / 2);
    for (i = 0; i < count; i++)
    {
        RaspberryPi_ShowColor((uint8_t)(x + i * 16), y, colors[i]);
    }
}

void RaspberryPi_ProcessFrames(void)
{
    uint8_t refresh_mask = 0U;
    uint8_t processed = 0U;

    /* 处理队列中所有已收到的帧，累计需要刷新的行 */
    while (s_frame_q_r != s_frame_q_w)
    {
        refresh_mask |= RaspberryPi_ParseFrame(s_frame_queue[s_frame_q_r].data);
        s_frame_q_r = (uint8_t)((s_frame_q_r + 1U) % RPI_FRAME_QUEUE_SIZE);
        processed = 1U;
    }

    if (processed)
    {
        g_rpi_data_ready = 0;
        s_rpi_hold_display = 1U; /* 停止 READY 状态刷新 */

        if (s_rpi_first_display == 0U)
        {
            /* 首次收到数据：整屏清屏并显示两行 */
            OLED_Clear();
            s_rpi_first_display = 1U;
            RaspberryPi_DisplayLine(0, g_rpi_line1_count, g_rpi_line1_colors);
            RaspberryPi_DisplayLine(3, g_rpi_line2_count, g_rpi_line2_colors);
        }
        else
        {
            /* 之后只刷新变化的那一行，避免全屏闪烁 */
            if (refresh_mask & 0x01U)
            {
                RaspberryPi_ClearLine(0);
                RaspberryPi_DisplayLine(0, g_rpi_line1_count, g_rpi_line1_colors);
            }
            if (refresh_mask & 0x02U)
            {
                RaspberryPi_ClearLine(3);
                RaspberryPi_DisplayLine(3, g_rpi_line2_count, g_rpi_line2_colors);
            }
        }
    }
}

void RaspberryPi_DisplayUpdate(void)
{
    /* once color/result is shown, stop READY status refresh */
    s_rpi_hold_display = 1;

    if (g_rpi_line1_count == 0 && g_rpi_line2_count == 0)
    {
        /* 解析失败：显示 ERR + 原始内容 */
        OLED_ShowString(0, 0, "ERR", 16, 0);
        OLED_ShowString(0, 2, s_rpi_last_raw, 16, 0);
        return;
    }

    /* 上行：2 个颜色，居中显示 */
    if (g_rpi_line1_count > 0)
    {
        RaspberryPi_DisplayLine(0, g_rpi_line1_count, g_rpi_line1_colors);
    }

    /* 下行：3 个颜色，居中显示 */
    if (g_rpi_line2_count > 0)
    {
        RaspberryPi_DisplayLine(3, g_rpi_line2_count, g_rpi_line2_colors);
    }
}

void RaspberryPi_SendString(const char *str)
{
    if (str == NULL)
    {
        return;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)str, (uint16_t)strlen(str), 100);
}

void RaspberryPi_SendReady(void)
{
    RaspberryPi_SendString("READY\r\n");
}

void RaspberryPi_SendEcho(void)
{
    /* received bytes are already echoed one-by-one in RaspberryPi_Task */
    (void)s_rpi_last_raw;
}

void RaspberryPi_OnUartError(void)
{
    g_rpi_err_count++;
    /* clear flags without double-count */
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    {
        volatile uint32_t dr = huart1.Instance->DR;
        (void)dr;
    }
    RaspberryPi_ForceHwInit();
    RaspberryPi_RestartReceiveIT();
}