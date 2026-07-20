#include "raspberry_pi.h"
#include "usart.h"
#include "oled.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;

uint8_t  g_rpi_data_ready = 0;
char     g_rpi_color[16] = {0};
uint16_t g_rpi_code1 = 0;
uint16_t g_rpi_code2 = 0;
uint8_t  s_rpi_rx_byte = 0;

/* live diagnostics */
volatile uint32_t g_rpi_rx_count = 0;
volatile uint32_t g_rpi_rx_last_byte = 0;
volatile uint32_t g_rpi_err_count = 0;

static char    s_rpi_rx_buf[RPI_RX_PACKET_MAX_LEN];
static uint8_t s_rpi_rx_state = 0;
static uint8_t s_rpi_rx_index = 0;
static char    s_rpi_last_raw[RPI_RX_PACKET_MAX_LEN];
static uint32_t s_ui_tick = 0;
static uint8_t  s_rpi_hold_display = 0; /* 1=keep color result on OLED */

/* echo queue: fill in ISR, flush in main loop (no blocking TX in IRQ) */
#define RPI_ECHO_Q_SIZE  128U
static volatile uint8_t  s_echo_q[RPI_ECHO_Q_SIZE];
static volatile uint16_t s_echo_w = 0;
static volatile uint16_t s_echo_r = 0;

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

static int RaspberryPi_ParseFrame(const char *frame)
{
    const char *p = frame;
    char color_tmp[16] = {0};
    int i = 0;
    long c1 = 0;
    long c2 = 0;
    char *end1 = NULL;
    char *end2 = NULL;

    if (p == NULL || *p == '\0')
    {
        return 0;
    }

    while (*p != '\0' && *p != ',' && i < (int)sizeof(color_tmp) - 1)
    {
        color_tmp[i++] = *p++;
    }
    color_tmp[i] = '\0';

    if (*p != ',')
    {
        return 0;
    }
    p++;

    c1 = strtol(p, &end1, 10);
    if (end1 == p || *end1 != ',')
    {
        return 0;
    }
    p = end1 + 1;

    c2 = strtol(p, &end2, 10);
    if (end2 == p)
    {
        return 0;
    }
    while (*end2 == ' ' || *end2 == '\t' || *end2 == '\r' || *end2 == '\n')
    {
        end2++;
    }
    if (*end2 != '\0')
    {
        return 0;
    }

    if (c1 < 0 || c1 > 65535 || c2 < 0 || c2 > 65535)
    {
        return 0;
    }

    strncpy(g_rpi_color, color_tmp, sizeof(g_rpi_color) - 1);
    g_rpi_color[sizeof(g_rpi_color) - 1] = '\0';
    g_rpi_code1 = (uint16_t)c1;
    g_rpi_code2 = (uint16_t)c2;
    return 1;
}

static void RaspberryPi_ProcessByte(uint8_t byte)
{
    g_rpi_rx_count++;
    g_rpi_rx_last_byte = byte;

    /* queue echo for main loop (ISR-safe) */
    RaspberryPi_EchoPush(byte);

    if (s_rpi_rx_state == 0)
    {
        if (byte == (uint8_t)'[')
        {
            s_rpi_rx_state = 1;
            s_rpi_rx_index = 0;
            memset(s_rpi_rx_buf, 0, sizeof(s_rpi_rx_buf));
        }
    }
    else if (s_rpi_rx_state == 1)
    {
        if (byte == (uint8_t)']')
        {
            s_rpi_rx_state = 0;
            s_rpi_rx_buf[s_rpi_rx_index] = '\0';
            strncpy(s_rpi_last_raw, s_rpi_rx_buf, sizeof(s_rpi_last_raw) - 1);
            s_rpi_last_raw[sizeof(s_rpi_last_raw) - 1] = '\0';

            if (RaspberryPi_ParseFrame(s_rpi_rx_buf))
            {
                g_rpi_data_ready = 1;
            }
            else
            {
                g_rpi_color[0] = '\0';
                g_rpi_code1 = 0;
                g_rpi_code2 = 0;
                g_rpi_data_ready = 2;
            }
        }
        else
        {
            if (s_rpi_rx_index < RPI_RX_PACKET_MAX_LEN - 1)
            {
                s_rpi_rx_buf[s_rpi_rx_index++] = (char)byte;
            }
            else
            {
                s_rpi_rx_state = 0;
                s_rpi_rx_index = 0;
                memset(s_rpi_rx_buf, 0, sizeof(s_rpi_rx_buf));
            }
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
    s_rpi_rx_state = 0;
    s_rpi_rx_index = 0;
    g_rpi_data_ready = 0;
    g_rpi_color[0] = '\0';
    g_rpi_code1 = 0;
    g_rpi_code2 = 0;
    g_rpi_rx_count = 0;
    g_rpi_rx_last_byte = 0;
    g_rpi_err_count = 0;
    s_rpi_hold_display = 0;
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

void RaspberryPi_DisplayUpdate(void)
{
    char line[20];
    char color_txt[8];

    /* once color/result is shown, stop READY status refresh */
    s_rpi_hold_display = 1;

    if (g_rpi_color[0] == '\0')
    {
        /* RAW fallback: bold title + bold payload */
        OLED_ShowStringBold(0, 0, "RAW", 16, 0);
        OLED_ShowStringBold(0, 2, s_rpi_last_raw, 16, 0);
        return;
    }

    memset(color_txt, 0, sizeof(color_txt));
    if (strcmp(g_rpi_color, "\xe7\xba\xa2") == 0 || strcmp(g_rpi_color, "RED") == 0 || strcmp(g_rpi_color, "red") == 0)
        strncpy(color_txt, "RED", sizeof(color_txt) - 1U);
    else if (strcmp(g_rpi_color, "\xe8\x93\x9d") == 0 || strcmp(g_rpi_color, "BLU") == 0 || strcmp(g_rpi_color, "blue") == 0)
        strncpy(color_txt, "BLU", sizeof(color_txt) - 1U);
    else if (strcmp(g_rpi_color, "\xe7\xbb\xbf") == 0 || strcmp(g_rpi_color, "GRN") == 0 || strcmp(g_rpi_color, "green") == 0)
        strncpy(color_txt, "GRN", sizeof(color_txt) - 1U);
    else if (strcmp(g_rpi_color, "\xe9\xbb\x84") == 0 || strcmp(g_rpi_color, "YLW") == 0 || strcmp(g_rpi_color, "yellow") == 0)
        strncpy(color_txt, "YLW", sizeof(color_txt) - 1U);
    else if (strcmp(g_rpi_color, "\xe6\xa9\x99") == 0 || strcmp(g_rpi_color, "ORG") == 0 || strcmp(g_rpi_color, "orange") == 0)
        strncpy(color_txt, "ORG", sizeof(color_txt) - 1U);
    else if (strcmp(g_rpi_color, "\xe7\xb4\xab") == 0 || strcmp(g_rpi_color, "PUR") == 0 || strcmp(g_rpi_color, "purple") == 0)
        strncpy(color_txt, "PUR", sizeof(color_txt) - 1U);
    else
    {
        memset(line, 0, sizeof(line));
        strncpy(line, g_rpi_color, 6);
        strncpy(color_txt, line, sizeof(color_txt) - 1U);
    }

    /*
     * Compact large layout on 128x64:
     *  - color: 2x font (16x32) on pages 0..3
     *  - code1 = 专属编码 (Hzk 8..11)
     *  - code2 = 争夺编码 (Hzk 12,13,10,11)
     */
    OLED_ShowString2x(0, 0, color_txt, 0);
    OLED_ShowString2x(1, 0, color_txt, 0);

    /* 专属编码 + 数字 */
    OLED_ShowCHinese(0, 4, 8, 0);   /* 专 */
    OLED_ShowCHinese(16, 4, 9, 0);  /* 属 */
    OLED_ShowCHinese(32, 4, 10, 0); /* 编 */
    OLED_ShowCHinese(48, 4, 11, 0); /* 码 */
    OLED_ShowNumBold(68, 4, g_rpi_code1, 5, 16, 0);
    OLED_ShowNumBold(69, 4, g_rpi_code1, 5, 16, 0);
    OLED_ShowNumBold(70, 4, g_rpi_code1, 5, 16, 0);

    /* 争夺编码 + 数字 */
    OLED_ShowCHinese(0, 6, 12, 0);  /* 争 */
    OLED_ShowCHinese(16, 6, 13, 0); /* 夺 */
    OLED_ShowCHinese(32, 6, 10, 0); /* 编 */
    OLED_ShowCHinese(48, 6, 11, 0); /* 码 */
    OLED_ShowNumBold(68, 6, g_rpi_code2, 5, 16, 0);
    OLED_ShowNumBold(69, 6, g_rpi_code2, 5, 16, 0);
    OLED_ShowNumBold(70, 6, g_rpi_code2, 5, 16, 0);
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
    RaspberryPi_SendString("[READY]\r\n");
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