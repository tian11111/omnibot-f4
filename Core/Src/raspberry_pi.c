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

static char    s_rpi_rx_buf[RPI_RX_PACKET_MAX_LEN];
static uint8_t s_rpi_rx_state = 0;
static uint8_t s_rpi_rx_index = 0;

void RaspberryPi_Init(void)
{
    memset(s_rpi_rx_buf, 0, sizeof(s_rpi_rx_buf));
    s_rpi_rx_state = 0;
    s_rpi_rx_index = 0;
    g_rpi_data_ready = 0;
}

void RaspberryPi_StartReceiveIT(void)
{
    HAL_UART_Receive_IT(&huart1, &s_rpi_rx_byte, 1);
}

void RaspberryPi_RxCallback(void)
{
    if (s_rpi_rx_state == 0)
    {
        if (s_rpi_rx_byte == '[' && g_rpi_data_ready == 0)
        {
            s_rpi_rx_state = 1;
            s_rpi_rx_index = 0;
            memset(s_rpi_rx_buf, 0, sizeof(s_rpi_rx_buf));
        }
    }
    else if (s_rpi_rx_state == 1)
    {
        if (s_rpi_rx_byte == ']')
        {
            s_rpi_rx_state = 0;
            s_rpi_rx_buf[s_rpi_rx_index] = '\0';

            char color_tmp[16] = {0};
            int c1 = 0, c2 = 0;
            if (sscanf(s_rpi_rx_buf, "%15[^,],%d,%d", color_tmp, &c1, &c2) == 3)
            {
                strncpy(g_rpi_color, color_tmp, sizeof(g_rpi_color) - 1);
                g_rpi_color[sizeof(g_rpi_color) - 1] = '\0';
                g_rpi_code1 = (uint16_t)c1;
                g_rpi_code2 = (uint16_t)c2;
                g_rpi_data_ready = 1;
            }
        }
        else
        {
            if (s_rpi_rx_index < RPI_RX_PACKET_MAX_LEN - 1)
            {
                s_rpi_rx_buf[s_rpi_rx_index++] = (char)s_rpi_rx_byte;
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



void RaspberryPi_DisplayUpdate(void)
{
    /* colour -> 3-letter abbreviation */
    if (strcmp(g_rpi_color, "\xe7\xba\xa2") == 0)       /* red */
        OLED_ShowString(0, 0, "RED", 16, 0);
    else if (strcmp(g_rpi_color, "\xe8\x93\x9d") == 0)  /* blue */
        OLED_ShowString(0, 0, "BLU", 16, 0);
    else if (strcmp(g_rpi_color, "\xe7\xbb\xbf") == 0)  /* green */
        OLED_ShowString(0, 0, "GRN", 16, 0);
    else if (strcmp(g_rpi_color, "\xe9\xbb\x84") == 0)  /* yellow: skipped per user request */
        return;
    else if (strcmp(g_rpi_color, "\xe6\xa9\x99") == 0)  /* orange */
        OLED_ShowString(0, 0, "ORG", 16, 0);
    else if (strcmp(g_rpi_color, "\xe7\xb4\xab") == 0)  /* purple */
        OLED_ShowString(0, 0, "PUR", 16, 0);
    else
    {
        char abbrev[4] = {0};
        strncpy(abbrev, g_rpi_color, 3);
        OLED_ShowString(0, 0, abbrev, 16, 0);
    }

    OLED_ShowNum(0, 2, g_rpi_code1, 5, 16, 0);
    OLED_ShowNum(0, 4, g_rpi_code2, 5, 16, 0);
}
void RaspberryPi_SendReady(void)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)"[READY]\r\n", 9, 100);
}
