#ifndef __RASPBERRY_PI_H
#define __RASPBERRY_PI_H

#include "main.h"

#define RPI_RX_PACKET_MAX_LEN    64
#define RPI_MAX_COLORS           3

extern uint8_t g_rpi_data_ready;

/* 上行 2 个颜色，下行 3 个颜色 */
extern char    g_rpi_line1_colors[2][16];
extern char    g_rpi_line2_colors[3][16];
extern uint8_t g_rpi_line1_count;
extern uint8_t g_rpi_line2_count;

extern uint16_t g_rpi_code1;
extern uint16_t g_rpi_code2;
extern uint8_t s_rpi_rx_byte;
extern volatile uint32_t g_rpi_rx_count;
extern volatile uint32_t g_rpi_rx_last_byte;
extern volatile uint32_t g_rpi_err_count;

void RaspberryPi_Init(void);
void RaspberryPi_StartReceiveIT(void);
void RaspberryPi_Task(void);
void RaspberryPi_RxCallback(void);
void RaspberryPi_DisplayUpdate(void);
void RaspberryPi_ProcessFrames(void);
void RaspberryPi_ShowStatus(void);
void RaspberryPi_SendReady(void);
void RaspberryPi_SendEcho(void);
void RaspberryPi_SendString(const char *str);
void RaspberryPi_OnUartError(void);

#endif /* __RASPBERRY_PI_H */