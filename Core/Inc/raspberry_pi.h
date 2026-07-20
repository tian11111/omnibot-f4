#ifndef __RASPBERRY_PI_H
#define __RASPBERRY_PI_H

#include "main.h"

#define RPI_RX_PACKET_MAX_LEN    64

extern uint8_t g_rpi_data_ready;
extern char    g_rpi_color[16];
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
void RaspberryPi_ShowStatus(void);
void RaspberryPi_SendReady(void);
void RaspberryPi_SendEcho(void);
void RaspberryPi_SendString(const char *str);
void RaspberryPi_OnUartError(void);

#endif /* __RASPBERRY_PI_H */