#ifndef __RASPBERRY_PI_H
#define __RASPBERRY_PI_H

#include "main.h"

#define RPI_RX_PACKET_MAX_LEN    256U
#define RPI_MAX_ITEMS            8U
#define RPI_MAX_FIELDS           3U
#define RPI_FIELD_MAX_LEN        16U

typedef struct
{
    uint8_t field_count;
    char fields[RPI_MAX_FIELDS][RPI_FIELD_MAX_LEN];
} RaspberryPi_Item;

extern uint8_t g_rpi_data_ready;
extern uint8_t g_rpi_item_count;
extern RaspberryPi_Item g_rpi_items[RPI_MAX_ITEMS];
extern char g_rpi_last_frame[RPI_RX_PACKET_MAX_LEN];
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
