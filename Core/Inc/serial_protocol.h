#ifndef __SERIAL_PROTOCOL_H
#define __SERIAL_PROTOCOL_H

#include "main.h"

/* 串口帧最大长度（含结尾 \\n） */
#define CLR_RX_PACKET_MAX_LEN    256

/* 颜色 ID 定义（与 CLR 协议对应） */
#define CLR_COLOR_UNKNOWN        0
#define CLR_COLOR_RED            1
#define CLR_COLOR_YELLOW         2
#define CLR_COLOR_BLUE           3
#define CLR_COLOR_GREEN          4
#define CLR_COLOR_WHITE          5
#define CLR_COLOR_BLACK          6
#define CLR_COLOR_PURPLE         7
#define CLR_COLOR_ORANGE         8
#define CLR_COLOR_GRAY           9
#define CLR_COLOR_BROWN          10
#define CLR_COLOR_PINK           11
#define CLR_COLOR_ROSE           12
#define CLR_COLOR_FLESH          13
#define CLR_COLOR_CYAN           14
#define CLR_COLOR_TEAL           15
#define CLR_COLOR_INDIGO         16
#define CLR_COLOR_GOLD           17
#define CLR_COLOR_SILVER         18
#define CLR_COLOR_BEIGE          19
#define CLR_COLOR_KHAKI          20
#define CLR_COLOR_MAROON         21
#define CLR_COLOR_WINE           22
#define CLR_COLOR_OLIVE          23
#define CLR_COLOR_DARK_GREEN     24
#define CLR_COLOR_GRASS_GREEN    25
#define CLR_COLOR_NAVY           26
#define CLR_COLOR_SKY_BLUE       27
#define CLR_COLOR_LAKE_BLUE      28
#define CLR_COLOR_LIGHT_GRAY     29
#define CLR_COLOR_DARK_GRAY      30

/* 解析结果结构体 */
typedef struct {
    uint32_t frame_no;          /* 帧号 */
    int8_t   g1_color[2];       /* G1 颜色 ID（-1 表示未识别/空） */
    int8_t   g2_color[3];       /* G2 颜色 ID */
    int8_t   dir_x;             /* 水平方向：-1 左，0 中，1 右 */
    int8_t   dir_y;             /* 垂直方向：-1 下，0 中，1 上 */
    char     text[4][64];       /* text1..text4 原始文本 */
    uint8_t  valid;             /* 当前是否解析成功 */
} CLR_Result_t;

extern CLR_Result_t g_clr_result;
extern volatile uint32_t g_clr_rx_count;
extern volatile uint32_t g_clr_rx_last_byte;
extern volatile uint32_t g_clr_err_count;

void SerialProtocol_Init(void);
void SerialProtocol_StartReceiveIT(void);
void SerialProtocol_RxCallback(void);
void SerialProtocol_OnUartError(void);
void SerialProtocol_Task(void);
void SerialProtocol_DisplayUpdate(void);
void SerialProtocol_ShowStatus(void);

#endif /* __SERIAL_PROTOCOL_H */