#include "serial_protocol.h"
#include "usart.h"
#include "oled.h"
#include "bluetooth.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

extern UART_HandleTypeDef huart1;

/* 全局解析结果 */
CLR_Result_t g_clr_result;

/* 调试用计数器 */
volatile uint32_t g_clr_rx_count = 0;
volatile uint32_t g_clr_rx_last_byte = 0;
volatile uint32_t g_clr_err_count = 0;

/* 私有接收缓冲 */
static uint8_t  s_rx_byte = 0;
static char     s_rx_buf[CLR_RX_PACKET_MAX_LEN];
static uint16_t s_rx_index = 0;
static uint8_t  s_frame_ready = 0;
static uint8_t  s_first_display = 0;

/* 上一次显示状态，用于状态切换时清屏 */
static uint8_t  s_last_no_cam = 0xFF;

/* 颜色名对应 Hzk[] 字模索引（0 结尾）
 * 颜色字在 oledfont.c 中索引：
 * 14:红 15:蓝 16:绿 17:黄 18:橙 19:紫 20:白 21:黑
 * 22:未 23:识 24:别 25:灰 26:棕 27:粉 28:玫 29:肉 30:青
 * 31:靛 32:金 33:银 34:米 35:卡 36:其 37:栗 38:酒 39:橄
 * 40:榄 41:墨 42:草 43:藏 44:天 45:湖 46:浅 47:深
 * 4: 色
 */
static const uint8_t s_color_name_0[]  = {22, 23, 24, 0};      /* 未识别 */
static const uint8_t s_color_name_1[]  = {14, 0};              /* 红 */
static const uint8_t s_color_name_2[]  = {17, 0};              /* 黄 */
static const uint8_t s_color_name_3[]  = {15, 0};              /* 蓝 */
static const uint8_t s_color_name_4[]  = {16, 0};              /* 绿 */
static const uint8_t s_color_name_5[]  = {20, 0};              /* 白 */
static const uint8_t s_color_name_6[]  = {21, 0};              /* 黑 */
static const uint8_t s_color_name_7[]  = {19, 0};              /* 紫 */
static const uint8_t s_color_name_8[]  = {18, 0};              /* 橙 */
static const uint8_t s_color_name_9[]  = {25, 0};              /* 灰 */
static const uint8_t s_color_name_10[] = {26, 0};              /* 棕 */
static const uint8_t s_color_name_11[] = {27, 0};              /* 粉 */
static const uint8_t s_color_name_12[] = {28, 14, 0};          /* 玫红 */
static const uint8_t s_color_name_13[] = {29, 4, 0};           /* 肉色 */
static const uint8_t s_color_name_14[] = {30, 0};              /* 青 */
static const uint8_t s_color_name_15[] = {15, 16, 0};          /* 蓝绿 */
static const uint8_t s_color_name_16[] = {31, 15, 0};          /* 靛蓝 */
static const uint8_t s_color_name_17[] = {32, 0};              /* 金 */
static const uint8_t s_color_name_18[] = {33, 0};              /* 银 */
static const uint8_t s_color_name_19[] = {34, 0};              /* 米 */
static const uint8_t s_color_name_20[] = {35, 36, 0};          /* 卡其 */
static const uint8_t s_color_name_21[] = {37, 0};              /* 栗 */
static const uint8_t s_color_name_22[] = {38, 14, 0};          /* 酒红 */
static const uint8_t s_color_name_23[] = {39, 40, 0};          /* 橄榄 */
static const uint8_t s_color_name_24[] = {41, 16, 0};          /* 墨绿 */
static const uint8_t s_color_name_25[] = {42, 16, 0};          /* 草绿 */
static const uint8_t s_color_name_26[] = {43, 15, 0};          /* 藏蓝 */
static const uint8_t s_color_name_27[] = {44, 15, 0};          /* 天蓝 */
static const uint8_t s_color_name_28[] = {45, 15, 0};          /* 湖蓝 */
static const uint8_t s_color_name_29[] = {46, 25, 0};          /* 浅灰 */
static const uint8_t s_color_name_30[] = {47, 25, 0};          /* 深灰 */

static const uint8_t *s_color_names[] = {
    s_color_name_0,  s_color_name_1,  s_color_name_2,  s_color_name_3,
    s_color_name_4,  s_color_name_5,  s_color_name_6,  s_color_name_7,
    s_color_name_8,  s_color_name_9,  s_color_name_10, s_color_name_11,
    s_color_name_12, s_color_name_13, s_color_name_14, s_color_name_15,
    s_color_name_16, s_color_name_17, s_color_name_18, s_color_name_19,
    s_color_name_20, s_color_name_21, s_color_name_22, s_color_name_23,
    s_color_name_24, s_color_name_25, s_color_name_26, s_color_name_27,
    s_color_name_28, s_color_name_29, s_color_name_30,
};

static const char *s_color_name_strings[] = {
    "Unknown", "Red", "Yellow", "Blue", "Green", "White", "Black",
    "Purple", "Orange", "Gray", "Brown", "Pink", "Rose", "Flesh",
    "Cyan", "Teal", "Indigo", "Gold", "Silver", "Beige", "Khaki",
    "Maroon", "Wine", "Olive", "DarkGreen", "GrassGreen", "Navy",
    "SkyBlue", "LakeBlue", "LightGray", "DarkGray",
};


static int  SerialProtocol_ParseInt(const char *s);
static void SerialProtocol_RestartReceiveIT(void);
static void SerialProtocol_ClearErrors(void);
static void SerialProtocol_DrawColor(uint8_t x, uint8_t y, int8_t color_id);
static uint8_t SerialProtocol_DrawColorList(uint8_t y, uint8_t count, const int8_t *colors);
static void SerialProtocol_ParseFrame(const char *frame);
static void SerialProtocol_FillSpaces(uint8_t x, uint8_t y, uint8_t to_x);
static const char *SerialProtocol_GetColorName(int8_t id);
static void SerialProtocol_SendToBluetooth(void);

void SerialProtocol_Init(void)
{
    memset(&g_clr_result, 0, sizeof(g_clr_result));
    g_clr_result.g1_color[0] = -1;
    g_clr_result.g1_color[1] = -1;
    g_clr_result.g2_color[0] = -1;
    g_clr_result.g2_color[1] = -1;
    g_clr_result.g2_color[2] = -1;
    g_clr_result.dir_x = 0;
    g_clr_result.dir_y = 0;

    s_rx_byte = 0;
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    s_rx_index = 0;
    s_frame_ready = 0;
    s_first_display = 0;
    s_last_no_cam = 0xFF;

    g_clr_rx_count = 0;
    g_clr_rx_last_byte = 0;
    g_clr_err_count = 0;
}

void SerialProtocol_StartReceiveIT(void)
{
    SerialProtocol_RestartReceiveIT();
}

static void SerialProtocol_RestartReceiveIT(void)
{
    if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1) != HAL_OK)
    {
        g_clr_err_count++;
    }
}

static void SerialProtocol_ClearErrors(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE)  ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_PE))
    {
        g_clr_err_count++;
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

void SerialProtocol_RxCallback(void)
{
    uint8_t byte = s_rx_byte;
    g_clr_rx_count++;
    g_clr_rx_last_byte = byte;

    if (s_rx_index < CLR_RX_PACKET_MAX_LEN - 1)
    {
        s_rx_buf[s_rx_index++] = (char)byte;
    }
    else
    {
        /* 超长帧，直接丢弃 */
        s_rx_index = 0;
    }

    if (byte == '\n')
    {
        s_rx_buf[s_rx_index] = '\0';
        s_frame_ready = 1;
        /* 保持指针位置，下一帧继续从 0 开始 */
        s_rx_index = 0;
    }

    SerialProtocol_RestartReceiveIT();
}

void SerialProtocol_OnUartError(void)
{
    SerialProtocol_ClearErrors();
    SerialProtocol_RestartReceiveIT();
}

static int SerialProtocol_ParseInt(const char *s)
{
    if (s == NULL || s[0] == '\0')
    {
        return -1;
    }

    while (*s != '\0' && isspace((unsigned char)*s))
    {
        s++;
    }

    if (*s == '\0')
    {
        return -1;
    }

    if (!isdigit((unsigned char)*s) && *s != '-' && *s != '+')
    {
        return -1;
    }

    long v = strtol(s, NULL, 10);
    if (v < -128)
    {
        v = -128;
    }
    if (v > 127)
    {
        v = 127;
    }
    return (int)v;
}

static void SerialProtocol_ParseFrame(const char *frame)
{
    char tmp[CLR_RX_PACKET_MAX_LEN];
    char *fields[16];
    uint8_t field_count = 0;
    uint8_t i;
    char *p;

    if (frame == NULL || frame[0] == '\0')
    {
        return;
    }

    strncpy(tmp, frame, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';

    /* 去掉行尾回车换行 */
    {
        size_t len = strlen(tmp);
        while (len > 0 && (tmp[len - 1] == '\r' || tmp[len - 1] == '\n'))
        {
            tmp[--len] = '\0';
        }
    }

    /* 按 '|' 切分 */
    p = tmp;
    while (field_count < 16)
    {
        char *start = p;
        while (*p != '\0' && *p != '|')
        {
            p++;
        }
        fields[field_count++] = start;
        if (*p == '\0')
        {
            break;
        }
        *p = '\0';
        p++;
    }

    /* 至少需要帧头 + 1 个字段 */
    if (field_count < 2)
    {
        return;
    }

    /* 检查帧头 */
    if (strcmp(fields[0], "CLR") != 0)
    {
        return;
    }

    /* 清空旧结果 */
    memset(&g_clr_result, 0, sizeof(g_clr_result));
    g_clr_result.g1_color[0] = -1;
    g_clr_result.g1_color[1] = -1;
    g_clr_result.g2_color[0] = -1;
    g_clr_result.g2_color[1] = -1;
    g_clr_result.g2_color[2] = -1;
    g_clr_result.dir_x = 0;
    g_clr_result.dir_y = 0;

    /* 解析 */
    if (field_count > 1)
    {
        g_clr_result.frame_no = (uint32_t)SerialProtocol_ParseInt(fields[1]);
    }
    if (field_count > 2)
    {
        g_clr_result.g1_color[0] = (int8_t)SerialProtocol_ParseInt(fields[2]);
    }
    if (field_count > 3)
    {
        g_clr_result.g1_color[1] = (int8_t)SerialProtocol_ParseInt(fields[3]);
    }
    if (field_count > 4)
    {
        g_clr_result.g2_color[0] = (int8_t)SerialProtocol_ParseInt(fields[4]);
    }
    if (field_count > 5)
    {
        g_clr_result.g2_color[1] = (int8_t)SerialProtocol_ParseInt(fields[5]);
    }
    if (field_count > 6)
    {
        g_clr_result.g2_color[2] = (int8_t)SerialProtocol_ParseInt(fields[6]);
    }
    if (field_count > 7)
    {
        g_clr_result.dir_x = (int8_t)SerialProtocol_ParseInt(fields[7]);
    }
    if (field_count > 8)
    {
        g_clr_result.dir_y = (int8_t)SerialProtocol_ParseInt(fields[8]);
    }

    for (i = 0; i < 4; i++)
    {
        if (field_count > (9 + i))
        {
            strncpy(g_clr_result.text[i], fields[9 + i], sizeof(g_clr_result.text[i]) - 1);
            g_clr_result.text[i][sizeof(g_clr_result.text[i]) - 1] = '\0';
        }
        else
        {
            g_clr_result.text[i][0] = '\0';
        }
    }

    g_clr_result.valid = 1;
}

void SerialProtocol_Task(void)
{
    if (s_frame_ready != 0)
    {
        char parse_buf[CLR_RX_PACKET_MAX_LEN];
        memcpy(parse_buf, s_rx_buf, sizeof(parse_buf));
        s_frame_ready = 0;
        SerialProtocol_ParseFrame(parse_buf);
        SerialProtocol_DisplayUpdate();
        SerialProtocol_SendToBluetooth();
    }
}

static void SerialProtocol_DrawColor(uint8_t x, uint8_t y, int8_t color_id)
{
    const uint8_t *indices;
    uint8_t i;

    if (color_id < 0 || color_id > 30)
    {
        OLED_ShowChar(x, y, '-', 16, 0);
        return;
    }

    indices = s_color_names[color_id];
    i = 0;
    while (indices[i] != 0)
    {
        OLED_ShowCHinese(x, y, indices[i], 0);
        x += 16;
        i++;
        if (x > 112)
        {
            break;
        }
    }
}

static uint8_t SerialProtocol_DrawColorList(uint8_t y, uint8_t count, const int8_t *colors)
{
    uint8_t x = 32;  /* 跳过 "G1:" 或 "G2:" */
    uint8_t i;

    for (i = 0; i < count; i++)
    {
        if (colors[i] < 0 || colors[i] > 30)
        {
            OLED_ShowChar(x, y, '-', 16, 0);
            x += 8;
        }
        else
        {
            SerialProtocol_DrawColor(x, y, colors[i]);
            x += 32;  /* 每个颜色最多两个汉字 */
        }

        if (i < count - 1 && x < 120)
        {
            OLED_ShowChar(x, y, ',', 16, 0);
            x += 8;
        }

        if (x > 112)
        {
            break;
        }
    }

    return x;
}

static void SerialProtocol_FillSpaces(uint8_t x, uint8_t y, uint8_t to_x)
{
    if (to_x > 128)
    {
        to_x = 128;
    }
    while (x < to_x)
    {
        OLED_ShowChar(x, y, ' ', 16, 0);
        x += 8;
    }
}

void SerialProtocol_DisplayUpdate(void)
{
    char num_buf[16];
    uint8_t no_cam = 0;
    uint8_t x;

    if (g_clr_result.frame_no == 0 &&
        g_clr_result.g1_color[0] < 0 && g_clr_result.g1_color[1] < 0 &&
        g_clr_result.g2_color[0] < 0 && g_clr_result.g2_color[1] < 0 && g_clr_result.g2_color[2] < 0)
    {
        no_cam = 1;
    }

    /* 只有"无摄像头"与"有数据"状态切换时才全屏清屏，避免每次刷新都闪烁 */
    if (no_cam != s_last_no_cam)
    {
        OLED_Clear();
        s_last_no_cam = no_cam;
    }

    if (no_cam)
    {
        OLED_ShowString(0, 0, "NO CAM", 16, 0);
        OLED_ShowString(0, 2, "Waiting...    ", 16, 0);
        s_first_display = 1;
        return;
    }

    /* G1 */
    OLED_ShowString(0, 0, "G1:", 16, 0);
    x = SerialProtocol_DrawColorList(0, 2, g_clr_result.g1_color);
    SerialProtocol_FillSpaces(x, 0, 128);

    /* G2 */
    OLED_ShowString(0, 2, "G2:", 16, 0);
    x = SerialProtocol_DrawColorList(2, 3, g_clr_result.g2_color);
    SerialProtocol_FillSpaces(x, 2, 128);

    /* Dir */
    OLED_ShowString(0, 4, "Dir:", 16, 0);
    {
        char dir_buf[4];
        char sx = (g_clr_result.dir_x < 0) ? '<' : ((g_clr_result.dir_x > 0) ? '>' : '-');
        char sy = (g_clr_result.dir_y < 0) ? 'v' : ((g_clr_result.dir_y > 0) ? '^' : '-');
        dir_buf[0] = sx;
        dir_buf[1] = sy;
        dir_buf[2] = '\0';
        OLED_ShowString(32, 4, "  ", 16, 0);
        OLED_ShowString(32, 4, dir_buf, 16, 0);
        SerialProtocol_FillSpaces(48, 4, 128);
    }

    /* Frame */
    OLED_ShowString(0, 6, "F:", 16, 0);
    snprintf(num_buf, sizeof(num_buf), "%lu", (unsigned long)g_clr_result.frame_no);
    OLED_ShowString(24, 6, "      ", 16, 0);
    OLED_ShowString(24, 6, num_buf, 16, 0);
    SerialProtocol_FillSpaces((uint8_t)(24 + strlen(num_buf) * 8), 6, 128);

    s_first_display = 1;
}


static const char *SerialProtocol_GetColorName(int8_t id)
{
    if (id < 0 || id > 30)
    {
        return "-";
    }
    return s_color_name_strings[id];
}

static void SerialProtocol_SendToBluetooth(void)
{
    static uint32_t s_last_bt_tx_tick = 0;
    static char     s_last_bt_tx[BT_TX_PACKET_MAX_LEN] = {0};
    char            tx_buf[BT_TX_PACKET_MAX_LEN];
    int             n;
    uint32_t        now;

    if (g_clr_result.valid == 0)
    {
        return;
    }

    now = HAL_GetTick();
    n = snprintf(tx_buf, sizeof(tx_buf),
                 "CLR:%lu|G1:%s,%s|G2:%s,%s,%s|Dir:%c%c\r\n",
                 (unsigned long)g_clr_result.frame_no,
                 SerialProtocol_GetColorName(g_clr_result.g1_color[0]),
                 SerialProtocol_GetColorName(g_clr_result.g1_color[1]),
                 SerialProtocol_GetColorName(g_clr_result.g2_color[0]),
                 SerialProtocol_GetColorName(g_clr_result.g2_color[1]),
                 SerialProtocol_GetColorName(g_clr_result.g2_color[2]),
                 (g_clr_result.dir_x < 0) ? '<' : ((g_clr_result.dir_x > 0) ? '>' : '-'),
                 (g_clr_result.dir_y < 0) ? 'v' : ((g_clr_result.dir_y > 0) ? '^' : '-'));

    if (n <= 0 || n >= (int)sizeof(tx_buf))
    {
        return;
    }

    /* 限制回传频率：至少间隔 100ms，且数据变化时才发送，避免阻塞主循环 */
    if ((uint32_t)(now - s_last_bt_tx_tick) < 100U)
    {
        return;
    }

    if (strcmp(tx_buf, s_last_bt_tx) == 0)
    {
        return;
    }

    strncpy(s_last_bt_tx, tx_buf, sizeof(s_last_bt_tx) - 1);
    s_last_bt_tx[sizeof(s_last_bt_tx) - 1] = '\0';
    s_last_bt_tx_tick = now;
    Bluetooth_SendString(tx_buf);
}
void SerialProtocol_ShowStatus(void)
{
    char buf[32];
    OLED_ShowString(0, 4, "UART1 OK", 16, 0);
    snprintf(buf, sizeof(buf), "RX:%lu", (unsigned long)g_clr_rx_count);
    OLED_ShowString(0, 6, buf, 16, 0);
}
