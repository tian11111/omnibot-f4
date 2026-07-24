#include "raspberry_pi.h"
#include "usart.h"
#include "oled.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;

uint8_t  g_rpi_data_ready = 0;
uint8_t  g_rpi_item_count = 0;
RaspberryPi_Item g_rpi_items[RPI_MAX_ITEMS];
char     g_rpi_last_frame[RPI_RX_PACKET_MAX_LEN] = {0};
char     g_rpi_color[16] = {0};
uint16_t g_rpi_code1 = 0;
uint16_t g_rpi_code2 = 0;
uint8_t  s_rpi_rx_byte = 0;

/* live diagnostics */
volatile uint32_t g_rpi_rx_count = 0;
volatile uint32_t g_rpi_rx_last_byte = 0;
volatile uint32_t g_rpi_err_count = 0;

static char     s_rpi_rx_buf[RPI_RX_PACKET_MAX_LEN];
static char     s_rpi_pending_frame[RPI_RX_PACKET_MAX_LEN];
static uint16_t s_rpi_rx_index = 0;
static volatile uint8_t s_rpi_frame_pending = 0;
static uint8_t  s_rpi_rx_overflow = 0;
static uint32_t s_ui_tick = 0;
static uint8_t  s_rpi_hold_display = 0; /* 1=keep QR result on OLED */
static uint8_t  s_rpi_display_page = 0;
static uint32_t s_rpi_display_tick = 0;

#define RPI_OLED_ITEMS_PER_PAGE  4U
#define RPI_OLED_PAGE_PERIOD_MS  1500U

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

static uint8_t RaspberryPi_IsSpace(char ch)
{
    return (uint8_t)((ch == ' ') || (ch == '\t') || (ch == '\r') || (ch == '\n'));
}

static char *RaspberryPi_Trim(char *text)
{
    char *end;

    while ((*text != '\0') && RaspberryPi_IsSpace(*text))
    {
        text++;
    }

    end = text + strlen(text);
    while ((end > text) && RaspberryPi_IsSpace(end[-1]))
    {
        end--;
    }
    *end = '\0';
    return text;
}

static int RaspberryPi_ParseItem(char *content, RaspberryPi_Item *item)
{
    char *field_start = content;

    memset(item, 0, sizeof(*item));

    while (1)
    {
        char *comma;
        char *field;
        size_t field_len;

        if (item->field_count >= RPI_MAX_FIELDS)
        {
            return 0;
        }

        comma = strchr(field_start, ',');
        if (comma != NULL)
        {
            *comma = '\0';
        }

        field = RaspberryPi_Trim(field_start);
        field_len = strlen(field);
        if ((field_len == 0U) || (field_len >= RPI_FIELD_MAX_LEN))
        {
            return 0;
        }

        memcpy(item->fields[item->field_count], field, field_len + 1U);
        item->field_count++;

        if (comma == NULL)
        {
            break;
        }
        field_start = comma + 1;
    }

    return 1;
}

static int RaspberryPi_ParseU16(const char *text, uint16_t *value)
{
    char *end = NULL;
    long parsed = strtol(text, &end, 10);

    if ((end == text) || (*end != '\0') || (parsed < 0L) || (parsed > 65535L))
    {
        return 0;
    }

    *value = (uint16_t)parsed;
    return 1;
}

static int RaspberryPi_ParseFrame(char *frame)
{
    RaspberryPi_Item parsed_items[RPI_MAX_ITEMS];
    char *cursor = RaspberryPi_Trim(frame);
    uint8_t item_count = 0;
    uint8_t i;

    memset(parsed_items, 0, sizeof(parsed_items));

    /*
     * OpenMV sends one bare comma-separated QR payload per line. Accept the
     * legacy bracketed "[Red,Blue]" form as well.
     */
    if (*cursor != '[')
    {
        if ((*cursor == '\0') || (strchr(cursor, ';') != NULL))
        {
            return 0;
        }

        if (!RaspberryPi_ParseItem(cursor, &parsed_items[0]))
        {
            return 0;
        }
        item_count = 1U;
    }
    else
    {
        while (*cursor != '\0')
        {
            char *content;
            char *close;

            if ((item_count >= RPI_MAX_ITEMS) || (*cursor != '['))
            {
                return 0;
            }

            content = cursor + 1;
            close = strchr(content, ']');
            if (close == NULL)
            {
                return 0;
            }
            *close = '\0';

            if (!RaspberryPi_ParseItem(content, &parsed_items[item_count]))
            {
                return 0;
            }
            item_count++;

            cursor = close + 1;
            while ((*cursor != '\0') && RaspberryPi_IsSpace(*cursor))
            {
                cursor++;
            }

            if (*cursor == '\0')
            {
                break;
            }
            if (*cursor != ';')
            {
                return 0;
            }

            cursor++;
            while ((*cursor != '\0') && RaspberryPi_IsSpace(*cursor))
            {
                cursor++;
            }
            if (*cursor == '\0')
            {
                return 0;
            }
        }
    }

    if (item_count == 0U)
    {
        return 0;
    }

    memset(g_rpi_items, 0, sizeof(g_rpi_items));
    memcpy(g_rpi_items, parsed_items, sizeof(parsed_items));
    g_rpi_item_count = item_count;

    /* Backward-compatible view: expose the first valid color + two-number item. */
    g_rpi_color[0] = '\0';
    g_rpi_code1 = 0U;
    g_rpi_code2 = 0U;
    for (i = 0U; i < item_count; i++)
    {
        uint16_t code1;
        uint16_t code2;

        if ((parsed_items[i].field_count == 3U) &&
            RaspberryPi_ParseU16(parsed_items[i].fields[1], &code1) &&
            RaspberryPi_ParseU16(parsed_items[i].fields[2], &code2))
        {
            strncpy(g_rpi_color, parsed_items[i].fields[0], sizeof(g_rpi_color) - 1U);
            g_rpi_color[sizeof(g_rpi_color) - 1U] = '\0';
            g_rpi_code1 = code1;
            g_rpi_code2 = code2;
            break;
        }
    }

    return 1;
}

static void RaspberryPi_ProcessByte(uint8_t byte)
{
    g_rpi_rx_count++;
    g_rpi_rx_last_byte = byte;

    if (byte == (uint8_t)'\r')
    {
        return;
    }

    if (byte == (uint8_t)'\n')
    {
        if (s_rpi_rx_overflow != 0U)
        {
            s_rpi_rx_overflow = 0U;
            s_rpi_rx_index = 0U;
            g_rpi_err_count++;
            return;
        }

        if (s_rpi_rx_index == 0U)
        {
            return;
        }

        s_rpi_rx_buf[s_rpi_rx_index] = '\0';
        /*
         * OpenMV sends the same QR payload on every camera frame. Keep only
         * the newest complete frame if the main loop is still updating OLED.
         */
        memcpy(s_rpi_pending_frame, s_rpi_rx_buf, (size_t)s_rpi_rx_index + 1U);
        s_rpi_frame_pending = 1U;

        s_rpi_rx_index = 0U;
        return;
    }

    if (s_rpi_rx_overflow != 0U)
    {
        return;
    }

    if (s_rpi_rx_index < (RPI_RX_PACKET_MAX_LEN - 1U))
    {
        s_rpi_rx_buf[s_rpi_rx_index++] = (char)byte;
    }
    else
    {
        s_rpi_rx_overflow = 1U;
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
    memset(s_rpi_pending_frame, 0, sizeof(s_rpi_pending_frame));
    memset(g_rpi_last_frame, 0, sizeof(g_rpi_last_frame));
    memset(g_rpi_items, 0, sizeof(g_rpi_items));
    s_rpi_rx_index = 0U;
    s_rpi_frame_pending = 0U;
    s_rpi_rx_overflow = 0U;
    g_rpi_data_ready = 0U;
    g_rpi_item_count = 0U;
    g_rpi_color[0] = '\0';
    g_rpi_code1 = 0U;
    g_rpi_code2 = 0U;
    g_rpi_rx_count = 0U;
    g_rpi_rx_last_byte = 0U;
    g_rpi_err_count = 0U;
    s_rpi_hold_display = 0U;
    s_rpi_display_page = 0U;
    s_rpi_display_tick = 0U;
    s_ui_tick = 0U;

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
    char frame[RPI_RX_PACKET_MAX_LEN];

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

    /* Parse complete newline-terminated frames in the main loop, not in the ISR. */
    if (s_rpi_frame_pending != 0U)
    {
        uint8_t duplicate_frame;

        __disable_irq();
        memcpy(frame, s_rpi_pending_frame, sizeof(frame));
        s_rpi_frame_pending = 0U;
        __enable_irq();

        frame[sizeof(frame) - 1U] = '\0';
        duplicate_frame = (uint8_t)(strcmp(g_rpi_last_frame, frame) == 0);
        strncpy(g_rpi_last_frame, frame, sizeof(g_rpi_last_frame) - 1U);
        g_rpi_last_frame[sizeof(g_rpi_last_frame) - 1U] = '\0';

        if (RaspberryPi_ParseFrame(frame))
        {
            /*
             * Do not clear and redraw OLED for identical camera frames. A new
             * QR result still updates immediately.
             */
            if (duplicate_frame == 0U)
            {
                s_rpi_display_page = 0U;
                g_rpi_data_ready = 1U;
            }
        }
        else
        {
            memset(g_rpi_items, 0, sizeof(g_rpi_items));
            g_rpi_item_count = 0U;
            g_rpi_color[0] = '\0';
            g_rpi_code1 = 0U;
            g_rpi_code2 = 0U;
            s_rpi_display_page = 0U;
            g_rpi_data_ready = 2U;
            g_rpi_err_count++;
        }
    }

    /* OLED live status ~5Hz if no frame parsed yet */
    if ((now - s_ui_tick) >= 200U)
    {
        s_ui_tick = now;
        if (s_rpi_hold_display == 0U)
        {
            RaspberryPi_ShowStatus();
        }
    }

    /* More than four QR items are shown on automatically rotating pages. */
    if ((s_rpi_hold_display != 0U) &&
        (g_rpi_item_count > RPI_OLED_ITEMS_PER_PAGE) &&
        ((now - s_rpi_display_tick) >= RPI_OLED_PAGE_PERIOD_MS))
    {
        uint8_t page_count = (uint8_t)((g_rpi_item_count + RPI_OLED_ITEMS_PER_PAGE - 1U) /
                                       RPI_OLED_ITEMS_PER_PAGE);
        s_rpi_display_page = (uint8_t)((s_rpi_display_page + 1U) % page_count);
        OLED_Clear();
        RaspberryPi_DisplayUpdate();
    }
}

static uint8_t RaspberryPi_AsciiEqualIgnoreCase(const char *left, const char *right)
{
    while ((*left != '\0') && (*right != '\0'))
    {
        char l = *left;
        char r = *right;

        if ((l >= 'A') && (l <= 'Z'))
        {
            l = (char)(l + ('a' - 'A'));
        }
        if ((r >= 'A') && (r <= 'Z'))
        {
            r = (char)(r + ('a' - 'A'));
        }
        if (l != r)
        {
            return 0U;
        }
        left++;
        right++;
    }

    return (uint8_t)((*left == '\0') && (*right == '\0'));
}

static void RaspberryPi_GetColorText(const char *source, char *output, size_t output_size)
{
    const char *normalized = source;

    if (RaspberryPi_AsciiEqualIgnoreCase(source, "red"))
        normalized = "RED";
    else if (RaspberryPi_AsciiEqualIgnoreCase(source, "blue") ||
             RaspberryPi_AsciiEqualIgnoreCase(source, "blu"))
        normalized = "BLU";
    else if (RaspberryPi_AsciiEqualIgnoreCase(source, "green") ||
             RaspberryPi_AsciiEqualIgnoreCase(source, "grn"))
        normalized = "GRN";
    else if (RaspberryPi_AsciiEqualIgnoreCase(source, "yellow") ||
             RaspberryPi_AsciiEqualIgnoreCase(source, "ylw"))
        normalized = "YLW";
    else if (RaspberryPi_AsciiEqualIgnoreCase(source, "orange") ||
             RaspberryPi_AsciiEqualIgnoreCase(source, "org"))
        normalized = "ORG";
    else if (RaspberryPi_AsciiEqualIgnoreCase(source, "purple") ||
             RaspberryPi_AsciiEqualIgnoreCase(source, "pur"))
        normalized = "PUR";

    if (output_size != 0U)
    {
        strncpy(output, normalized, output_size - 1U);
        output[output_size - 1U] = '\0';
    }
}

static uint8_t RaspberryPi_ItemIsBallCode(const RaspberryPi_Item *item)
{
    uint16_t value1;
    uint16_t value2;

    return (uint8_t)((item->field_count == 2U) &&
                     RaspberryPi_ParseU16(item->fields[0], &value1) &&
                     RaspberryPi_ParseU16(item->fields[1], &value2));
}

static uint8_t RaspberryPi_ItemHasNumericCodes(const RaspberryPi_Item *item)
{
    uint16_t value1;
    uint16_t value2;

    return (uint8_t)((item->field_count == 3U) &&
                     RaspberryPi_ParseU16(item->fields[1], &value1) &&
                     RaspberryPi_ParseU16(item->fields[2], &value2));
}

static void RaspberryPi_ShowEmphasizedValue(const char *text)
{
    size_t length = strlen(text);

    if (length <= 7U)
    {
        OLED_ShowString2x(0, 2, (char *)text, 0);
        OLED_ShowString2x(1, 2, (char *)text, 0);
    }
    else if (length == 8U)
    {
        OLED_ShowString2x(0, 2, (char *)text, 0);
    }
    else
    {
        /* A 2x string longer than eight characters would run past 128 pixels. */
        OLED_ShowStringBold(0, 2, (char *)text, 16, 0);
    }
}

static void RaspberryPi_DisplaySingleItem(const RaspberryPi_Item *item)
{
    char line[17];
    char color1[8];
    char color2[8];
    char color3[8];

    memset(line, 0, sizeof(line));
    memset(color1, 0, sizeof(color1));
    memset(color2, 0, sizeof(color2));
    memset(color3, 0, sizeof(color3));

    if (item->field_count == 1U)
    {
        RaspberryPi_GetColorText(item->fields[0], color1, sizeof(color1));
        OLED_ShowStringBold(0, 0, "SINGLE COLOR", 16, 0);
        RaspberryPi_ShowEmphasizedValue(color1);
    }
    else if (item->field_count == 2U)
    {
        if (RaspberryPi_ItemIsBallCode(item))
        {
            OLED_ShowStringBold(0, 0, "BALL CODE", 16, 0);
            (void)snprintf(line, sizeof(line), "%s,%s", item->fields[0], item->fields[1]);
        }
        else
        {
            RaspberryPi_GetColorText(item->fields[0], color1, sizeof(color1));
            RaspberryPi_GetColorText(item->fields[1], color2, sizeof(color2));
            OLED_ShowStringBold(0, 0, "COLOR PAIR", 16, 0);
            (void)snprintf(line, sizeof(line), "%s+%s", color1, color2);
        }
        RaspberryPi_ShowEmphasizedValue(line);
    }
    else if (RaspberryPi_ItemHasNumericCodes(item))
    {
        RaspberryPi_GetColorText(item->fields[0], color1, sizeof(color1));
        OLED_ShowString2x(0, 0, color1, 0);
        OLED_ShowString2x(1, 0, color1, 0);

        (void)snprintf(line, sizeof(line), "OWN:%s", item->fields[1]);
        OLED_ShowStringBold(0, 4, line, 16, 0);
        (void)snprintf(line, sizeof(line), "CORE:%s", item->fields[2]);
        OLED_ShowStringBold(0, 6, line, 16, 0);
    }
    else
    {
        RaspberryPi_GetColorText(item->fields[0], color1, sizeof(color1));
        RaspberryPi_GetColorText(item->fields[1], color2, sizeof(color2));
        RaspberryPi_GetColorText(item->fields[2], color3, sizeof(color3));
        OLED_ShowStringBold(0, 0, "COLOR SEQUENCE", 16, 0);
        (void)snprintf(line, sizeof(line), "%s>%s>%s", color1, color2, color3);
        RaspberryPi_ShowEmphasizedValue(line);
    }
}

static void RaspberryPi_FormatCompactItem(const RaspberryPi_Item *item,
                                          uint8_t item_number,
                                          char *line,
                                          size_t line_size)
{
    char color1[8] = {0};
    char color2[8] = {0};

    if (item->field_count == 1U)
    {
        RaspberryPi_GetColorText(item->fields[0], color1, sizeof(color1));
        (void)snprintf(line, line_size, "%u:COLOR %s", (unsigned)item_number, color1);
    }
    else if (item->field_count == 2U)
    {
        if (RaspberryPi_ItemIsBallCode(item))
        {
            (void)snprintf(line, line_size, "%u:BALL %s,%s",
                           (unsigned)item_number, item->fields[0], item->fields[1]);
        }
        else
        {
            RaspberryPi_GetColorText(item->fields[0], color1, sizeof(color1));
            RaspberryPi_GetColorText(item->fields[1], color2, sizeof(color2));
            (void)snprintf(line, line_size, "%u:%s+%s",
                           (unsigned)item_number, color1, color2);
        }
    }
    else if (RaspberryPi_ItemHasNumericCodes(item))
    {
        RaspberryPi_GetColorText(item->fields[0], color1, sizeof(color1));
        (void)snprintf(line, line_size, "%u:%s O%s C%s",
                       (unsigned)item_number, color1, item->fields[1], item->fields[2]);
    }
    else
    {
        char color3[8] = {0};

        RaspberryPi_GetColorText(item->fields[0], color1, sizeof(color1));
        RaspberryPi_GetColorText(item->fields[1], color2, sizeof(color2));
        RaspberryPi_GetColorText(item->fields[2], color3, sizeof(color3));
        (void)snprintf(line, line_size, "%u:%s>%s>%s",
                       (unsigned)item_number, color1, color2, color3);
    }
}

static void RaspberryPi_DisplayMultipleItems(void)
{
    uint8_t first = (uint8_t)(s_rpi_display_page * RPI_OLED_ITEMS_PER_PAGE);
    uint8_t end = (uint8_t)(first + RPI_OLED_ITEMS_PER_PAGE);
    uint8_t item_index;
    uint8_t row = 0U;
    char line[17];

    if (end > g_rpi_item_count)
    {
        end = g_rpi_item_count;
    }

    for (item_index = first; item_index < end; item_index++)
    {
        memset(line, 0, sizeof(line));
        RaspberryPi_FormatCompactItem(&g_rpi_items[item_index],
                                      (uint8_t)(item_index + 1U),
                                      line,
                                      sizeof(line));
        OLED_ShowStringBold(0, (uint8_t)(row * 2U), line, 16, 0);
        row++;
    }
}

void RaspberryPi_DisplayUpdate(void)
{
    char raw_preview[17];

    /* Once a QR result is shown, stop the READY diagnostics refresh. */
    s_rpi_hold_display = 1U;
    s_rpi_display_tick = HAL_GetTick();

    if (g_rpi_item_count == 0U)
    {
        /* Invalid frames are retained for wiring/protocol diagnostics. */
        memset(raw_preview, 0, sizeof(raw_preview));
        strncpy(raw_preview, g_rpi_last_frame, sizeof(raw_preview) - 1U);
        OLED_ShowStringBold(0, 0, "RAW", 16, 0);
        OLED_ShowStringBold(0, 2, raw_preview, 16, 0);
        return;
    }

    if (g_rpi_item_count == 1U)
    {
        RaspberryPi_DisplaySingleItem(&g_rpi_items[0]);
    }
    else
    {
        RaspberryPi_DisplayMultipleItems();
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
    RaspberryPi_SendString("[READY]\r\n");
}

void RaspberryPi_SendEcho(void)
{
    /* Protocol is one-way (OpenMV TX -> STM32 RX); no echo is required. */
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
