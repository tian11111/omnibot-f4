/**
 * @file  motor_driver_X42S.c
 * @brief Emm-firmware X42S control, sensorless homing and stall protection.
 */

#include "motor_driver_X42S.h"
#include "usart.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

#define X42S_FC_CLEAR_PROTECTION  0x0EU
#define X42S_FC_READ_HOME_PARAMS  0x22U
#define X42S_FC_FLAGS             0x3CU
#define X42S_FC_CONFIG            0x42U
#define X42S_FC_STATUS            0x43U
#define X42S_FC_WRITE_HOME_PARAMS 0x4CU
#define X42S_FC_HOME              0x9AU
#define X42S_FC_ABORT_HOME        0x9CU
#define X42S_FC_ENABLE            0xF3U
#define X42S_FC_VELOCITY          0xF6U
#define X42S_FC_AUTORUN           0xF7U
#define X42S_FC_STOP              0xFEU
#define X42S_FOOTER               0x6BU
#define X42S_REPLY_OK             0x02U
#define X42S_REPLY_COMPLETE       0x9FU

#define X42S_FRAME_VELOCITY_LEN   8U
#define X42S_FRAME_REPLY_LEN      4U
#define X42S_FRAME_FLAGS_LEN      5U
#define X42S_FRAME_HOME_DATA_LEN  18U
#define X42S_FRAME_STATUS_LEN     31U
#define X42S_FRAME_CONFIG_LEN     33U
#define X42S_FRAME_DATA_MAX_LEN   33U
#define X42S_FRAME_AUX_MAX_LEN    20U

#define X42S_MAX_SPEED_RPM        3000
#define X42S_DEFAULT_ACCEL        30U
#define X42S_COMMAND_TIMEOUT_MS   300U
#define X42S_STATUS_PERIOD_MS     100U
#define X42S_STATUS_TIMEOUT_MS    300U
#define X42S_RX_FRAME_TIMEOUT_MS  60U
#define X42S_QUERY_TIMEOUT_MS     80U
#define X42S_ACTION_TIMEOUT_MS    300U
#define X42S_BUS_IDLE_GUARD_MS    10U
#define X42S_HOME_TIMEOUT_MS      12000U
#define X42S_HOME_ENABLE_TIMEOUT_MS 1000U

#define X42S_HOME_SPEED_RPM       30U
#define X42S_HOME_DRIVER_TIMEOUT  10000UL
#define X42S_HOME_DETECT_RPM      300U
#define X42S_HOME_DETECT_CURRENT  800U
#define X42S_HOME_DETECT_TIME_MS  60U

#define X42S_EVENT_QUEUE_LEN      8U

typedef enum
{
    X42S_AUX_NONE = 0,
    X42S_AUX_DISABLE_AUTOHOME,
    X42S_AUX_HOME_CONFIG,
    X42S_AUX_HOME_TRIGGER,
    X42S_AUX_HOME_ENABLE,
    X42S_AUX_CLEAR_PROTECTION,
    X42S_AUX_CLEAR_AUTORUN
} X42S_AuxReason;

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t address;
    char axis_name;

    int16_t requested_speed;
    int16_t sent_speed;
    uint8_t velocity_pending;
    uint8_t velocity_tx[X42S_FRAME_VELOCITY_LEN];

    uint8_t aux_tx[X42S_FRAME_AUX_MAX_LEN];
    uint8_t aux_len;
    uint8_t aux_pending;
    uint8_t aux_code;
    X42S_AuxReason aux_reason;
    uint8_t awaiting_action_code;
    X42S_AuxReason awaiting_reason;
    uint32_t awaiting_action_tick;

    uint8_t query_tx[4];
    uint8_t query_pending;
    uint8_t query_active;
    uint32_t query_sent_tick;

    uint8_t rx_byte;
    uint8_t rx_buffer[X42S_FRAME_DATA_MAX_LEN];
    uint8_t rx_index;
    uint8_t rx_expected;
    uint32_t rx_last_byte_tick;
    uint32_t last_bus_activity_tick;

    X42S_StatusData live_status;
    volatile uint8_t safety_status_valid;
    volatile uint32_t last_safety_status_tick;
    uint32_t last_safety_query_tick;
    uint32_t motion_start_tick;
    int8_t blocked_direction;
    uint8_t block_reported;
    uint8_t unhomed_reported;
    uint8_t comm_fault_active;

    uint8_t home_params[X42S_FRAME_HOME_DATA_LEN];
    uint8_t auto_disable_pending;
    uint8_t clear_verify_pending;
    uint32_t clear_request_tick;
} X42S_AxisState;

static X42S_AxisState g_axis_x;
static X42S_AxisState g_axis_y;
static volatile X42S_LiftState g_lift_state;
static volatile uint8_t g_lift_homed_valid;
static volatile uint8_t g_home_command_accepted;
static volatile uint8_t g_home_running_seen;
static volatile uint8_t g_home_trigger_pending;
static volatile uint8_t g_home_enable_pending;
static volatile uint8_t g_home_enable_accepted;
static volatile uint8_t g_home_stop_pending;
static uint32_t g_home_start_tick;
static uint32_t g_home_enable_start_tick;

static uint32_t g_last_command_tick;
static uint8_t g_motion_command_active;

static volatile uint8_t g_status_ready;
static volatile uint8_t g_config_ready;
static volatile uint8_t g_error_ready;
static uint8_t g_lift_status_requested;
static uint8_t g_lift_config_requested;
static X42S_StatusData g_status_data;
static X42S_ConfigData g_config_data;
static X42S_ErrorData g_error_data;

static X42S_EventData g_event_queue[X42S_EVENT_QUEUE_LEN];
static volatile uint8_t g_event_head;
static volatile uint8_t g_event_tail;

static uint32_t X42S_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void X42S_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void X42S_QueueEvent(X42S_EventType type, char axis, int8_t direction)
{
    uint32_t primask = X42S_EnterCritical();
    uint8_t next = (uint8_t)((g_event_head + 1U) % X42S_EVENT_QUEUE_LEN);

    if (next == g_event_tail)
    {
        g_event_tail = (uint8_t)((g_event_tail + 1U) % X42S_EVENT_QUEUE_LEN);
    }
    g_event_queue[g_event_head].type = type;
    g_event_queue[g_event_head].axis = axis;
    g_event_queue[g_event_head].direction = direction;
    g_event_head = next;
    X42S_ExitCritical(primask);
}

static uint16_t X42S_ReadU16BE(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint32_t X42S_ReadU32BE(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) |
           ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) |
           (uint32_t)data[3];
}

static void X42S_WriteU16BE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)(value & 0xFFU);
}

static void X42S_WriteU32BE(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)(value & 0xFFU);
}

static uint32_t X42S_BaudIndexToRate(uint8_t index)
{
    static const uint32_t rates[] = {
        9600U, 19200U, 25000U, 38400U, 57600U,
        115200U, 256000U, 512000U, 921600U
    };

    if (index < (uint8_t)(sizeof(rates) / sizeof(rates[0])))
    {
        return rates[index];
    }
    return 0U;
}

static int8_t X42S_SpeedDirection(int16_t speed)
{
    if (speed > 0)
    {
        return 1;
    }
    if (speed < 0)
    {
        return -1;
    }
    return 0;
}

static int16_t X42S_LimitSpeed(int16_t speed)
{
    if (speed > X42S_MAX_SPEED_RPM)
    {
        return X42S_MAX_SPEED_RPM;
    }
    if (speed < -X42S_MAX_SPEED_RPM)
    {
        return -X42S_MAX_SPEED_RPM;
    }
    return speed;
}

static void X42S_BuildVelocityFrame(uint8_t *cmd, uint8_t address,
                                    uint8_t direction, uint8_t acceleration,
                                    uint16_t speed_rpm, bool synchronous)
{
    if (speed_rpm > X42S_MAX_SPEED_RPM)
    {
        speed_rpm = X42S_MAX_SPEED_RPM;
    }

    cmd[0] = address;
    cmd[1] = X42S_FC_VELOCITY;
    cmd[2] = direction;
    cmd[3] = (uint8_t)(speed_rpm >> 8);
    cmd[4] = (uint8_t)(speed_rpm & 0xFFU);
    cmd[5] = acceleration;
    cmd[6] = synchronous ? 1U : 0U;
    cmd[7] = X42S_FOOTER;
}

static void X42S_BuildAxisVelocity(X42S_AxisState *axis, uint8_t invert)
{
    int16_t speed = axis->requested_speed;
    uint16_t magnitude;
    uint8_t direction;

    if (speed < 0)
    {
        magnitude = (uint16_t)(-speed);
        direction = 1U;
    }
    else
    {
        magnitude = (uint16_t)speed;
        direction = 0U;
    }
    if (invert != 0U)
    {
        direction ^= 1U;
    }

    X42S_BuildVelocityFrame(axis->velocity_tx, axis->address, direction,
                            X42S_DEFAULT_ACCEL, magnitude, false);
}

static bool X42S_QueueAux(X42S_AxisState *axis, const uint8_t *frame,
                          uint8_t length, uint8_t code,
                          X42S_AuxReason reason)
{
    if ((length > X42S_FRAME_AUX_MAX_LEN) ||
        (axis->aux_pending != 0U) ||
        (axis->awaiting_reason != X42S_AUX_NONE))
    {
        return false;
    }

    memcpy(axis->aux_tx, frame, length);
    axis->aux_len = length;
    axis->aux_code = code;
    axis->aux_reason = reason;
    axis->aux_pending = 1U;
    return true;
}

static void X42S_TrySendAux(X42S_AxisState *axis)
{
    uint32_t now = HAL_GetTick();

    if ((axis->aux_pending == 0U) ||
        (axis->query_active != 0U) ||
        ((uint32_t)(now - axis->last_bus_activity_tick) <
         X42S_BUS_IDLE_GUARD_MS) ||
        (axis->huart->gState != HAL_UART_STATE_READY))
    {
        return;
    }

    if (HAL_UART_Transmit_DMA(axis->huart, axis->aux_tx,
                              axis->aux_len) == HAL_OK)
    {
        axis->awaiting_action_code = axis->aux_code;
        axis->awaiting_reason = axis->aux_reason;
        axis->awaiting_action_tick = now;
        axis->aux_pending = 0U;
        axis->last_bus_activity_tick = now;

        if (axis->aux_reason == X42S_AUX_HOME_TRIGGER)
        {
            g_lift_state = X42S_LIFT_STATE_HOMING;
            g_home_start_tick = HAL_GetTick();
            axis->motion_start_tick = g_home_start_tick;
            axis->last_safety_status_tick = g_home_start_tick;
            g_home_command_accepted = 0U;
            g_home_running_seen = 0U;
        }
        else if (axis->aux_reason == X42S_AUX_HOME_ENABLE)
        {
            g_home_enable_start_tick = now;
        }
    }
}

static void X42S_TrySendVelocity(X42S_AxisState *axis, uint8_t invert)
{
    uint32_t now = HAL_GetTick();

    if ((axis->velocity_pending == 0U) ||
        (axis->aux_pending != 0U) ||
        (axis->awaiting_reason != X42S_AUX_NONE) ||
        (axis->query_active != 0U) ||
        ((uint32_t)(now - axis->last_bus_activity_tick) <
         X42S_BUS_IDLE_GUARD_MS) ||
        (axis->huart->gState != HAL_UART_STATE_READY))
    {
        return;
    }

    X42S_BuildAxisVelocity(axis, invert);
    if (HAL_UART_Transmit_DMA(axis->huart, axis->velocity_tx,
                              X42S_FRAME_VELOCITY_LEN) == HAL_OK)
    {
        axis->sent_speed = axis->requested_speed;
        axis->velocity_pending = 0U;
        axis->last_bus_activity_tick = now;
    }
}

static void X42S_RequestQuery(X42S_AxisState *axis, uint8_t function)
{
    if ((axis->query_pending == 0U) && (axis->query_active == 0U))
    {
        axis->query_pending = function;
    }
}

static void X42S_TrySendQuery(X42S_AxisState *axis)
{
    uint8_t length;
    uint32_t now = HAL_GetTick();

    if ((axis->query_pending == 0U) ||
        (axis->aux_pending != 0U) ||
        (axis->awaiting_reason != X42S_AUX_NONE) ||
        (axis->velocity_pending != 0U) ||
        ((uint32_t)(now - axis->last_bus_activity_tick) <
         X42S_BUS_IDLE_GUARD_MS) ||
        (axis->huart->gState != HAL_UART_STATE_READY))
    {
        return;
    }

    axis->query_tx[0] = axis->address;
    axis->query_tx[1] = axis->query_pending;
    if (axis->query_pending == X42S_FC_STATUS)
    {
        axis->query_tx[2] = 0x7AU;
        axis->query_tx[3] = X42S_FOOTER;
        length = 4U;
    }
    else if (axis->query_pending == X42S_FC_CONFIG)
    {
        axis->query_tx[2] = 0x6CU;
        axis->query_tx[3] = X42S_FOOTER;
        length = 4U;
    }
    else
    {
        axis->query_tx[2] = X42S_FOOTER;
        length = 3U;
    }

    if (HAL_UART_Transmit_IT(axis->huart, axis->query_tx, length) == HAL_OK)
    {
        axis->query_active = axis->query_pending;
        axis->query_pending = 0U;
        axis->query_sent_tick = now;
        axis->last_bus_activity_tick = now;
        if (axis->query_active == X42S_FC_FLAGS)
        {
            axis->last_safety_query_tick = now;
        }
    }
}

static void X42S_SetRawRequestedSpeed(X42S_AxisState *axis, int16_t speed)
{
    uint32_t now = HAL_GetTick();

    speed = X42S_LimitSpeed(speed);
    if ((axis->requested_speed == 0) && (speed != 0))
    {
        axis->motion_start_tick = now;
        axis->last_safety_status_tick = now;
    }
    if (axis->requested_speed != speed)
    {
        axis->requested_speed = speed;
        axis->velocity_pending = 1U;
    }
}

static void X42S_SetGuardedSpeed(X42S_AxisState *axis, int16_t speed)
{
    int8_t direction = X42S_SpeedDirection(speed);

    if ((direction != 0) && (axis->comm_fault_active != 0U))
    {
        X42S_SetRawRequestedSpeed(axis, 0);
        return;
    }

    if ((direction != 0) && (direction == axis->blocked_direction))
    {
        X42S_SetRawRequestedSpeed(axis, 0);
        if (axis->block_reported == 0U)
        {
            X42S_QueueEvent(X42S_EVENT_LIMIT, axis->axis_name, direction);
            axis->block_reported = 1U;
        }
        return;
    }

    if ((direction != 0) && (direction != axis->blocked_direction))
    {
        axis->block_reported = 0U;
    }
    X42S_SetRawRequestedSpeed(axis, speed);
}

static void X42S_StopOne(X42S_AxisState *axis)
{
    uint8_t cmd[5];

    cmd[0] = axis->address;
    cmd[1] = X42S_FC_STOP;
    cmd[2] = 0x98U;
    cmd[3] = 0U;
    cmd[4] = X42S_FOOTER;

    (void)HAL_UART_AbortTransmit(axis->huart);
    (void)HAL_UART_Transmit(axis->huart, cmd, sizeof(cmd), 20U);
    axis->requested_speed = 0;
    axis->sent_speed = 0;
    axis->velocity_pending = 0U;
    axis->aux_pending = 0U;
    axis->awaiting_action_code = 0U;
    axis->awaiting_reason = X42S_AUX_NONE;
    axis->awaiting_action_tick = 0U;
    axis->query_pending = 0U;
    axis->query_active = 0U;
    axis->query_sent_tick = 0U;
    axis->rx_index = 0U;
    axis->rx_expected = 0U;
    axis->last_bus_activity_tick = HAL_GetTick();
}

static void X42S_AbortHomeOne(X42S_AxisState *axis)
{
    uint8_t cmd[4];

    cmd[0] = axis->address;
    cmd[1] = X42S_FC_ABORT_HOME;
    cmd[2] = 0x48U;
    cmd[3] = X42S_FOOTER;
    (void)HAL_UART_Transmit(axis->huart, cmd, sizeof(cmd), 20U);
}

static void X42S_RecordError(const X42S_AxisState *axis, uint8_t function,
                             uint8_t status, uint8_t source)
{
    g_error_data.axis = axis->axis_name;
    g_error_data.function_code = function;
    g_error_data.status_code = status;
    g_error_data.source = source;
    g_error_ready = 1U;
}

static void X42S_HomeFailed(void)
{
    uint8_t enable_failed;
    uint8_t stop_required;

    if ((g_lift_state == X42S_LIFT_STATE_CONFIGURING) ||
        (g_lift_state == X42S_LIFT_STATE_HOMING) ||
        (g_lift_state == X42S_LIFT_STATE_ENABLING))
    {
        enable_failed = (g_lift_state == X42S_LIFT_STATE_ENABLING) ? 1U : 0U;
        stop_required = (g_lift_state != X42S_LIFT_STATE_ENABLING) ? 1U : 0U;
        g_lift_state = X42S_LIFT_STATE_FAULT;
        g_lift_homed_valid = 0U;
        g_home_command_accepted = 0U;
        g_home_running_seen = 0U;
        g_home_enable_pending = 0U;
        g_home_enable_accepted = 0U;
        g_home_enable_start_tick = 0U;
        if (stop_required != 0U)
        {
            g_home_stop_pending = 1U;
        }
        X42S_QueueEvent((enable_failed != 0U) ?
                        X42S_EVENT_ENABLE_FAILED : X42S_EVENT_HOME_FAILED,
                        'x', 0);
    }
}

static void X42S_BeginHomeEnable(void)
{
    if (g_lift_state == X42S_LIFT_STATE_HOMING)
    {
        g_lift_state = X42S_LIFT_STATE_ENABLING;
        g_home_command_accepted = 0U;
        g_home_running_seen = 0U;
        g_home_enable_pending = 1U;
        g_home_enable_accepted = 0U;
        g_home_enable_start_tick = HAL_GetTick();
    }
}

static void X42S_FinishHomeReady(void)
{
    if (g_lift_state == X42S_LIFT_STATE_ENABLING)
    {
        g_lift_state = X42S_LIFT_STATE_READY;
        g_lift_homed_valid = 1U;
        g_home_enable_pending = 0U;
        g_home_enable_accepted = 0U;
        g_home_enable_start_tick = 0U;
        g_axis_x.blocked_direction = -1;
        g_axis_x.block_reported = 0U;
        g_axis_x.unhomed_reported = 0U;
        X42S_QueueEvent(X42S_EVENT_HOME_READY, 'x', 0);
    }
}

static void X42S_ParseActionReply(X42S_AxisState *axis)
{
    uint8_t function = axis->rx_buffer[1];
    uint8_t status = axis->rx_buffer[2];
    X42S_AuxReason reason = X42S_AUX_NONE;

    if (axis->awaiting_action_code == function)
    {
        reason = axis->awaiting_reason;
        axis->awaiting_action_code = 0U;
        axis->awaiting_reason = X42S_AUX_NONE;
        axis->awaiting_action_tick = 0U;
    }

    if ((function == X42S_FC_HOME) && (status == X42S_REPLY_OK))
    {
        g_home_command_accepted = 1U;
        return;
    }
    if ((function == X42S_FC_HOME) &&
        ((status == 0x12U) || (status == 0x22U)))
    {
        X42S_BeginHomeEnable();
        return;
    }
    if ((function == X42S_FC_HOME) && (status == X42S_REPLY_COMPLETE))
    {
        X42S_BeginHomeEnable();
        return;
    }

    if (status != X42S_REPLY_OK)
    {
        X42S_RecordError(axis, function, status,
                         X42S_ERROR_SOURCE_DRIVER);
        if ((function == X42S_FC_STATUS) &&
            (axis == &g_axis_x))
        {
            g_lift_status_requested = 0U;
        }
        else if ((function == X42S_FC_CONFIG) &&
                 (axis == &g_axis_x))
        {
            g_lift_config_requested = 0U;
        }
        if ((function == X42S_FC_HOME) ||
            (reason == X42S_AUX_HOME_CONFIG) ||
            (reason == X42S_AUX_HOME_TRIGGER) ||
            (reason == X42S_AUX_HOME_ENABLE))
        {
            X42S_HomeFailed();
        }
        return;
    }

    if (reason == X42S_AUX_HOME_CONFIG)
    {
        g_home_trigger_pending = 1U;
    }
    else if (reason == X42S_AUX_DISABLE_AUTOHOME)
    {
        X42S_QueueEvent(X42S_EVENT_AUTOHOME_DISABLED,
                        axis->axis_name, 0);
    }
    else if (reason == X42S_AUX_CLEAR_PROTECTION)
    {
        axis->clear_verify_pending = 1U;
        axis->clear_request_tick = HAL_GetTick();
        X42S_RequestQuery(axis, X42S_FC_FLAGS);
    }
    else if (reason == X42S_AUX_HOME_ENABLE)
    {
        g_home_enable_accepted = 1U;
        X42S_RequestQuery(&g_axis_x, X42S_FC_FLAGS);
    }
    else if (reason == X42S_AUX_CLEAR_AUTORUN)
    {
        X42S_QueueEvent(X42S_EVENT_AUTORUN_CLEARED,
                        axis->axis_name, 0);
    }
}

static void X42S_ParseStatus(X42S_AxisState *axis, const uint8_t *data)
{
    uint32_t error_raw;

    axis->live_status.bus_voltage_mv = X42S_ReadU16BE(&data[4]);
    axis->live_status.phase_current_ma = X42S_ReadU16BE(&data[6]);
    axis->live_status.speed_negative = data[15] != 0U ? 1U : 0U;
    axis->live_status.speed_rpm = X42S_ReadU16BE(&data[16]);
    axis->live_status.position_error_negative = data[23] != 0U ? 1U : 0U;
    error_raw = X42S_ReadU32BE(&data[24]);
    axis->live_status.position_error_x100_deg =
        (uint32_t)(((uint64_t)error_raw * 36000U + 32768U) / 65536U);
    axis->live_status.home_flags = data[28];
    axis->live_status.motor_flags = data[29];
    axis->live_status.homed_valid = g_lift_homed_valid;
    axis->live_status.software_state = g_lift_state;

    if ((axis == &g_axis_x) && (g_lift_status_requested != 0U))
    {
        g_status_data = axis->live_status;
        g_status_ready = 1U;
        g_lift_status_requested = 0U;
    }
}

static void X42S_ParseFlags(X42S_AxisState *axis, const uint8_t *data)
{
    axis->live_status.home_flags = data[2];
    axis->live_status.motor_flags = data[3];
    axis->live_status.homed_valid = g_lift_homed_valid;
    axis->live_status.software_state = g_lift_state;
    axis->last_safety_status_tick = HAL_GetTick();
    axis->safety_status_valid = 1U;

    if (axis->comm_fault_active != 0U)
    {
        axis->comm_fault_active = 0U;
        X42S_QueueEvent(X42S_EVENT_COMM_RECOVERED,
                        axis->axis_name, 0);
    }
}

static void X42S_ParseLiftConfig(const uint8_t *data)
{
    g_config_data.pulse_port_mode = data[5];
    g_config_data.max_current_ma = X42S_ReadU16BE(&data[14]);
    g_config_data.uart_baud_index = data[18];
    g_config_data.uart_baud_rate =
        X42S_BaudIndexToRate(g_config_data.uart_baud_index);
    g_config_data.clog_mode = data[23];
    g_config_data.clog_speed_rpm = X42S_ReadU16BE(&data[24]);
    g_config_data.clog_current_ma = X42S_ReadU16BE(&data[26]);
    g_config_data.clog_time_ms = X42S_ReadU16BE(&data[28]);
    g_config_ready = 1U;
}

static void X42S_ParseHomeParams(X42S_AxisState *axis, const uint8_t *data)
{
    memcpy(axis->home_params, data, X42S_FRAME_HOME_DATA_LEN);
    if (data[16] != 0U)
    {
        axis->auto_disable_pending = 1U;
    }
}

static void X42S_ParseCompletedFrame(X42S_AxisState *axis)
{
    const uint8_t *data = axis->rx_buffer;
    uint8_t function = data[1];

    if (data[axis->rx_expected - 1U] != X42S_FOOTER)
    {
        if (axis->query_active == function)
        {
            axis->query_active = 0U;
            axis->query_sent_tick = 0U;
        }
        if ((axis == &g_axis_x) && (function == X42S_FC_STATUS))
        {
            g_lift_status_requested = 0U;
        }
        else if ((axis == &g_axis_x) && (function == X42S_FC_CONFIG))
        {
            g_lift_config_requested = 0U;
        }
        X42S_RecordError(axis, function, 0xEEU,
                         X42S_ERROR_SOURCE_PARSER);
        return;
    }

    if (axis->query_active == function)
    {
        axis->query_active = 0U;
        axis->query_sent_tick = 0U;
    }

    if (axis->rx_expected == X42S_FRAME_REPLY_LEN)
    {
        X42S_ParseActionReply(axis);
    }
    else if ((function == X42S_FC_FLAGS) &&
             (axis->rx_expected == X42S_FRAME_FLAGS_LEN))
    {
        X42S_ParseFlags(axis, data);
    }
    else if ((function == X42S_FC_STATUS) &&
             (axis->rx_expected == X42S_FRAME_STATUS_LEN))
    {
        X42S_ParseStatus(axis, data);
    }
    else if ((axis == &g_axis_x) &&
             (function == X42S_FC_CONFIG) &&
             (axis->rx_expected == X42S_FRAME_CONFIG_LEN))
    {
        X42S_ParseLiftConfig(data);
        g_lift_config_requested = 0U;
    }
    else if ((function == X42S_FC_READ_HOME_PARAMS) &&
             (axis->rx_expected == X42S_FRAME_HOME_DATA_LEN))
    {
        X42S_ParseHomeParams(axis, data);
    }
}

static void X42S_ProcessRxByte(X42S_AxisState *axis)
{
    uint8_t byte = axis->rx_byte;
    uint8_t function;
    uint32_t now = HAL_GetTick();

    axis->rx_last_byte_tick = now;
    axis->last_bus_activity_tick = now;

    if (axis->rx_index == 0U)
    {
        if (byte != axis->address)
        {
            return;
        }
        axis->rx_buffer[axis->rx_index++] = byte;
        return;
    }

    if (axis->rx_index >= X42S_FRAME_DATA_MAX_LEN)
    {
        axis->rx_index = 0U;
        axis->rx_expected = 0U;
        if (axis->query_active != 0U)
        {
            function = axis->query_active;
            axis->query_active = 0U;
            axis->query_sent_tick = 0U;
            X42S_RecordError(axis, function, 0xEEU,
                             X42S_ERROR_SOURCE_PARSER);
        }
        return;
    }

    axis->rx_buffer[axis->rx_index++] = byte;
    if (axis->rx_index == 2U)
    {
        if ((byte == X42S_FC_STATUS) || (byte == X42S_FC_CONFIG) ||
            (byte == X42S_FC_READ_HOME_PARAMS))
        {
            axis->rx_expected = 0U;
        }
        else if (byte == X42S_FC_FLAGS)
        {
            axis->rx_expected = X42S_FRAME_FLAGS_LEN;
        }
        else
        {
            axis->rx_expected = X42S_FRAME_REPLY_LEN;
        }
    }

    if (axis->rx_index == 3U)
    {
        if ((axis->rx_buffer[1] == X42S_FC_STATUS) &&
            (byte == X42S_FRAME_STATUS_LEN))
        {
            axis->rx_expected = X42S_FRAME_STATUS_LEN;
        }
        else if ((axis->rx_buffer[1] == X42S_FC_CONFIG) &&
                 (byte == X42S_FRAME_CONFIG_LEN))
        {
            axis->rx_expected = X42S_FRAME_CONFIG_LEN;
        }
        else if ((axis->rx_buffer[1] == X42S_FC_READ_HOME_PARAMS) &&
                 (byte <= 0x05U))
        {
            axis->rx_expected = X42S_FRAME_HOME_DATA_LEN;
        }
        else if ((axis->rx_buffer[1] == X42S_FC_STATUS) ||
                 (axis->rx_buffer[1] == X42S_FC_CONFIG) ||
                 (axis->rx_buffer[1] == X42S_FC_READ_HOME_PARAMS))
        {
            axis->rx_expected = X42S_FRAME_REPLY_LEN;
        }
    }

    if ((axis->rx_index == X42S_FRAME_REPLY_LEN) &&
        (axis->rx_buffer[1] == X42S_FC_FLAGS) &&
        (byte == X42S_FOOTER) &&
        ((axis->rx_buffer[2] == X42S_REPLY_OK) ||
         (axis->rx_buffer[2] == 0xE2U) ||
         (axis->rx_buffer[2] == 0xEEU)))
    {
        axis->rx_expected = X42S_FRAME_REPLY_LEN;
    }

    if ((axis->rx_expected != 0U) &&
        (axis->rx_index >= axis->rx_expected))
    {
        X42S_ParseCompletedFrame(axis);
        axis->rx_index = 0U;
        axis->rx_expected = 0U;
    }
}

static void X42S_ClearDiagnosticRequest(X42S_AxisState *axis,
                                         uint8_t function)
{
    if (axis != &g_axis_x)
    {
        return;
    }
    if (function == X42S_FC_STATUS)
    {
        g_lift_status_requested = 0U;
    }
    else if (function == X42S_FC_CONFIG)
    {
        g_lift_config_requested = 0U;
    }
}

static void X42S_HandleCommunicationState(X42S_AxisState *axis)
{
    uint32_t now = HAL_GetTick();
    uint8_t function;
    X42S_AuxReason reason;

    if ((axis->rx_index != 0U) &&
        ((uint32_t)(now - axis->rx_last_byte_tick) >
         X42S_RX_FRAME_TIMEOUT_MS))
    {
        function = (axis->rx_index >= 2U) ?
                   axis->rx_buffer[1] : axis->query_active;
        axis->rx_index = 0U;
        axis->rx_expected = 0U;
        if (axis->query_active != 0U)
        {
            function = axis->query_active;
            axis->query_active = 0U;
            axis->query_sent_tick = 0U;
        }
        X42S_ClearDiagnosticRequest(axis, function);
        X42S_RecordError(axis, function, 0xEEU,
                         X42S_ERROR_SOURCE_PARSER);
    }

    if ((axis->query_active != 0U) &&
        ((uint32_t)(now - axis->query_sent_tick) >
         X42S_QUERY_TIMEOUT_MS) &&
        (axis->rx_index == 0U))
    {
        function = axis->query_active;
        axis->query_active = 0U;
        axis->query_sent_tick = 0U;
        axis->rx_index = 0U;
        axis->rx_expected = 0U;
        X42S_ClearDiagnosticRequest(axis, function);
        if (function != X42S_FC_FLAGS)
        {
            X42S_RecordError(axis, function, 0xEEU,
                             X42S_ERROR_SOURCE_PARSER);
        }
    }

    if ((axis->awaiting_reason != X42S_AUX_NONE) &&
        ((uint32_t)(now - axis->awaiting_action_tick) >
         X42S_ACTION_TIMEOUT_MS))
    {
        function = axis->awaiting_action_code;
        reason = axis->awaiting_reason;
        axis->awaiting_action_code = 0U;
        axis->awaiting_reason = X42S_AUX_NONE;
        axis->awaiting_action_tick = 0U;
        X42S_RecordError(axis, function, 0xEEU,
                         X42S_ERROR_SOURCE_PARSER);
        if ((reason == X42S_AUX_HOME_CONFIG) ||
            (reason == X42S_AUX_HOME_TRIGGER) ||
            (reason == X42S_AUX_HOME_ENABLE))
        {
            X42S_HomeFailed();
        }
    }
}

static void X42S_BuildDisableAutoHome(X42S_AxisState *axis)
{
    uint8_t frame[20];
    const uint8_t *source = axis->home_params;

    frame[0] = axis->address;
    frame[1] = X42S_FC_WRITE_HOME_PARAMS;
    frame[2] = 0xAEU;
    frame[3] = 1U;
    memcpy(&frame[4], &source[2], 14U);
    frame[18] = 0U;
    frame[19] = X42S_FOOTER;

    if (X42S_QueueAux(axis, frame, sizeof(frame),
                       X42S_FC_WRITE_HOME_PARAMS,
                       X42S_AUX_DISABLE_AUTOHOME))
    {
        axis->auto_disable_pending = 0U;
    }
}

static void X42S_BuildLiftHomeConfig(void)
{
    uint8_t frame[20];
    uint8_t down_direction = (uint8_t)(1U ^ X42S_MOTOR_X_DIR_INVERT);

    frame[0] = g_axis_x.address;
    frame[1] = X42S_FC_WRITE_HOME_PARAMS;
    frame[2] = 0xAEU;
    frame[3] = 0U;
    frame[4] = 0x02U;
    frame[5] = down_direction;
    X42S_WriteU16BE(&frame[6], X42S_HOME_SPEED_RPM);
    X42S_WriteU32BE(&frame[8], X42S_HOME_DRIVER_TIMEOUT);
    X42S_WriteU16BE(&frame[12], X42S_HOME_DETECT_RPM);
    X42S_WriteU16BE(&frame[14], X42S_HOME_DETECT_CURRENT);
    X42S_WriteU16BE(&frame[16], X42S_HOME_DETECT_TIME_MS);
    frame[18] = 0U;
    frame[19] = X42S_FOOTER;

    (void)X42S_QueueAux(&g_axis_x, frame, sizeof(frame),
                         X42S_FC_WRITE_HOME_PARAMS,
                         X42S_AUX_HOME_CONFIG);
}

static void X42S_BuildHomeTrigger(void)
{
    uint8_t frame[5];

    frame[0] = g_axis_x.address;
    frame[1] = X42S_FC_HOME;
    frame[2] = 0x02U;
    frame[3] = 0U;
    frame[4] = X42S_FOOTER;

    if (X42S_QueueAux(&g_axis_x, frame, sizeof(frame), X42S_FC_HOME,
                       X42S_AUX_HOME_TRIGGER))
    {
        g_home_trigger_pending = 0U;
    }
}

static void X42S_BuildHomeEnable(void)
{
    uint8_t frame[6];

    frame[0] = g_axis_x.address;
    frame[1] = X42S_FC_ENABLE;
    frame[2] = 0xABU;
    frame[3] = 1U;
    frame[4] = 0U;
    frame[5] = X42S_FOOTER;

    if (X42S_QueueAux(&g_axis_x, frame, sizeof(frame), X42S_FC_ENABLE,
                       X42S_AUX_HOME_ENABLE))
    {
        g_home_enable_pending = 0U;
    }
}

static bool X42S_CopyLiveStatus(X42S_AxisState *axis,
                                 X42S_StatusData *status,
                                 uint32_t *tick)
{
    uint32_t primask = X42S_EnterCritical();

    if (axis->safety_status_valid == 0U)
    {
        X42S_ExitCritical(primask);
        return false;
    }
    *status = axis->live_status;
    *tick = axis->last_safety_status_tick;
    X42S_ExitCritical(primask);
    return true;
}

static void X42S_HandleAxisSafety(X42S_AxisState *axis)
{
    X42S_StatusData status;
    uint32_t status_tick;
    uint32_t now = HAL_GetTick();
    uint8_t stall;
    int8_t direction;
    bool has_status;

    has_status = X42S_CopyLiveStatus(axis, &status, &status_tick);
    if (!has_status)
    {
        status_tick = axis->last_safety_status_tick;
    }

    if (has_status && (axis->clear_verify_pending != 0U) &&
        (status_tick >= axis->clear_request_tick) &&
        ((status.motor_flags & 0x0CU) == 0U))
    {
        axis->clear_verify_pending = 0U;
        axis->blocked_direction = 0;
        axis->block_reported = 0U;
        X42S_QueueEvent(X42S_EVENT_PROTECTION_CLEARED,
                        axis->axis_name, 0);
    }

    if (axis->requested_speed == 0)
    {
        return;
    }

    if (((uint32_t)(now - axis->motion_start_tick) >
         X42S_STATUS_TIMEOUT_MS) &&
        ((uint32_t)(now - status_tick) > X42S_STATUS_TIMEOUT_MS))
    {
        direction = X42S_SpeedDirection(axis->requested_speed);
        axis->comm_fault_active = 1U;
        X42S_StopOne(axis);
        X42S_QueueEvent(X42S_EVENT_RX_TIMEOUT,
                        axis->axis_name, direction);
        return;
    }

    if (!has_status || (status_tick < axis->motion_start_tick))
    {
        return;
    }

    stall = (uint8_t)(status.motor_flags & 0x0CU);
    if (stall != 0U)
    {
        direction = X42S_SpeedDirection(axis->requested_speed);
        X42S_StopOne(axis);
        axis->blocked_direction = direction;
        axis->block_reported = 0U;
        X42S_QueueEvent(X42S_EVENT_LIMIT,
                        axis->axis_name, direction);
        if ((stall & 0x08U) != 0U)
        {
            X42S_QueueEvent(X42S_EVENT_STALL_FAULT,
                            axis->axis_name, direction);
        }
    }
    else if ((axis->blocked_direction != 0) &&
             (X42S_SpeedDirection(axis->requested_speed) !=
              axis->blocked_direction))
    {
        axis->blocked_direction = 0;
        axis->block_reported = 0U;
    }
}

static void X42S_HandleHoming(void)
{
    X42S_StatusData status;
    uint32_t status_tick;
    uint32_t elapsed;

    if (g_lift_state != X42S_LIFT_STATE_HOMING)
    {
        return;
    }

    elapsed = (uint32_t)(HAL_GetTick() - g_home_start_tick);
    if ((elapsed > X42S_STATUS_TIMEOUT_MS) &&
        ((uint32_t)(HAL_GetTick() - g_axis_x.last_safety_status_tick) >
         X42S_STATUS_TIMEOUT_MS))
    {
        g_axis_x.comm_fault_active = 1U;
        X42S_HomeFailed();
        X42S_QueueEvent(X42S_EVENT_RX_TIMEOUT, 'x', 0);
        return;
    }
    if (elapsed > X42S_HOME_TIMEOUT_MS)
    {
        X42S_HomeFailed();
        X42S_QueueEvent(X42S_EVENT_HOME_TIMEOUT, 'x', 0);
        return;
    }

    if ((g_home_command_accepted != 0U) &&
        X42S_CopyLiveStatus(&g_axis_x, &status, &status_tick) &&
        (status_tick >= g_home_start_tick))
    {
        (void)status_tick;
        if ((status.home_flags & 0x08U) != 0U)
        {
            X42S_HomeFailed();
        }
        else if ((status.motor_flags & 0x08U) != 0U)
        {
            X42S_QueueEvent(X42S_EVENT_STALL_FAULT, 'x', -1);
            X42S_HomeFailed();
        }
        else if ((status.home_flags & 0x04U) != 0U)
        {
            g_home_running_seen = 1U;
        }
        else if (g_home_running_seen != 0U)
        {
            X42S_BeginHomeEnable();
        }
    }
}

static void X42S_HandleHomeEnable(void)
{
    X42S_StatusData status;
    uint32_t status_tick;

    if (g_lift_state != X42S_LIFT_STATE_ENABLING)
    {
        return;
    }

    if ((g_home_enable_start_tick != 0U) &&
        ((uint32_t)(HAL_GetTick() - g_home_enable_start_tick) >
         X42S_HOME_ENABLE_TIMEOUT_MS))
    {
        X42S_HomeFailed();
        return;
    }

    if ((g_home_enable_accepted != 0U) &&
        X42S_CopyLiveStatus(&g_axis_x, &status, &status_tick) &&
        (status_tick >= g_home_enable_start_tick))
    {
        if ((status.home_flags & 0x08U) != 0U)
        {
            X42S_HomeFailed();
        }
        else if (((status.home_flags & 0x04U) == 0U) &&
                 ((status.motor_flags & 0x01U) != 0U))
        {
            X42S_FinishHomeReady();
        }
    }
}

void X_V2_Vel_Control(UART_HandleTypeDef *huart, uint8_t addr,
                      uint8_t dir, uint16_t acc, float vel, bool snF)
{
    uint8_t cmd[X42S_FRAME_VELOCITY_LEN];
    uint8_t acceleration;
    uint16_t speed_rpm;

    if (vel < 0.0f)
    {
        vel = -vel;
    }
    if (vel > (float)X42S_MAX_SPEED_RPM)
    {
        vel = (float)X42S_MAX_SPEED_RPM;
    }
    speed_rpm = (uint16_t)(vel + 0.5f);
    acceleration = (acc > 255U) ? 255U : (uint8_t)acc;
    X42S_BuildVelocityFrame(cmd, addr, dir, acceleration, speed_rpm, snF);
    (void)HAL_UART_Transmit(huart, cmd, sizeof(cmd), HAL_MAX_DELAY);
}

void MotorDriverX42S_Serial_Init(void)
{
    memset(&g_axis_x, 0, sizeof(g_axis_x));
    memset(&g_axis_y, 0, sizeof(g_axis_y));

    g_axis_x.huart = &huart6;
    g_axis_x.address = X42S_MOTOR2_ADDR;
    g_axis_x.axis_name = 'x';
    g_axis_x.sent_speed = 32767;

    g_axis_y.huart = &huart2;
    g_axis_y.address = X42S_MOTOR1_ADDR;
    g_axis_y.axis_name = 'y';
    g_axis_y.sent_speed = 32767;

    g_lift_state = X42S_LIFT_STATE_UNHOMED;
    g_lift_homed_valid = 0U;
    g_home_command_accepted = 0U;
    g_home_running_seen = 0U;
    g_home_trigger_pending = 0U;
    g_home_enable_pending = 0U;
    g_home_enable_accepted = 0U;
    g_home_stop_pending = 0U;
    g_home_enable_start_tick = 0U;
    g_motion_command_active = 0U;
    g_last_command_tick = HAL_GetTick();
    g_status_ready = 0U;
    g_config_ready = 0U;
    g_error_ready = 0U;
    g_lift_status_requested = 0U;
    g_lift_config_requested = 0U;
    g_event_head = 0U;
    g_event_tail = 0U;

    (void)HAL_UART_Receive_IT(g_axis_x.huart, &g_axis_x.rx_byte, 1U);
    (void)HAL_UART_Receive_IT(g_axis_y.huart, &g_axis_y.rx_byte, 1U);

    /* PD3 is high from GPIO init. Abort any driver-stored boot motion first. */
    X42S_AbortHomeOne(&g_axis_x);
    X42S_StopOne(&g_axis_x);
    X42S_AbortHomeOne(&g_axis_y);
    X42S_StopOne(&g_axis_y);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

    X42S_RequestQuery(&g_axis_x, X42S_FC_READ_HOME_PARAMS);
    X42S_RequestQuery(&g_axis_y, X42S_FC_READ_HOME_PARAMS);
}

void MotorDriverX42S_SetDualSpeed(int16_t x_speed, int16_t y_speed)
{
    g_last_command_tick = HAL_GetTick();
    x_speed = X42S_LimitSpeed(x_speed);
    y_speed = X42S_LimitSpeed(y_speed);

    if ((x_speed != 0) &&
        ((g_lift_homed_valid == 0U) ||
         (g_lift_state != X42S_LIFT_STATE_READY)))
    {
        x_speed = 0;
        if (g_axis_x.unhomed_reported == 0U)
        {
            X42S_QueueEvent(X42S_EVENT_UNHOMED, 'x', 0);
            g_axis_x.unhomed_reported = 1U;
        }
    }
    else if (x_speed == 0)
    {
        g_axis_x.unhomed_reported = 0U;
    }

    X42S_SetGuardedSpeed(&g_axis_x, x_speed);
    X42S_SetGuardedSpeed(&g_axis_y, y_speed);
    g_motion_command_active =
        ((g_axis_x.requested_speed != 0) ||
         (g_axis_y.requested_speed != 0)) ? 1U : 0U;
}

void MotorDriverX42S_StopAll(void)
{
    X42S_StopOne(&g_axis_x);
    X42S_StopOne(&g_axis_y);
    g_motion_command_active = 0U;
    if ((g_lift_state == X42S_LIFT_STATE_CONFIGURING) ||
        (g_lift_state == X42S_LIFT_STATE_HOMING) ||
        (g_lift_state == X42S_LIFT_STATE_ENABLING))
    {
        X42S_HomeFailed();
    }
}

void MotorDriverX42S_ControlTask(void)
{
    uint32_t now = HAL_GetTick();

    if ((g_motion_command_active != 0U) &&
        ((uint32_t)(now - g_last_command_tick) >
         X42S_COMMAND_TIMEOUT_MS))
    {
        X42S_SetRawRequestedSpeed(&g_axis_x, 0);
        X42S_SetRawRequestedSpeed(&g_axis_y, 0);
        g_motion_command_active = 0U;
    }

    if (g_home_stop_pending != 0U)
    {
        g_home_stop_pending = 0U;
        X42S_StopOne(&g_axis_x);
    }

    X42S_HandleCommunicationState(&g_axis_x);
    X42S_HandleCommunicationState(&g_axis_y);

    if (g_axis_x.auto_disable_pending != 0U)
    {
        X42S_BuildDisableAutoHome(&g_axis_x);
    }
    if (g_axis_y.auto_disable_pending != 0U)
    {
        X42S_BuildDisableAutoHome(&g_axis_y);
    }
    if (g_home_trigger_pending != 0U)
    {
        X42S_BuildHomeTrigger();
    }
    if (g_home_enable_pending != 0U)
    {
        X42S_BuildHomeEnable();
    }

    X42S_HandleHoming();
    X42S_HandleHomeEnable();
    if ((g_lift_state != X42S_LIFT_STATE_HOMING) &&
        (g_lift_state != X42S_LIFT_STATE_ENABLING))
    {
        X42S_HandleAxisSafety(&g_axis_x);
    }
    X42S_HandleAxisSafety(&g_axis_y);

    if (((g_axis_x.requested_speed != 0) ||
         (g_lift_state == X42S_LIFT_STATE_HOMING) ||
         (g_lift_state == X42S_LIFT_STATE_ENABLING) ||
         (g_axis_x.comm_fault_active != 0U) ||
         (g_axis_x.clear_verify_pending != 0U)) &&
        ((uint32_t)(now - g_axis_x.last_safety_query_tick) >=
         X42S_STATUS_PERIOD_MS))
    {
        X42S_RequestQuery(&g_axis_x, X42S_FC_FLAGS);
    }
    if (((g_axis_y.requested_speed != 0) ||
         (g_axis_y.comm_fault_active != 0U) ||
         (g_axis_y.clear_verify_pending != 0U)) &&
        ((uint32_t)(now - g_axis_y.last_safety_query_tick) >=
         X42S_STATUS_PERIOD_MS))
    {
        X42S_RequestQuery(&g_axis_y, X42S_FC_FLAGS);
    }

    if (g_lift_status_requested != 0U)
    {
        X42S_RequestQuery(&g_axis_x, X42S_FC_STATUS);
    }
    if (g_lift_config_requested != 0U)
    {
        X42S_RequestQuery(&g_axis_x, X42S_FC_CONFIG);
    }

    X42S_TrySendAux(&g_axis_x);
    X42S_TrySendAux(&g_axis_y);
    X42S_TrySendVelocity(&g_axis_x, X42S_MOTOR_X_DIR_INVERT);
    X42S_TrySendVelocity(&g_axis_y, X42S_MOTOR_Y_DIR_INVERT);
    X42S_TrySendQuery(&g_axis_x);
    X42S_TrySendQuery(&g_axis_y);
}

void MotorDriverX42S_RxCallback(UART_HandleTypeDef *huart)
{
    X42S_AxisState *axis = NULL;

    if (huart->Instance == USART6)
    {
        axis = &g_axis_x;
    }
    else if (huart->Instance == USART2)
    {
        axis = &g_axis_y;
    }

    if (axis != NULL)
    {
        X42S_ProcessRxByte(axis);
        (void)HAL_UART_Receive_IT(axis->huart, &axis->rx_byte, 1U);
    }
}

void MotorDriverX42S_OnUartError(UART_HandleTypeDef *huart)
{
    X42S_AxisState *axis = NULL;
    uint8_t function;

    if (huart->Instance == USART6)
    {
        axis = &g_axis_x;
    }
    else if (huart->Instance == USART2)
    {
        axis = &g_axis_y;
    }

    if (axis != NULL)
    {
        function = axis->query_active;
        axis->rx_index = 0U;
        axis->rx_expected = 0U;
        axis->query_active = 0U;
        axis->query_sent_tick = 0U;
        axis->last_bus_activity_tick = HAL_GetTick();
        if (function != 0U)
        {
            X42S_ClearDiagnosticRequest(axis, function);
            X42S_RecordError(axis, function, 0xEEU,
                             X42S_ERROR_SOURCE_PARSER);
        }
        (void)HAL_UART_Receive_IT(axis->huart, &axis->rx_byte, 1U);
    }
}

void MotorDriverX42S_QueryLiftStatus(void)
{
    g_lift_status_requested = 1U;
    X42S_RequestQuery(&g_axis_x, X42S_FC_STATUS);
}

void MotorDriverX42S_QueryLiftConfig(void)
{
    g_lift_config_requested = 1U;
    X42S_RequestQuery(&g_axis_x, X42S_FC_CONFIG);
}

bool MotorDriverX42S_StartLiftHoming(void)
{
    if ((g_lift_state == X42S_LIFT_STATE_CONFIGURING) ||
        (g_lift_state == X42S_LIFT_STATE_HOMING) ||
        (g_lift_state == X42S_LIFT_STATE_ENABLING) ||
        (g_axis_x.requested_speed != 0) ||
        (g_axis_x.velocity_pending != 0U) ||
        (g_axis_x.comm_fault_active != 0U) ||
        (g_axis_x.aux_pending != 0U) ||
        (g_axis_x.awaiting_reason != X42S_AUX_NONE) ||
        (g_axis_x.query_pending != 0U) ||
        (g_axis_x.query_active != 0U) ||
        (g_axis_x.huart->gState != HAL_UART_STATE_READY))
    {
        return false;
    }

    g_lift_state = X42S_LIFT_STATE_CONFIGURING;
    g_lift_homed_valid = 0U;
    g_axis_x.unhomed_reported = 0U;
    g_home_command_accepted = 0U;
    g_home_running_seen = 0U;
    g_home_trigger_pending = 0U;
    g_home_enable_pending = 0U;
    g_home_enable_accepted = 0U;
    g_home_enable_start_tick = 0U;
    X42S_BuildLiftHomeConfig();
    X42S_QueueEvent(X42S_EVENT_HOME_RUNNING, 'x', -1);
    return true;
}

bool MotorDriverX42S_ClearProtection(char axis_name)
{
    X42S_AxisState *axis;
    uint8_t frame[4];

    if (axis_name == 'x')
    {
        axis = &g_axis_x;
    }
    else if (axis_name == 'y')
    {
        axis = &g_axis_y;
    }
    else
    {
        return false;
    }

    frame[0] = axis->address;
    frame[1] = X42S_FC_CLEAR_PROTECTION;
    frame[2] = 0x52U;
    frame[3] = X42S_FOOTER;
    return X42S_QueueAux(axis, frame, sizeof(frame),
                          X42S_FC_CLEAR_PROTECTION,
                          X42S_AUX_CLEAR_PROTECTION);
}

bool MotorDriverX42S_ClearStoredAutorun(void)
{
    uint8_t frame_x[10] = {
        X42S_MOTOR2_ADDR, X42S_FC_AUTORUN, 0x1CU, 0U, 0U,
        0U, 0U, 0U, 0U, X42S_FOOTER
    };
    uint8_t frame_y[10] = {
        X42S_MOTOR1_ADDR, X42S_FC_AUTORUN, 0x1CU, 0U, 0U,
        0U, 0U, 0U, 0U, X42S_FOOTER
    };

    if ((g_axis_x.aux_pending != 0U) ||
        (g_axis_y.aux_pending != 0U) ||
        (g_axis_x.awaiting_reason != X42S_AUX_NONE) ||
        (g_axis_y.awaiting_reason != X42S_AUX_NONE))
    {
        return false;
    }
    if (!X42S_QueueAux(&g_axis_x, frame_x, sizeof(frame_x),
                        X42S_FC_AUTORUN, X42S_AUX_CLEAR_AUTORUN))
    {
        return false;
    }
    if (!X42S_QueueAux(&g_axis_y, frame_y, sizeof(frame_y),
                        X42S_FC_AUTORUN, X42S_AUX_CLEAR_AUTORUN))
    {
        g_axis_x.aux_pending = 0U;
        return false;
    }
    return true;
}

bool MotorDriverX42S_TakeLiftStatus(X42S_StatusData *status)
{
    uint32_t primask;

    if (status == NULL)
    {
        return false;
    }
    primask = X42S_EnterCritical();
    if (g_status_ready == 0U)
    {
        X42S_ExitCritical(primask);
        return false;
    }
    *status = g_status_data;
    g_status_ready = 0U;
    X42S_ExitCritical(primask);
    return true;
}

bool MotorDriverX42S_TakeLiftConfig(X42S_ConfigData *config)
{
    uint32_t primask;

    if (config == NULL)
    {
        return false;
    }
    primask = X42S_EnterCritical();
    if (g_config_ready == 0U)
    {
        X42S_ExitCritical(primask);
        return false;
    }
    *config = g_config_data;
    g_config_ready = 0U;
    X42S_ExitCritical(primask);
    return true;
}

bool MotorDriverX42S_TakeError(X42S_ErrorData *error)
{
    uint32_t primask;

    if (error == NULL)
    {
        return false;
    }
    primask = X42S_EnterCritical();
    if (g_error_ready == 0U)
    {
        X42S_ExitCritical(primask);
        return false;
    }
    *error = g_error_data;
    g_error_ready = 0U;
    X42S_ExitCritical(primask);
    return true;
}

bool MotorDriverX42S_TakeEvent(X42S_EventData *event)
{
    uint32_t primask;

    if (event == NULL)
    {
        return false;
    }
    primask = X42S_EnterCritical();
    if (g_event_tail == g_event_head)
    {
        X42S_ExitCritical(primask);
        return false;
    }
    *event = g_event_queue[g_event_tail];
    g_event_tail = (uint8_t)((g_event_tail + 1U) % X42S_EVENT_QUEUE_LEN);
    X42S_ExitCritical(primask);
    return true;
}
