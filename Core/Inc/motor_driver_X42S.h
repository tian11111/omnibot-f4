#ifndef MOTOR_DRIVER_X42S_H
#define MOTOR_DRIVER_X42S_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* The two motors use separate UARTs, so both may keep the factory address 1. */
#define X42S_MOTOR1_ADDR          1U  /* USART2: auxiliary gripper axis (Y) */
#define X42S_MOTOR2_ADDR          1U  /* USART6: gripper lift axis (X) */

#define X42S_MOTOR_X_DIR_INVERT   0U
#define X42S_MOTOR_Y_DIR_INVERT   0U

typedef enum
{
    X42S_LIFT_STATE_UNHOMED = 0,
    X42S_LIFT_STATE_CONFIGURING,
    X42S_LIFT_STATE_HOMING,
    X42S_LIFT_STATE_ENABLING,
    X42S_LIFT_STATE_READY,
    X42S_LIFT_STATE_FAULT
} X42S_LiftState;

typedef struct
{
    uint16_t bus_voltage_mv;
    uint16_t phase_current_ma;
    uint16_t speed_rpm;
    uint32_t position_error_x100_deg;
    uint8_t speed_negative;
    uint8_t position_error_negative;
    uint8_t home_flags;
    uint8_t motor_flags;
    uint8_t homed_valid;
    X42S_LiftState software_state;
} X42S_StatusData;

typedef struct
{
    uint8_t pulse_port_mode;
    uint16_t max_current_ma;
    uint8_t uart_baud_index;
    uint32_t uart_baud_rate;
    uint8_t clog_mode;
    uint16_t clog_speed_rpm;
    uint16_t clog_current_ma;
    uint16_t clog_time_ms;
} X42S_ConfigData;

typedef struct
{
    char axis;
    uint8_t function_code;
    uint8_t status_code;
    uint8_t source;
} X42S_ErrorData;

#define X42S_ERROR_SOURCE_DRIVER  0U
#define X42S_ERROR_SOURCE_PARSER  1U

typedef enum
{
    X42S_EVENT_HOME_RUNNING = 1,
    X42S_EVENT_HOME_READY,
    X42S_EVENT_HOME_FAILED,
    X42S_EVENT_LIMIT,
    X42S_EVENT_STALL_FAULT,
    X42S_EVENT_RX_TIMEOUT,
    X42S_EVENT_HOME_TIMEOUT,
    X42S_EVENT_UNHOMED,
    X42S_EVENT_PROTECTION_CLEARED,
    X42S_EVENT_AUTOHOME_DISABLED,
    X42S_EVENT_AUTORUN_CLEARED,
    X42S_EVENT_COMM_RECOVERED,
    X42S_EVENT_ENABLE_FAILED
} X42S_EventType;

typedef struct
{
    X42S_EventType type;
    char axis;
    int8_t direction;
} X42S_EventData;

void X_V2_Vel_Control(UART_HandleTypeDef *huart, uint8_t addr,
                      uint8_t dir, uint16_t acc, float vel, bool snF);
void MotorDriverX42S_SetDualSpeed(int16_t x_speed, int16_t y_speed);
void MotorDriverX42S_StopAll(void);
void MotorDriverX42S_Serial_Init(void);
void MotorDriverX42S_ControlTask(void);

void MotorDriverX42S_RxCallback(UART_HandleTypeDef *huart);
void MotorDriverX42S_OnUartError(UART_HandleTypeDef *huart);

void MotorDriverX42S_QueryLiftStatus(void);
void MotorDriverX42S_QueryLiftConfig(void);
bool MotorDriverX42S_StartLiftHoming(void);
bool MotorDriverX42S_ClearProtection(char axis);
bool MotorDriverX42S_ClearStoredAutorun(void);
bool MotorDriverX42S_TakeLiftStatus(X42S_StatusData *status);
bool MotorDriverX42S_TakeLiftConfig(X42S_ConfigData *config);
bool MotorDriverX42S_TakeError(X42S_ErrorData *error);
bool MotorDriverX42S_TakeEvent(X42S_EventData *event);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_X42S_H */
