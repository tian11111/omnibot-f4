#include "app_control.h"
#include "bluetooth.h"
#include "motor_driver_dc4ch.h"
#include "motor_closedloop.h"
#include "motor_driver_X42S.h"
#include "usart.h"
#include "solenoid_valve.h"
#include "gpio.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define JOYSTICK_DEADZONE      8
#define AXIS_SCALE             8
#define MOTOR_MAX_RPM          300U
#define MOTOR_ACCEL            10U
#define MECANUM_PWM_MAX        100
#define JOYSTICK_TIMEOUT_MS    300U

static uint32_t g_last_joystick_tick = 0U;
static uint8_t g_joystick_command_active = 0U;
static uint8_t g_estop_active = 0U;

static const char *App_X42SLiftStateName(X42S_LiftState state)
{
    switch (state)
    {
    case X42S_LIFT_STATE_CONFIGURING:
        return "configuring";
    case X42S_LIFT_STATE_HOMING:
        return "homing";
    case X42S_LIFT_STATE_ENABLING:
        return "enabling";
    case X42S_LIFT_STATE_READY:
        return "ready";
    case X42S_LIFT_STATE_FAULT:
        return "fault";
    case X42S_LIFT_STATE_UNHOMED:
    default:
        return "unhomed";
    }
}

static const char *App_X42SHomeStateName(const X42S_StatusData *status)
{
    if ((status->home_flags & 0x08U) != 0U)
    {
        return "failed";
    }
    if ((status->home_flags & 0x04U) != 0U)
    {
        return "running";
    }
    return (status->homed_valid != 0U) ? "ready" : "unhomed";
}

static void App_X42SDiagnosticTask(void)
{
    char tx_buf[BT_TX_PACKET_MAX_LEN];
    X42S_StatusData status;
    X42S_ConfigData config;
    X42S_ErrorData error;
    X42S_EventData event;

    if (MotorDriverX42S_TakeLiftStatus(&status))
    {
        (void)snprintf(tx_buf, sizeof(tx_buf),
                       "[x42s:status,fw=Emm,v=%umV,i=%umA,"
                       "rpm=%c%u,err=%c%lu.%02ludeg,"
                       "clog=%u,protect=%u,en=%u,home=%s,sw=%s]\r\n",
                       status.bus_voltage_mv,
                       status.phase_current_ma,
                       status.speed_negative ? '-' : '+',
                       status.speed_rpm,
                       status.position_error_negative ? '-' : '+',
                       (unsigned long)(status.position_error_x100_deg / 100U),
                       (unsigned long)(status.position_error_x100_deg % 100U),
                       (status.motor_flags >> 2) & 1U,
                       (status.motor_flags >> 3) & 1U,
                       status.motor_flags & 1U,
                       App_X42SHomeStateName(&status),
                       App_X42SLiftStateName(status.software_state));
        Bluetooth_SendString(tx_buf);
    }

    if (MotorDriverX42S_TakeLiftConfig(&config))
    {
        (void)snprintf(tx_buf, sizeof(tx_buf),
                       "[x42s:config,fw=Emm,baud=%lu,Ma_Limit=%umA,"
                       "clogmode=%u,clogrpm=%u,clogcur=%umA,clogms=%u]\r\n",
                       (unsigned long)config.uart_baud_rate,
                       config.max_current_ma,
                       config.clog_mode,
                       config.clog_speed_rpm,
                       config.clog_current_ma,
                       config.clog_time_ms);
        Bluetooth_SendString(tx_buf);
    }

    if (MotorDriverX42S_TakeError(&error))
    {
        (void)snprintf(tx_buf, sizeof(tx_buf),
                       "[x42s:error,axis=%c,func=%02X,code=%02X,source=%s]\r\n",
                       error.axis, error.function_code, error.status_code,
                       (error.source == X42S_ERROR_SOURCE_DRIVER) ?
                       "driver" : "parser");
        Bluetooth_SendString(tx_buf);
    }

    while (MotorDriverX42S_TakeEvent(&event))
    {
        switch (event.type)
        {
        case X42S_EVENT_HOME_RUNNING:
            Bluetooth_SendString("[x42s:home,axis=x,state=running]\r\n");
            break;
        case X42S_EVENT_HOME_READY:
            Bluetooth_SendString("[x42s:home,axis=x,state=ready]\r\n");
            break;
        case X42S_EVENT_HOME_FAILED:
            Bluetooth_SendString("[x42s:home,axis=x,state=failed]\r\n");
            break;
        case X42S_EVENT_LIMIT:
            (void)snprintf(tx_buf, sizeof(tx_buf),
                           "[x42s:limit,axis=%c,dir=%s]\r\n",
                           event.axis,
                           (event.direction > 0) ? "up" : "down");
            Bluetooth_SendString(tx_buf);
            break;
        case X42S_EVENT_STALL_FAULT:
            (void)snprintf(tx_buf, sizeof(tx_buf),
                           "[x42s:fault,axis=%c,reason=stall]\r\n",
                           event.axis);
            Bluetooth_SendString(tx_buf);
            break;
        case X42S_EVENT_RX_TIMEOUT:
            (void)snprintf(tx_buf, sizeof(tx_buf),
                           "[x42s:fault,axis=%c,reason=rx-timeout]\r\n",
                           event.axis);
            Bluetooth_SendString(tx_buf);
            break;
        case X42S_EVENT_HOME_TIMEOUT:
            Bluetooth_SendString("[x42s:fault,axis=x,reason=home-timeout]\r\n");
            break;
        case X42S_EVENT_UNHOMED:
            Bluetooth_SendString("[x42s:fault,axis=x,reason=unhomed]\r\n");
            break;
        case X42S_EVENT_PROTECTION_CLEARED:
            (void)snprintf(tx_buf, sizeof(tx_buf),
                           "[x42s:protection,axis=%c,state=cleared]\r\n",
                           event.axis);
            Bluetooth_SendString(tx_buf);
            break;
        case X42S_EVENT_AUTOHOME_DISABLED:
            (void)snprintf(tx_buf, sizeof(tx_buf),
                           "[x42s:auto-home,axis=%c,state=disabled]\r\n",
                           event.axis);
            Bluetooth_SendString(tx_buf);
            break;
        case X42S_EVENT_AUTORUN_CLEARED:
            (void)snprintf(tx_buf, sizeof(tx_buf),
                           "[x42s:autorun,axis=%c,state=cleared]\r\n",
                           event.axis);
            Bluetooth_SendString(tx_buf);
            break;
        case X42S_EVENT_COMM_RECOVERED:
            (void)snprintf(tx_buf, sizeof(tx_buf),
                           "[x42s:comm,axis=%c,state=recovered]\r\n",
                           event.axis);
            Bluetooth_SendString(tx_buf);
            break;
        case X42S_EVENT_ENABLE_FAILED:
            Bluetooth_SendString("[x42s:fault,axis=x,reason=enable-failed]\r\n");
            break;
        default:
            break;
        }
    }
}

/* 自动绘图控制 */
uint8_t g_auto_plot_enable = 0;
uint16_t g_auto_plot_interval_ms = 100;  /* 默认100ms发送一次 */
static uint32_t g_last_plot_time = 0;

/* -------------------- Gripper stepper subsystem -------------------- */

static int16_t clamp_i16(int16_t v, int16_t lo, int16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int16_t limit_value(int16_t val, int16_t min, int16_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

static uint16_t abs_i16_to_u16(int16_t value)
{
    return (uint16_t)((value < 0) ? -value : value);
}

#if 0
static void Motor_SetSignedSpeed(Emm42_Handle *motor, int16_t speed)
{
    bool ccw = false;
    uint16_t rpm = 0U;

    speed = limit_value(speed, -(int16_t)MOTOR_MAX_RPM, (int16_t)MOTOR_MAX_RPM);

    if (speed < 0)
    {
        ccw = true;
        rpm = abs_i16_to_u16(speed);
    }
    else
    {
        rpm = (uint16_t)speed;
    }

    (void)Emm42_SetSpeed(motor, ccw, rpm, MOTOR_ACCEL, false);
}
#endif

#if 0
void Gripper_Init(void)
{
    Emm42_Init(&motor_x, &huart6, 1U);  /* M1: USART6 */
    Emm42_Init(&motor_y, &huart2, 1U);  /* M2: USART2 */

    (void)Emm42_Enable(&motor_x, true, false);
    HAL_Delay(10);
    (void)Emm42_Enable(&motor_y, true, false);
    HAL_Delay(10);
}
#endif

#if 0
void Gripper_SetSpeed(int16_t x_speed, int16_t y_speed)
{
    Motor_SetSignedSpeed(&motor_x, x_speed);
    HAL_Delay(2);
    Motor_SetSignedSpeed(&motor_y, y_speed);
}
#endif

#if 0
void Gripper_StopAll(void)
{
    (void)Emm42_StopNow(&motor_x, false);
    HAL_Delay(2);
    (void)Emm42_StopNow(&motor_y, false);
}
#endif

/* -------------------- Mecanum DC subsystem -------------------- */
void Mecanum_Init(void)
{
    DC4_Motor_Init();
    DC4_Motor_Start();
    DC4_Motor_AllStop();
    //MotorClosedLoop_Init();
   // MotorClosedLoop_Start();
}

void Mecanum_SetMotion(int16_t vx, int16_t vy, int16_t wz)
{
    int16_t x = clamp_i16(vx, -MECANUM_PWM_MAX, MECANUM_PWM_MAX);
    int16_t y = clamp_i16(vy, -MECANUM_PWM_MAX, MECANUM_PWM_MAX);
    int16_t w = clamp_i16(wz, -MECANUM_PWM_MAX, MECANUM_PWM_MAX);

    int32_t fl = (int32_t)x + (int32_t)y + (int32_t)w;
    int32_t fr = (int32_t)x - (int32_t)y - (int32_t)w;
    int32_t bl = (int32_t)x - (int32_t)y + (int32_t)w;
    int32_t br = (int32_t)x + (int32_t)y - (int32_t)w;

    int32_t maxv = abs(fl);
    if (abs(fr) > maxv) maxv = abs(fr);
    if (abs(bl) > maxv) maxv = abs(bl);
    if (abs(br) > maxv) maxv = abs(br);

    if (maxv > MECANUM_PWM_MAX)
    {
        fl = (int32_t)((int64_t)fl * MECANUM_PWM_MAX / maxv);
        fr = (int32_t)((int64_t)fr * MECANUM_PWM_MAX / maxv);
        bl = (int32_t)((int64_t)bl * MECANUM_PWM_MAX / maxv);
        br = (int32_t)((int64_t)br * MECANUM_PWM_MAX / maxv);
    }

    /* Physical channel order verified by wheel-up pair-intersection tests. */
    DC4_Motor_SetSignedSpeed(0, (int16_t)fr);
    DC4_Motor_SetSignedSpeed(1, (int16_t)fl);
    DC4_Motor_SetSignedSpeed(2, (int16_t)bl);
    DC4_Motor_SetSignedSpeed(3, (int16_t)br);
}

void Mecanum_StopAll(void)
{
    DC4_Motor_SetSignedSpeed(0, 0);
    DC4_Motor_SetSignedSpeed(1, 0);
    DC4_Motor_SetSignedSpeed(2, 0);
    DC4_Motor_SetSignedSpeed(3, 0);
}

/* -------------------- Protocol -------------------- */
static void App_ParseJoystickPacket(const char *packet)
{
    char type[20] = {0};
    int lx = 0, ly = 0, rx = 0, ry = 0;

    if (sscanf(packet, "%19[^,],%d,%d,%d,%d", type, &lx, &ly, &rx, &ry) == 5)
    {
        if (strcmp(type, "joystick") == 0)
        {
#ifdef USE_MECANUM
            if (g_estop_active == 0U)
            {
                int16_t vx = (int16_t)clamp_i16((int16_t)(ly), -100, 100);
                /* App lx > 0 means move right; invert it to match the chassis Y axis. */
                int16_t vy = (int16_t)-clamp_i16((int16_t)(lx), -100, 100);
                /* App steering-wheel RX direction is opposite to chassis yaw. */
                int16_t wz = (int16_t)-clamp_i16((int16_t)(rx), -100, 100);
                Mecanum_SetMotion(vx, vy, wz);
                g_last_joystick_tick = HAL_GetTick();
                g_joystick_command_active = 1U;
            }
#endif
        }
        else if (strcmp(type, "gripper") == 0)
        {
            /* TODO: 迁移至 motor_driver_X42S 驱动 */
#if 0
            int16_t gx = (int16_t)clamp_i16((int16_t)lx, -300, 300);
            int16_t gy = (int16_t)clamp_i16((int16_t)ly, -300, 300);
            Gripper_SetSpeed(gx, gy);
#endif
        }
        else if (strcmp(type, "pid") == 0)
        {
            uint8_t motor_idx = (uint8_t)lx;
            float kp = (float)ly / 100.0f;
            float ki = (float)rx / 100.0f;
            float kd = (float)ry / 100.0f;

            if (motor_idx < 4)
            {
                MotorClosedLoop_SetPIDParams(motor_idx, kp, ki, kd);
            }
            else if (motor_idx == 4)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    MotorClosedLoop_SetPIDParams(i, kp, ki, kd);
                }
            }
        }
    }
}

void App_ControlTask(void)
{
    if (BT_RxFlag == 1)
    {
        /* 回显接收到的指令 */
        Bluetooth_SendString("[rx:");
        Bluetooth_SendString(BT_RxPacket);
        Bluetooth_SendString("]\r\n");

        /* ---- X42S 步进升降电机指令 ---- */
        if (strncmp(BT_RxPacket, "gripper,", 8) == 0)
        {
            int x_speed = 0, y_speed = 0;
            if ((sscanf(BT_RxPacket, "gripper,%d,%d", &x_speed, &y_speed) == 2) &&
                (x_speed >= -3000) && (x_speed <= 3000) &&
                (y_speed >= -3000) && (y_speed <= 3000))
            {
                if (g_estop_active == 0U)
                {
                    MotorDriverX42S_SetDualSpeed((int16_t)x_speed,
                                                  (int16_t)y_speed);
                }
            }
            else
            {
                Bluetooth_SendString("[x42s:error,gripper-range=-3000..3000]\r\n");
            }
        }
        else if (strncmp(BT_RxPacket, "x42s,level,", 11) == 0)
        {
            int level_speed = 0;
            char trailing = '\0';

            if ((sscanf(BT_RxPacket, "x42s,level,%d%c",
                        &level_speed, &trailing) == 1) &&
                (level_speed >= -X42S_LEVEL_MAX_SPEED_RPM) &&
                (level_speed <= X42S_LEVEL_MAX_SPEED_RPM))
            {
                if (g_estop_active == 0U)
                {
                    MotorDriverX42S_SetFrontLevelSpeed((int16_t)level_speed);
                }
            }
            else
            {
                Bluetooth_SendString(
                    "[x42s:error,level-range=-300..300]\r\n");
            }
        }
		else if (strcmp(BT_RxPacket, "x42s,status") == 0)
		{
			MotorDriverX42S_QueryLiftStatus();
		}
		else if (strcmp(BT_RxPacket, "x42s,config") == 0)
		{
			MotorDriverX42S_QueryLiftConfig();
		}
		else if (strcmp(BT_RxPacket, "x42s,home,x") == 0)
		{
			if ((g_estop_active != 0U) ||
			    !MotorDriverX42S_StartLiftHoming())
			{
				Bluetooth_SendString("[x42s:home,axis=x,state=busy]\r\n");
			}
		}
		else if (strcmp(BT_RxPacket, "x42s,clear,x") == 0)
		{
			if (!MotorDriverX42S_ClearProtection('x'))
			{
				Bluetooth_SendString("[x42s:protection,axis=x,state=busy]\r\n");
			}
		}
		else if (strcmp(BT_RxPacket, "x42s,clear,y") == 0)
		{
			if (!MotorDriverX42S_ClearProtection('y'))
			{
				Bluetooth_SendString("[x42s:protection,axis=y,state=busy]\r\n");
			}
		}
		else if (strcmp(BT_RxPacket, "x42s,autorun,clear") == 0)
		{
			if (!MotorDriverX42S_ClearStoredAutorun())
			{
				Bluetooth_SendString("[x42s:autorun,state=busy]\r\n");
			}
		}
				/* ---- 电磁阀1：控制夹爪开闭 ---- */
				/* 电磁阀打开 */
        else if (strcmp(BT_RxPacket, "valve1,on") == 0)
        {
            SolenoidValve1_On();
            Bluetooth_SendString("[valve1:on]\r\n");
        }/* 电磁阀关闭 */
        else if (strcmp(BT_RxPacket, "valve1,off") == 0)
        {
            SolenoidValve1_Off();
            Bluetooth_SendString("[valve1:off]\r\n");
        }/* 电磁阀翻转开关 */
        else if (strcmp(BT_RxPacket, "valve1,toggle") == 0)
        {
            SolenoidValve1_Toggle();
            Bluetooth_SendString(SolenoidValve1_IsOn() ?
                                 "[valve1:on]\r\n" : "[valve1:off]\r\n");
        }/* 电磁阀发送脉冲 */
        else if (strncmp(BT_RxPacket, "valve1,pulse,", 13) == 0)
        {
            unsigned long duration_ms = 0UL;

            if ((sscanf(BT_RxPacket, "valve1,pulse,%lu", &duration_ms) == 1) &&
                SolenoidValve1_Pulse((uint32_t)duration_ms))
            {
                Bluetooth_SendString("[valve1:pulse]\r\n");
            }
            else
            {
                Bluetooth_SendString("[valve1:error,pulse-range=1..60000]\r\n");
            }
        }/* 判断电磁阀状态 */
        else if (strcmp(BT_RxPacket, "valve1,query") == 0)
        {
            Bluetooth_SendString(SolenoidValve1_IsOn() ?
                                 "[valve1:on]\r\n" : "[valve1:off]\r\n");
        }

				/* ---- 电磁阀2：控制夹爪前后伸缩 ---- */
        else if (strcmp(BT_RxPacket, "valve2,on") == 0)
        {
            SolenoidValve2_On();
            Bluetooth_SendString("[valve2:on]\r\n");
        }
        else if (strcmp(BT_RxPacket, "valve2,off") == 0)
        {
            SolenoidValve2_Off();
            Bluetooth_SendString("[valve2:off]\r\n");
        }
        else if (strcmp(BT_RxPacket, "valve2,toggle") == 0)
        {
            SolenoidValve2_Toggle();
            Bluetooth_SendString(SolenoidValve2_IsOn() ?
                                 "[valve2:on]\r\n" : "[valve2:off]\r\n");
        }
        else if (strncmp(BT_RxPacket, "valve2,pulse,", 13) == 0)
        {
            unsigned long duration_ms = 0UL;

            if ((sscanf(BT_RxPacket, "valve2,pulse,%lu", &duration_ms) == 1) &&
                SolenoidValve2_Pulse((uint32_t)duration_ms))
            {
                Bluetooth_SendString("[valve2:pulse]\r\n");
            }
            else
            {
                Bluetooth_SendString("[valve2:error,pulse-range=1..60000]\r\n");
            }
        }
        else if (strcmp(BT_RxPacket, "valve2,query") == 0)
        {
            Bluetooth_SendString(SolenoidValve2_IsOn() ?
                                 "[valve2:on]\r\n" : "[valve2:off]\r\n");
        }
        else if (strcmp(BT_RxPacket, "query") == 0)
        {
            Bluetooth_SendMotorStatus();
        }
        else if (strcmp(BT_RxPacket, "plot") == 0 || strcmp(BT_RxPacket, "p") == 0)
        {
            /* 发送四个电机的当前速度 */
            int32_t s0 = MotorClosedLoop_GetCurrentSpeed(0);
            int32_t s1 = MotorClosedLoop_GetCurrentSpeed(1);
            int32_t s2 = MotorClosedLoop_GetCurrentSpeed(2);
            int32_t s3 = MotorClosedLoop_GetCurrentSpeed(3);
            Bluetooth_SendPlotData(s0, s1, s2, s3);
        }
        else if (strcmp(BT_RxPacket, "plot-clear") == 0 || strcmp(BT_RxPacket, "p-c") == 0)
        {
            Bluetooth_SendPlotClear();
        }
        else if (strcmp(BT_RxPacket, "auto") == 0)
        {
            /* 开始自动发送绘图数据 */
            g_auto_plot_enable = 1;
            g_last_plot_time = HAL_GetTick();
            Bluetooth_SendString("[auto:start]\r\n");
        }
        else if (strcmp(BT_RxPacket, "stop") == 0)
        {
            /* 停止自动发送绘图数据 */
            g_auto_plot_enable = 0;
            Bluetooth_SendString("[auto:stop]\r\n");
        }
        else
        {
            App_ParseJoystickPacket(BT_RxPacket);
        }
        BT_RxFlag = 0;
    }

#ifdef USE_MECANUM
    /*
     * Fail-safe: stop the chassis if joystick packets disappear because of
     * Bluetooth disconnects or prolonged packet loss.  Unsigned subtraction
     * keeps the timeout check correct across the HAL tick wraparound.
     */
    if ((g_joystick_command_active != 0U) &&
        ((uint32_t)(HAL_GetTick() - g_last_joystick_tick) > JOYSTICK_TIMEOUT_MS))
    {
        Mecanum_StopAll();
        g_joystick_command_active = 0U;
    }
#endif

    App_X42SDiagnosticTask();
}

/* ---- PG4 急停检测（按键切换） ---- */
/* 消抖间隔 200ms，避免单次按下多次翻转 */
#define ESTOP_DEBOUNCE_MS  200U

void App_EmergencyStopCheck(void)
{
    static uint32_t s_last_press_tick = 0;
    static uint8_t  s_last_level      = 0xFF; /* 未初始化 */

    uint8_t level = (uint8_t)HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_4);

    /* 首次调用跳过边沿检测，避免上电误触发 */
    if (s_last_level == 0xFF) {
        s_last_level = level;
        return;
    }

    /* 下降沿检测：高到低，且距离上次翻转 > 消抖时间 */
    if (s_last_level == 1 && level == 0 &&
        (HAL_GetTick() - s_last_press_tick) >= ESTOP_DEBOUNCE_MS)
    {
        s_last_press_tick = HAL_GetTick();
        g_estop_active = (g_estop_active != 0U) ? 0U : 1U;

        if (g_estop_active != 0U)
        {
            Mecanum_StopAll();
            g_joystick_command_active = 0U;
            MotorDriverX42S_StopAll();
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);  /* PD9 LED on (active low) */
        }
        else
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);    /* PD9 LED off */
        }
    }

    /* 急停锁住期间持续强制停止电机 */
    if (g_estop_active != 0U)
    {
        Mecanum_StopAll();
    }

    s_last_level = level;
}

/* 自动发送绘图数据（需要在主循环中调用） */
void App_AutoPlotTask(void)
{
    if (g_auto_plot_enable)
    {
        uint32_t now = HAL_GetTick();
        if (now - g_last_plot_time >= g_auto_plot_interval_ms)
        {
            g_last_plot_time = now;
            int32_t s0 = MotorClosedLoop_GetCurrentSpeed(0);
            int32_t s1 = MotorClosedLoop_GetCurrentSpeed(1);
            int32_t s2 = MotorClosedLoop_GetCurrentSpeed(2);
            int32_t s3 = MotorClosedLoop_GetCurrentSpeed(3);
            Bluetooth_SendPlotData(s0, s1, s2, s3);
        }
    }
}

