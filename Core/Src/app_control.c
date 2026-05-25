#include "app_control.h"
#include "bluetooth.h"
#include "motor_driver_emm42.h"
#include "motor_driver_dc4ch.h"
#include "motor_closedloop.h"
#include "usart.h"

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

/* -------------------- Gripper stepper subsystem -------------------- */
static Emm42_Handle motor_x;   /* X axis - UART6 (M1) */
static Emm42_Handle motor_y;   /* Y axis - UART2 (M2) */

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

void Gripper_Init(void)
{
    Emm42_Init(&motor_x, &huart6, 1U);  /* M1: USART6 */
    Emm42_Init(&motor_y, &huart2, 1U);  /* M2: USART2 */

    (void)Emm42_Enable(&motor_x, true, false);
    HAL_Delay(10);
    (void)Emm42_Enable(&motor_y, true, false);
    HAL_Delay(10);
}

void Gripper_SetSpeed(int16_t x_speed, int16_t y_speed)
{
    Motor_SetSignedSpeed(&motor_x, x_speed);
    HAL_Delay(2);
    Motor_SetSignedSpeed(&motor_y, y_speed);
}

void Gripper_StopAll(void)
{
    (void)Emm42_StopNow(&motor_x, false);
    HAL_Delay(2);
    (void)Emm42_StopNow(&motor_y, false);
}

/* -------------------- Mecanum DC subsystem -------------------- */
void Mecanum_Init(void)
{
    DC4_Motor_Init();
    DC4_Motor_Start();
    DC4_Motor_AllStop();
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

    DC4_Motor_SetSignedSpeed(0, (int16_t)fl);
    DC4_Motor_SetSignedSpeed(1, (int16_t)fr);
    DC4_Motor_SetSignedSpeed(2, (int16_t)bl);
    DC4_Motor_SetSignedSpeed(3, (int16_t)br);
}

void Mecanum_StopAll(void)
{
    DC4_Motor_AllStop();
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
            int16_t vx = (int16_t)clamp_i16((int16_t)(ly), -100, 100);
            int16_t vy = (int16_t)clamp_i16((int16_t)(lx), -100, 100);
            int16_t wz = (int16_t)clamp_i16((int16_t)(rx), -100, 100);
            Mecanum_SetMotion(vx, vy, wz);
#endif
        }
        else if (strcmp(type, "gripper") == 0)
        {
            int16_t gx = (int16_t)clamp_i16((int16_t)lx, -300, 300);
            int16_t gy = (int16_t)clamp_i16((int16_t)ly, -300, 300);
            Gripper_SetSpeed(gx, gy);
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
        App_ParseJoystickPacket(BT_RxPacket);
        BT_RxFlag = 0;
    }
}

