#include "app_control.h"
#include "bluetooth.h"
#include "motor_driver_emm42.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define JOYSTICK_DEADZONE      8       // 摇杆死区
#define VX_SCALE               1      // 前后缩放到 -100~100
#define VY_SCALE               10      // 左右平移缩放
#define WZ_SCALE               8       // 转弯缩放，别太猛
#define MOTOR_MAX_RPM          300U
#define MOTOR_ACCEL            10U

Emm42_Handle motor_fl;  // 左前
Emm42_Handle motor_fr;  // 右前
Emm42_Handle motor_rl;  // 左后
Emm42_Handle motor_rr;  // 右后

static int16_t apply_deadzone(int16_t val, int16_t deadzone)
{
    if (val > -deadzone && val < deadzone)
    {
        return 0;
    }
    return val;
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

static void normalize_wheel_rpm(int16_t *fl, int16_t *fr, int16_t *rl, int16_t *rr)
{
    uint16_t max_abs = abs_i16_to_u16(*fl);

    if (abs_i16_to_u16(*fr) > max_abs) max_abs = abs_i16_to_u16(*fr);
    if (abs_i16_to_u16(*rl) > max_abs) max_abs = abs_i16_to_u16(*rl);
    if (abs_i16_to_u16(*rr) > max_abs) max_abs = abs_i16_to_u16(*rr);

    if (max_abs > MOTOR_MAX_RPM)
    {
        *fl = (int16_t)((int32_t)(*fl) * MOTOR_MAX_RPM / max_abs);
        *fr = (int16_t)((int32_t)(*fr) * MOTOR_MAX_RPM / max_abs);
        *rl = (int16_t)((int32_t)(*rl) * MOTOR_MAX_RPM / max_abs);
        *rr = (int16_t)((int32_t)(*rr) * MOTOR_MAX_RPM / max_abs);
    }
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

void Mecanum_Init(void)
{
    Emm42_Init(&motor_fl, &huart1, 1U);
    Emm42_Init(&motor_fr, &huart2, 1U);
    Emm42_Init(&motor_rl, &huart4, 1U);
    Emm42_Init(&motor_rr, &huart5, 1U);

    (void)Emm42_Enable(&motor_fl, true, false);
    HAL_Delay(10);
    (void)Emm42_Enable(&motor_fr, true, false);
    HAL_Delay(10);
    (void)Emm42_Enable(&motor_rl, true, false);
    HAL_Delay(10);
    (void)Emm42_Enable(&motor_rr, true, false);
    HAL_Delay(10);
}

void Mecanum_SetVelocity(int16_t vx, int16_t vy, int16_t wz)
{
    int16_t fl;
    int16_t fr;
    int16_t rl;
    int16_t rr;

    fl = vx + vy + wz;
    fr = vx - vy - wz;
    rl = vx - vy + wz;
    rr = vx + vy - wz;

    normalize_wheel_rpm(&fl, &fr, &rl, &rr);

    Motor_SetSignedSpeed(&motor_fl, fl);
    HAL_Delay(2);
    Motor_SetSignedSpeed(&motor_fr, fr);
    HAL_Delay(2);
    Motor_SetSignedSpeed(&motor_rl, rl);
    HAL_Delay(2);
    Motor_SetSignedSpeed(&motor_rr, rr);
}

static void App_ParseJoystickPacket(const char *packet)
{
    char type[20] = {0};
    int lx = 0, ly = 0, rx = 0, ry = 0;

    // 解析格式：joystick,a,b,c,d
    if (sscanf(packet, "%19[^,],%d,%d,%d,%d", type, &lx, &ly, &rx, &ry) == 5)
    {
        if (strcmp(type, "joystick") == 0)
        {
            int16_t vx, vy, wz;

            // 死区处理
            lx = apply_deadzone((int16_t)lx, JOYSTICK_DEADZONE);
            ly = apply_deadzone((int16_t)ly, JOYSTICK_DEADZONE);
            rx = apply_deadzone((int16_t)rx, JOYSTICK_DEADZONE);
            ry = apply_deadzone((int16_t)ry, JOYSTICK_DEADZONE);

            // 映射关系：
            // 左摇杆上下 -> 前后 vx
            // 左摇杆左右 -> 转弯 wz
            // 右摇杆左右 -> 左右平移 vy
            vx = (int16_t)(ly * VX_SCALE);
            wz = (int16_t)(lx * WZ_SCALE);
            vy = (int16_t)(rx * VY_SCALE);

            // 限幅
            vx = limit_value(vx, -1000, 1000);
            vy = limit_value(vy, -1000, 1000);
            wz = limit_value(wz, -1000, 1000);

            Mecanum_SetVelocity(vx, vy, wz);
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
