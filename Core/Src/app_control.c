#include "app_control.h"
#include "bluetooth.h"
#include "motor.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define JOYSTICK_DEADZONE      8       // 摇杆死区
#define VX_SCALE               1      // 前后缩放到 -100~100
#define VY_SCALE               10      // 左右平移缩放
#define WZ_SCALE               8       // 转弯缩放，别太猛

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

            Mecanum_SetSpeed(vx, vy, wz);
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
