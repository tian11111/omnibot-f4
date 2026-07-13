#include "solenoid_valve.h"

static bool s1_valve_is_on = false;
static bool s1_pulse_active = false;
static uint32_t s1_pulse_deadline = 0U;
static bool s2_valve_is_on = false;
static bool s2_pulse_active = false;
static uint32_t s2_pulse_deadline = 0U;

/* 初始化 -- 上电关断 */
void SolenoidValve_Init(void)
{
    SolenoidValve1_Off();
    SolenoidValve2_Off();
}

/* ---- Valve 1 ---- */

void SolenoidValve1_Set(bool enabled)
{
    HAL_GPIO_WritePin(SOLENOID_VALVE1_GPIO_Port,
                      SOLENOID_VALVE1_Pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    s1_valve_is_on = enabled;
    s1_pulse_active = false;
}

void SolenoidValve1_On(void)
{
    SolenoidValve1_Set(true);
}

void SolenoidValve1_Off(void)
{
    SolenoidValve1_Set(false);
}

void SolenoidValve1_Toggle(void)
{
    SolenoidValve1_Set(!s1_valve_is_on);
}

bool SolenoidValve1_Pulse(uint32_t duration_ms)
{
    if ((duration_ms == 0U) || (duration_ms > SOLENOID_VALVE_MAX_PULSE_MS))
    {
        return false;
    }
    SolenoidValve1_On();
    s1_pulse_deadline = HAL_GetTick() + duration_ms;
    s1_pulse_active = true;
    return true;
}

bool SolenoidValve1_IsOn(void)
{
    return s1_valve_is_on;
}

/* ---- Valve 2 ---- */

void SolenoidValve2_Set(bool enabled)
{
    HAL_GPIO_WritePin(SOLENOID_VALVE2_GPIO_Port,
                      SOLENOID_VALVE2_Pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    s2_valve_is_on = enabled;
    s2_pulse_active = false;
}

void SolenoidValve2_On(void)
{
    SolenoidValve2_Set(true);
}

void SolenoidValve2_Off(void)
{
    SolenoidValve2_Set(false);
}

void SolenoidValve2_Toggle(void)
{
    SolenoidValve2_Set(!s2_valve_is_on);
}

bool SolenoidValve2_Pulse(uint32_t duration_ms)
{
    if ((duration_ms == 0U) || (duration_ms > SOLENOID_VALVE_MAX_PULSE_MS))
    {
        return false;
    }
    SolenoidValve2_On();
    s2_pulse_deadline = HAL_GetTick() + duration_ms;
    s2_pulse_active = true;
    return true;
}

bool SolenoidValve2_IsOn(void)
{
    return s2_valve_is_on;
}

/* ---- 通用任务 : 扫描脉冲是否到期 ---- */

void SolenoidValve_Task(void)
{
    if (s1_pulse_active &&
        ((int32_t)(HAL_GetTick() - s1_pulse_deadline) >= 0))
    {
        SolenoidValve1_Off();
    }
    if (s2_pulse_active &&
        ((int32_t)(HAL_GetTick() - s2_pulse_deadline) >= 0))
    {
        SolenoidValve2_Off();
    }
}
