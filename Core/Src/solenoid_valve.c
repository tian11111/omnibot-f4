#include "solenoid_valve.h"

static bool s_valve_is_on = false;
static bool s_pulse_active = false;
static uint32_t s_pulse_deadline = 0U;


/* 初始化 -- 上电后断电 */
void SolenoidValve_Init(void)
{
    SolenoidValve_Off();
}

void SolenoidValve_Set(bool enabled)
{
    HAL_GPIO_WritePin(SOLENOID_VALVE_GPIO_Port,
                      SOLENOID_VALVE_Pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);

    s_valve_is_on = enabled;
    s_pulse_active = false;
}

/* 打开 */
void SolenoidValve_On(void)
{
    SolenoidValve_Set(true);
}

/* 关闭 */
void SolenoidValve_Off(void)
{
    SolenoidValve_Set(false);
}

/*  切换状态  */
void SolenoidValve_Toggle(void)
{
    SolenoidValve_Set(!s_valve_is_on);
}

/*  定时脉冲 让电磁阀打开指定时间，然后自动关闭 */
bool SolenoidValve_Pulse(uint32_t duration_ms)
{
    if ((duration_ms == 0U) || (duration_ms > SOLENOID_VALVE_MAX_PULSE_MS))
    {
        return false;
    }

    SolenoidValve_On();
    s_pulse_deadline = HAL_GetTick() + duration_ms;
    s_pulse_active = true;
    return true;
}

/* 检查脉冲是否结束 */
void SolenoidValve_Task(void)
{
    if (s_pulse_active &&
        ((int32_t)(HAL_GetTick() - s_pulse_deadline) >= 0))
    {
        SolenoidValve_Off();
    }
}

bool SolenoidValve_IsOn(void)
{
    return s_valve_is_on;
}
