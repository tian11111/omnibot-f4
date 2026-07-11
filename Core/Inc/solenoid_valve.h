#ifndef __SOLENOID_VALVE_H
#define __SOLENOID_VALVE_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Board1_1 schematic:
 *   PE10 high -> Q3 on -> Q4 gate low -> VOUT1 enabled.
 * Keep the output low whenever the valve is not intentionally energized.
 */
#define SOLENOID_VALVE_MAX_PULSE_MS  60000U

void SolenoidValve_Init(void);
void SolenoidValve_Set(bool enabled);
void SolenoidValve_On(void);
void SolenoidValve_Off(void);
void SolenoidValve_Toggle(void);
bool SolenoidValve_Pulse(uint32_t duration_ms);
void SolenoidValve_Task(void);
bool SolenoidValve_IsOn(void);

#endif /* __SOLENOID_VALVE_H */
