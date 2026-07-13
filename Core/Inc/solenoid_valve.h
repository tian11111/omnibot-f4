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
void SolenoidValve_Task(void);

void SolenoidValve1_Set(bool enabled);
void SolenoidValve2_Set(bool enabled);
void SolenoidValve1_On(void);
void SolenoidValve2_On(void);
void SolenoidValve1_Off(void);
void SolenoidValve2_Off(void);
void SolenoidValve1_Toggle(void);
void SolenoidValve2_Toggle(void);
bool SolenoidValve1_Pulse(uint32_t duration_ms);
bool SolenoidValve2_Pulse(uint32_t duration_ms);
bool SolenoidValve1_IsOn(void);
bool SolenoidValve2_IsOn(void);

#endif /* __SOLENOID_VALVE_H */
