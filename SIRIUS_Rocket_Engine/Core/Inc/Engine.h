#pragma once

#include "../sirius-embedded-common/Inc/Sensor/PressureSensor/ETM375.h"
#include "../sirius-embedded-common/sirius-headers-common/PressureSensor/PressureSensorData.h"

#include "../sirius-embedded-common/Inc/Sensor/TemperatureSensor/NTC3950.h"
#include "../sirius-embedded-common/sirius-headers-common/TemperatureSensor/TemperatureSensorData.h"

#include "../sirius-embedded-common/Inc/Device/Valve/HBL388.h"

#include "../sirius-embedded-common/Inc/LowLevelDriver/GPIO/GPIOHAL.h"
#include "../sirius-embedded-common/Inc/LowLevelDriver/PWM/PWMHAL.h"
#include "../sirius-embedded-common/Inc/LowLevelDriver/ADC/ADC12HAL.h"
#include "../sirius-embedded-common/Inc/LowLevelDriver/UART/UARTHAL.h"

#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineStatus.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineErrorStatus.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineSensors.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineState.h"

#include "stm32f4xx_hal.h"

#define FUNCTION_NULL_POINTER 0

typedef struct {
  EngineErrorStatus errorStatus;
  EngineStatus      status;

  EngineState       currentState;

  ADC12*            adc;
  PWM  *            pwms;

  Valve*             valves;
  TemperatureSensor* temperatureSensors;
  PressureSensor*    pressureSensors;
}
Engine;

extern void Engine_init(PWM* pwms, ADC12* adc, Valve* valves, TemperatureSensor* temperatureSensors);

extern void Engine_tick(uint32_t timestamp_ms);

extern void Engine_execute(uint32_t timestamp_ms);