#pragma once

#include "../sirius-embedded-common/Inc/Sensor/PressureSensor/PressureSensor.h"
#include "../sirius-embedded-common/sirius-headers-common/PressureSensor/PressureSensorData.h"

#include "../sirius-embedded-common/Inc/Sensor/TemperatureSensor/NTC3950.h"
#include "../sirius-embedded-common/sirius-headers-common/TemperatureSensor/TemperatureSensorData.h"

#include "../sirius-embedded-common/Inc/Device/Valve/SG90.h"
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
  
  PressureSensorData    pressureSensorsCurrentData[ENGINE_PRESSURE_SENSOR_AMOUNT];
  TemperatureSensorData temperatureSensorsCurrentData[ENGINE_THERMISTANCE_AMOUNT];

  ADC12*            adc;
  PWM  *            pwms;

  Valve*             valves;
  TemperatureSensor* temperatureSensors;
  PressureSensor*    pressureSensors;
}
Engine;

extern void Engine_init(PWM* pwms, ADC12* adc, Valve* valves, TemperatureSensor* temperatureSensors);

extern void Engine_execute();