#pragma once

#include "../sirius-embedded-common/Inc/Sensor/PressureSensor/PressureSensor.h"

#include "../sirius-embedded-common/Inc/Device/Valve/SG90.h"
#include "../sirius-embedded-common/Inc/Device/Valve/HBL388.h"

#include "../sirius-embedded-common/Inc/LowLevelDriver/PWM/PWMHAL.h"

#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineStatus.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineErrorStatus.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineSensors.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineState.h"

#define FUNCTION_NULL_POINTER 0

#define PRESSURE_SENSOR_AMOUNT 1
#define TEMPERATURE_SENSOR_AMOUNT 8

typedef struct {
  EngineErrorStatus errorStatus;
  EngineStatus      status;

  EngineState       currentState;
  PressureSensor    pressureSensors[PRESSURE_SENSOR_AMOUNT];

  PWM  *            pwms;
  Valve*            valves;
}
Engine;

extern void Engine_init(PWM* pwms, Valve* valves);

extern void Engine_execute();