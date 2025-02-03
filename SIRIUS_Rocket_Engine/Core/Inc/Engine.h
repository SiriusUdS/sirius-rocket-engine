#pragma once

#include "../sirius-embedded-common/Inc/Sensor/PressureSensor/PressureSensor.h"
#include "../sirius-embedded-common/Inc/Device/Valve/Valve.h"

#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineStatus.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineErrorStatus.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineSensors.h"

#define PRESSURE_SENSOR_AMOUNT 1
#define TEMPERATURE_SENSOR_AMOUNT 8

typedef enum {
  ENGINE_STATE_IDLE
}
EngineState;

typedef struct {
  EngineErrorStatus errorStatus;
  EngineStatus      status;

  EngineState       currentState;
  PressureSensor    pressureSensors[PRESSURE_SENSOR_AMOUNT];
  Valve**           valves;
}
Engine;

extern void Engine_init(Valve** valves);

extern void Engine_execute();