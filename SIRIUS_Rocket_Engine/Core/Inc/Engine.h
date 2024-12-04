#pragma once

#include "../sirius-embedded-common/Inc/Sensors/PressureSensor/PressureSensor.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineStatus.h"

#define PRESSURE_SENSOR_AMOUNT 1
#define VALVE_AMOUNT 1
#define TEMPERATURE_SENSOR_AMOUNT 8

typedef enum {
  ENGINE_STATE_IDLE
}
EngineState;

typedef struct {
  EngineStatus status;
  EngineState currentState;
  PressureSensor pressureSensors[PRESSURE_SENSOR_AMOUNT];
}
Engine;

extern void Engine_init();

extern void Engine_execute();