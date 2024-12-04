#pragma once

#include "Sensors/PressureSensor.h"

#define PRESSURE_SENSOR_AMOUNT 1

#define ENGINE_STATE_IDLE 0

typedef union {
  struct {
    uint16_t notInitialized : 1;
    uint16_t invalidState : 1;
    uint16_t reserved : 14;
  }
  bits;
  uint16_t value;
}
EngineStatus;

typedef struct {
  EngineStatus status;
  uint16_t currentState;
  PressureSensor pressureSensors[PRESSURE_SENSOR_AMOUNT];
}
Engine;

extern void Engine_init();

extern void Engine_execute();