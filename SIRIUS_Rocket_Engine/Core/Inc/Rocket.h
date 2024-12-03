#pragma once

#include "Sensors/PressureSensor.h"

#define PRESSURE_SENSOR_AMOUNT 1

#define ROCKET_STATE_IDLE 0

typedef union {
  struct {
    uint16_t notInitialized : 1;
    uint16_t invalidState : 1;
    uint16_t reserved : 14;
  }
  bits;
  uint16_t value;
}
RocketStatus;

typedef struct {
  RocketStatus status;
  uint16_t currentState;
  PressureSensor pressureSensors[PRESSURE_SENSOR_AMOUNT];
}
Rocket;

extern void Rocket_init();

extern void Rocket_execute();