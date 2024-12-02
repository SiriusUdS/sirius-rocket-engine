#pragma once

#include "Sensors/PressureSensor.h"

#define PRESSURE_SENSOR_AMOUNT 1

typedef struct {
  PressureSensor pressureSensors[PRESSURE_SENSOR_AMOUNT];
  uint8_t test;
}
Rocket;

extern uint8_t Rocket_init();

extern uint8_t Rocket_executeIdle();