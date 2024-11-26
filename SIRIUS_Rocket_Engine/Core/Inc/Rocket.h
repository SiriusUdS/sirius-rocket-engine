#pragma once

#include "Sensors/PressureSensor.h"

#define PRESSURE_SENSOR_AMOUNT 1

typedef struct {
  PressureSensor pressureSensors[PRESSURE_SENSOR_AMOUNT];

}
Rocket;

extern uint8_t Rocket_init(Rocket* instance);

extern uint8_t Rocket_executeIdle(Rocket* instance);