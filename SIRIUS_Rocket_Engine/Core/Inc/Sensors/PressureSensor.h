#pragma once

#include <stdint.h>

struct PressureSensor;

typedef uint8_t (*PressureSensor_init)(struct PressureSensor* instance);

typedef struct {
  PressureSensor_init initFunction;
}
PressureSensor;

extern uint8_t PressureSensor_initDefault(PressureSensor* instance);