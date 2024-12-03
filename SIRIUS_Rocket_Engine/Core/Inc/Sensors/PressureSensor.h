#pragma once

#include <stdint.h>

struct PressureSensor;

typedef void (*PressureSensor_init)(struct PressureSensor* instance);

typedef struct {
  PressureSensor_init initFunction;
}
PressureSensor;

extern void PressureSensor_initDefault(PressureSensor* instance);