#pragma once

#include <stdint.h>

struct PressureSensor;

typedef uint8_t (*PressureSensor_Init)(struct PressureSensor* instance);

typedef struct {
  PressureSensor_Init initFunction;
}
PressureSensor;

extern uint8_t PressureSensor_InitDefault(PressureSensor* instance);