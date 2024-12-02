#include "../Inc/Rocket.h"

static volatile Rocket rocket;

uint8_t Rocket_init() {
  rocket.test = 1;
  return 0;
}

uint8_t Rocket_executeIdle() {
  uint8_t test = 0;
  return 0;
}