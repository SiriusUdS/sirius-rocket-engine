#include "../Inc/Rocket.h"

static volatile Rocket rocket;

static void executeIdle();

void Rocket_init() {
  rocket.currentState = ROCKET_STATE_IDLE;
  rocket.status.bits.notInitialized = 0;
}

void Rocket_execute() {
  switch (rocket.currentState) {
    case ROCKET_STATE_IDLE:
      executeIdle();
      break;
    default:
      rocket.status.bits.invalidState = 1;
      executeIdle();
      break;
  }
}

void executeIdle() {
  
}

