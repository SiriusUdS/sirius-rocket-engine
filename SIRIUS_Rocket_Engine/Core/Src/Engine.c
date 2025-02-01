#include "../Inc/Engine.h"

static volatile Engine engine;

static void executeIdle();

void Engine_init() {
  engine.currentState = ENGINE_STATE_IDLE;
  engine.errorStatus.value - 0;

  for (uint8_t i = 0; i < VALVE_AMOUNT; i++) {
    engine.valves->gatherData = Valve_initDefault;
  }
}

void Engine_execute() {
  switch (engine.currentState) {
    case ENGINE_STATE_IDLE:
      executeIdle();
      break;
    default:
      engine.errorStatus.bits.invalidState = 1;
      executeIdle();
      break;
  }
}

void executeIdle() {
  for (uint8_t i = 0; i < VALVE_AMOUNT; i++) {
    engine.valves->gatherData(&engine.valves[i]);
  }
}

