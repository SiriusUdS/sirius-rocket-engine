#include "../Inc/Engine.h"

static volatile Engine engine;

static void executeIdle();

void Engine_init() {
  engine.currentState = ENGINE_STATE_IDLE;
  engine.status.bits.notInitialized = 0;
}

void Engine_execute() {
  switch (engine.currentState) {
    case ENGINE_STATE_IDLE:
      executeIdle();
      break;
    default:
      engine.status.bits.invalidState = 1;
      executeIdle();
      break;
  }
}

void executeIdle() {
  
}

