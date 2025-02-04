#include "../Inc/Engine.h"

static volatile Engine engine;

static void executeIdle();

static void initPWMs();

static void initValves();

void Engine_init(PWM* pwms, Valve* valves) {
  engine.errorStatus.value  = 0;
  engine.status.value       = 0;
  engine.currentState       = ENGINE_STATE_IDLE;
  

  engine.pwms   = pwms;
  engine.valves = valves;

  initPWMs();
  initValves();
}

void initPWMs() {
  for (uint8_t i = 0; i < ENGINE_PWM_AMOUNT; i++) {
    if (engine.pwms[i].init == FUNCTION_NULL_POINTER) {
      engine.pwms[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.pwms[i].init((struct PWM*)&engine.pwms[i]);
  }
}

void initValves() {
  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    if (engine.valves[i].init == FUNCTION_NULL_POINTER) {
      engine.valves[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.valves[i].init((struct Valve*)&engine.valves[i]);
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
  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    engine.valves[i].gatherData((struct Valve*)&engine.valves[i]);
  }
}

