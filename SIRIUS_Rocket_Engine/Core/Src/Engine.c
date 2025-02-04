#include "../Inc/Engine.h"

static volatile Engine engine;

static void executeIdle();
static void executeArmed();
static void executeIgnition();
static void executePoweredFlight();
static void executeUnpoweredFlight();
static void executeAbort();

static void initPWMs();
static void initValves();

static void tickValves();

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
  engine.valves[ENGINE_IPA_VALVE_INDEX].pwm = &engine.pwms[ENGINE_IPA_VALVE_PWM_INDEX];
  engine.valves[ENGINE_NOS_VALVE_INDEX].pwm = &engine.pwms[ENGINE_NOS_VALVE_PWM_INDEX];
  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    if (engine.valves[i].init == FUNCTION_NULL_POINTER) {
      engine.valves[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.valves[i].init((struct Valve*)&engine.valves[i]);
  }
}

void tickValves() {
  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    engine.valves[i].tick((struct Valve*)&engine.valves[i]);
  }
}

void Engine_execute() {
  switch (engine.currentState) {
    case ENGINE_STATE_IDLE:
      executeIdle();
      break;
    case ENGINE_STATE_ARMED:
      executeArmed();
      break;
    case ENGINE_STATE_IGNITION:
      executeIgnition();
      break;
    case ENGINE_STATE_POWERED_FLIGHT:
      executePoweredFlight();
      break;
    case ENGINE_STATE_UNPOWERED_FLIGHT:
      executeUnpoweredFlight();
      break;
    case ENGINE_STATE_ABORT:
      executeAbort();
      break;
    default:
      engine.errorStatus.bits.invalidState = 1;
      executeIdle();
      break;
  }
}

void executeIdle() {
  tickValves();

  engine.currentState = ENGINE_STATE_ARMED;
}

void executeArmed() {
  tickValves();

  engine.currentState = ENGINE_STATE_IGNITION;
}

void executeIgnition() {
  engine.valves[ENGINE_IPA_VALVE_INDEX].setDutyCycle((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], 10);
  tickValves();
  
  if (!engine.valves[ENGINE_IPA_VALVE_INDEX].status.bits.isOpening && !engine.valves[ENGINE_IPA_VALVE_INDEX].status.bits.isClosing)
  {
    engine.currentState = ENGINE_STATE_POWERED_FLIGHT;
  }
}

void executePoweredFlight() {
  tickValves();
  engine.valves[ENGINE_IPA_VALVE_INDEX].setDutyCycle((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], 5);
}

void executeUnpoweredFlight() {

}

void executeAbort() {

}