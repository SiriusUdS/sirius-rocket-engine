#include "../Inc/Engine.h"

static volatile Engine engine;

static void executeIdle();
static void executeArmed();
static void executeIgnition();
static void executePoweredFlight();
static void executeUnpoweredFlight();
static void executeAbort();

static void initPWMs();
static void initADC();
static void initGPIOs();
static void initUARTs();

static void initValves();
static void initThermistances();

static void tickValves();
static void tickThermistances();

void Engine_init(PWM* pwms, ADC12* adc, Valve* valves, TemperatureSensor* temperatureSensors) {
  engine.errorStatus.value  = 0;
  engine.status.value       = 0;
  engine.currentState       = ENGINE_STATE_IDLE;
  

  engine.pwms   = pwms;
  engine.adc    = adc;

  engine.valves = valves;
  engine.temperatureSensors = temperatureSensors;

  initValves();
  initThermistances();

  initADC();
  initPWMs();
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

void initADC() {
  if (engine.adc->init == FUNCTION_NULL_POINTER) {
    engine.adc->errorStatus.bits.nullFunctionPointer = 1;
  }
  else {
    engine.adc->init(engine.adc, ENGINE_ADC_CHANNEL_AMOUNT);
  }

  for (uint8_t i = 0; i < ENGINE_ADC_CHANNEL_AMOUNT; i++) {
    if (engine.adc->channels[i].init == FUNCTION_NULL_POINTER) {
      engine.adc->channels[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.adc->channels[i].init((struct ADC12*)&engine.adc->channels[i]);
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

void initThermistances() {
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_ADC_CHANNEL_INDEX];

  for (uint8_t i = 0; i < ENGINE_THERMISTANCE_AMOUNT; i++) {
    if (engine.temperatureSensors[i].init == FUNCTION_NULL_POINTER) {
      engine.temperatureSensors[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.temperatureSensors[i].init((struct TemperatureSensor*)&engine.temperatureSensors[i]);
  }
}

void tickValves() {
  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    engine.valves[i].tick((struct Valve*)&engine.valves[i]);
  }
}

void tickThermistances() {
  for (uint8_t i = 0; i < ENGINE_THERMISTANCE_AMOUNT; i++) {
    engine.temperatureSensorsCurrentData[i] = engine.temperatureSensors[i].readData((struct TemperatureSensor*)&engine.temperatureSensors[i]);
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
  tickThermistances();
  tickValves();

  engine.currentState = ENGINE_STATE_ARMED;
}

void executeArmed() {
  tickThermistances();
  tickValves();

  engine.currentState = ENGINE_STATE_IGNITION;
}

void executeIgnition() {
  tickThermistances();

  engine.valves[ENGINE_IPA_VALVE_INDEX].setDutyCycle((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], 28);
  tickValves();
  
  if (!engine.valves[ENGINE_IPA_VALVE_INDEX].status.bits.isOpening && !engine.valves[ENGINE_IPA_VALVE_INDEX].status.bits.isClosing)
  {
    engine.currentState = ENGINE_STATE_POWERED_FLIGHT;
  }
}

void executePoweredFlight() {
  tickThermistances();

  engine.valves[ENGINE_IPA_VALVE_INDEX].setDutyCycle((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], 72);
  tickValves();

  if (!engine.valves[ENGINE_IPA_VALVE_INDEX].status.bits.isOpening && !engine.valves[ENGINE_IPA_VALVE_INDEX].status.bits.isClosing)
  {
    engine.currentState = ENGINE_STATE_IGNITION;
  }
}

void executeUnpoweredFlight() {
  tickThermistances();
}

void executeAbort() {
  tickThermistances();
}