#include "../Inc/Engine.h"

static volatile Engine engine;

uint32_t lastTimestamp_ms;

uint16_t adcTimestampIndex;

uint8_t sdCardBuffer[32768] = {0};
uint8_t sdCardTimestampBuffer[2048] = {0};

static void executeInit(uint32_t timestamp_ms);
static void executeIdle(uint32_t timestamp_ms);
static void executeAbort(uint32_t timestamp_ms);
static void executeTest(uint32_t timestamp_ms);
static void executeArming(uint32_t timestamp_ms);
static void executeIgnition(uint32_t timestamp_ms);
static void executePoweredFlight(uint32_t timestamp_ms);
static void executeUnpoweredFlight(uint32_t timestamp_ms);

static void initPWMs();
static void initADC();
static void initGPIOs();
static void initUART();
static void initUSB();

static void initValves();
static void initTemperatureSensors();
static void initTelecom();

static void initStorageDevices();

static void tickValves(uint32_t timestamp_ms);
static void tickTemperatureSensors();

static void handleDataStorage(uint32_t timestamp_ms);
static void handleTelecommunication(uint32_t timestamp_ms);
static void handleCurrentCommand(uint8_t currentState);

static void sendTelemetryPacket(uint32_t timestamp_ms);
static void sendStatusPacket(uint32_t timestamp_ms);
static uint32_t computeCrc(uint8_t* data, uint16_t size);

static void getReceivedCommand();
static uint8_t checkCommandCrc();

BoardCommand currentCommand = {0};

EngineStatusPacket statusPacket = {
  .fields = {
    .header = {
      .bits = {
        .type = TELEMETRY_HEADER_TYPE_STATUS,
        .boardId = TELEMETRY_ENGINE_BOARD_ID
      }
    },
    .timestamp_ms = 0,
    .temperatureSensorErrorStatus = {0},
    .pressureSensorErrorStatus = {0},
    .engineErrorStatus = 0,
    .engineStatus = 0,
    .crc = 0
  }
};

EngineTelemetryPacket telemetryPacket = {
  .fields = {
    .header = {
      .bits = {
        .type = TELEMETRY_HEADER_TYPE_TELEMETRY,
        .boardId = TELEMETRY_ENGINE_BOARD_ID
      }
    },
    .timestamp_ms = 0,
    .adcValues = {0},
    .crc = 0
  }
};

void Engine_init(PWM* pwms, ADC12* adc, GPIO* gpios, UART* uart, volatile USB* usb, Valve* valves, TemperatureSensor* temperatureSensors, Telecommunication* telecom, Storage* storageDevices, ADCBuffer* dmaAdcBuffer, ADCTimestampsBuffer* dmaAdcTimestampsBuffer) {
  engine.errorStatus.value  = 0;
  engine.status.value       = 0;
  engine.currentState       = ENGINE_STATE_INIT;
  
  engine.dataGatheringMode = DATA_GATHERING_MODE_FAST;

  engine.pwms   = pwms;
  engine.adc    = adc;
  engine.gpios  = gpios;
  engine.uart   = uart;
  engine.usb    = usb;

  engine.valves = valves;
  engine.temperatureSensors = temperatureSensors;
  engine.telecommunication = telecom;

  engine.storageDevices = storageDevices;
  engine.sdCardBufferPosition = 0;

  engine.dmaAdcBuffer = dmaAdcBuffer;
  engine.dmaAdcTimestampsBuffer = dmaAdcTimestampsBuffer;

  adcTimestampIndex = 0;

  initValves();
  initTemperatureSensors();
  initTelecom();

  initStorageDevices();
  initADC();
  initPWMs();
  initGPIOs();
  initUART();
  initUSB();
}

void Engine_tick(uint32_t timestamp_ms) {
  tickTemperatureSensors(timestamp_ms);
  tickValves(timestamp_ms);
  engine.telecommunication->tick((struct Telecommunication*)engine.telecommunication, timestamp_ms);
  engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].tick(&engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], timestamp_ms);

  handleDataStorage(timestamp_ms);
  handleTelecommunication(timestamp_ms);

  Engine_execute(timestamp_ms);
  // TEST
  //executeTest(timestamp_ms);
}

void Engine_execute(uint32_t timestamp_ms) {
  switch (engine.currentState) {
    case ENGINE_STATE_INIT:
      executeInit(timestamp_ms);
      break;
    case ENGINE_STATE_IDLE:
      executeIdle(timestamp_ms);
      break;
    case ENGINE_STATE_ARMING:
      executeArming(timestamp_ms);
      break;
    case ENGINE_STATE_IGNITION:
      executeIgnition(timestamp_ms);
      break;
    case ENGINE_STATE_POWERED_FLIGHT:
      executePoweredFlight(timestamp_ms);
      break;
    case ENGINE_STATE_UNPOWERED_FLIGHT:
      executeUnpoweredFlight(timestamp_ms);
      break;
    case ENGINE_STATE_ABORT:
      executeAbort(timestamp_ms);
      break;
    default:
      engine.errorStatus.bits.invalidState = 1;
      executeIdle(timestamp_ms);
      break;
  }
}

void executeInit(uint32_t timestamp_ms) {
  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    engine.valves[i].close((struct Valve*)&engine.valves[i], timestamp_ms);
  }
  engine.currentState = ENGINE_STATE_IDLE;

  engine.telecommunication->config((struct Telecommunication*) engine.telecommunication);
}

void executeIdle(uint32_t timestamp_ms) {
  uint8_t test = 0;
}

void executeAbort(uint32_t timestamp_ms) {
  // Check flowcharts for wtf to do
}

void executeArming(uint32_t timestamp_ms) {
  // Wait for ignition command, completely armed
}

void executeIgnition(uint32_t timestamp_ms) {
  engine.valves[ENGINE_IPA_VALVE_INDEX].open((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], timestamp_ms);
}

void executePoweredFlight(uint32_t timestamp_ms) {
  // save as much data as possible
}

void executeUnpoweredFlight(uint32_t timestamp_ms) {
  // stay mostly idle
}

void executeTest(uint32_t timestamp_ms) {
  
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
    engine.adc->init((struct ADC12*)engine.adc, engine.dmaAdcBuffer, ENGINE_ADC_CHANNEL_AMOUNT);
  }

  for (uint8_t i = 0; i < ENGINE_ADC_CHANNEL_AMOUNT; i++) {
    if (engine.adc->channels[i].init == FUNCTION_NULL_POINTER) {
      engine.adc->channels[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.adc->channels[i].init((struct ADC12Channel*)&engine.adc->channels[i]);
  }
}

void initGPIOs() {
  for (uint8_t i = 0; i < ENGINE_GPIO_AMOUNT; i++) {
    if (engine.gpios[i].init == FUNCTION_NULL_POINTER) {
      engine.gpios[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.gpios[i].init((struct GPIO*)&engine.gpios[i]);
  }
}

void initUART() {
  if (engine.uart->init == FUNCTION_NULL_POINTER) {
    engine.uart->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  engine.uart->init((struct UART*)engine.uart);
}

void initUSB() {
  if (engine.usb->init == FUNCTION_NULL_POINTER) {
    engine.usb->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  engine.usb->init((struct USB*)engine.usb);
}

void initValves() {
  engine.valves[ENGINE_IPA_VALVE_INDEX].pwm = &engine.pwms[ENGINE_IPA_VALVE_PWM_INDEX];
  engine.valves[ENGINE_IPA_VALVE_INDEX].gpio[VALVE_GPIO_OPENED_INDEX] = &engine.gpios[ENGINE_IPA_VALVE_OPENED_GPIO_INDEX];
  engine.valves[ENGINE_IPA_VALVE_INDEX].gpio[VALVE_GPIO_CLOSED_INDEX] = &engine.gpios[ENGINE_IPA_VALVE_CLOSED_GPIO_INDEX];

  engine.valves[ENGINE_NOS_VALVE_INDEX].pwm = &engine.pwms[ENGINE_NOS_VALVE_PWM_INDEX];
  engine.valves[ENGINE_NOS_VALVE_INDEX].gpio[VALVE_GPIO_OPENED_INDEX] = &engine.gpios[ENGINE_NOS_VALVE_OPENED_GPIO_INDEX];
  engine.valves[ENGINE_NOS_VALVE_INDEX].gpio[VALVE_GPIO_CLOSED_INDEX] = &engine.gpios[ENGINE_NOS_VALVE_CLOSED_GPIO_INDEX];

  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    if (engine.valves[i].init == FUNCTION_NULL_POINTER) {
      engine.valves[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.valves[i].init((struct Valve*)&engine.valves[i]);
  }
}

void initTemperatureSensors() {
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_ADC_CHANNEL_INDEX];

  for (uint8_t i = 0; i < ENGINE_TEMPERATURE_SENSOR_AMOUNT; i++) {
    if (engine.temperatureSensors[i].init == FUNCTION_NULL_POINTER) {
      engine.temperatureSensors[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.temperatureSensors[i].init((struct TemperatureSensor*)&engine.temperatureSensors[i]);
  }
}

void initTelecom() {
  if(engine.telecommunication->init == FUNCTION_NULL_POINTER){
    engine.telecommunication->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  engine.telecommunication->init((struct Telecommunication*)engine.telecommunication);
  engine.telecommunication->uart = engine.uart;
}

void initStorageDevices() {
  for (uint8_t i = 0; i < ENGINE_STORAGE_AMOUNT; i++) {
    if (engine.storageDevices[i].init == FUNCTION_NULL_POINTER) {
      engine.storageDevices[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.storageDevices[i].init((struct Storage*)&engine.storageDevices[i]);
  }
}

void tickValves(uint32_t timestamp_ms) {
  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    engine.valves[i].tick((struct Valve*)&engine.valves[i], timestamp_ms);
  }
}

void tickTemperatureSensors() {
  for (uint8_t i = 0; i < ENGINE_TEMPERATURE_SENSOR_AMOUNT; i++) {
    engine.temperatureSensors[i].tick((struct TemperatureSensor*)&engine.temperatureSensors[i]);
  }
}

void handleDataStorage(uint32_t timestamp_ms) {
  if (engine.dataGatheringMode == DATA_GATHERING_MODE_FAST || engine.storageTimestampTarget_ms <= timestamp_ms) {
    engine.storageTimestampTarget_ms = timestamp_ms + STORAGE_DELAY_BETWEEN_SLOW_SAVES_MS;

    if (engine.adc->status.bits.dmaFull) {
      engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].store((struct Storage*)&engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], STORAGE_ADC_DESTINATION, engine.dmaAdcBuffer->hex + (sizeof(ADCBuffer) / 2), (sizeof(ADCBuffer) / 2));
      engine.adc->status.bits.dmaFull = 0;
    }
  
    if (engine.adc->status.bits.dmaHalfFull) {
      engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].store((struct Storage*)&engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], STORAGE_ADC_DESTINATION, engine.dmaAdcBuffer->hex, (sizeof(ADCBuffer) / 2));
      engine.adc->status.bits.dmaHalfFull = 0;
    }
  }
  

  /*if (engine.sdCardTimestampsBufferFull) {
    engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].store(&engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], STORAGE_ADC_TIMESTAMP_DESTINATION, sdCardTimestampBuffer, sizeof(sdCardTimestampBuffer));

    engine.sdCardTimestampsBufferFull = 0;
    adcTimestampIndex = 0;
  }*/
}

void handleTelecommunication(uint32_t timestamp_ms) {
  if (engine.telecommunicationTimestampTarget_ms <= timestamp_ms) {
    if (engine.telecommunicationTelemetryPacketCount > TELEMETRY_PACKETS_BETWEEN_STATUS_PACKETS) {
      engine.telecommunicationTelemetryPacketCount = 0;
      sendStatusPacket(timestamp_ms);
    }
    else {
      sendTelemetryPacket(timestamp_ms);
      engine.telecommunicationTelemetryPacketCount++;
    }
    engine.telecommunicationTimestampTarget_ms = timestamp_ms + TIME_BETWEEN_TELEMETRY_PACKETS_MS;
  }
  getReceivedCommand();
  /*if (engine.usb->status.bits.rxDataReady == 1) {
    //uint8_t* test = engine.usb->rxBuffer;
    // header
    for (uint8_t i = 0; i < 4; i++) {
      currentCommand.data[i] = engine.usb->rxBuffer[i];
    }

    for (uint8_t i = 4; i < sizeof(BoardCommand); i++) {
      currentCommand.data[i] = engine.usb->rxBuffer[i];
    }
    //currentCommand.data = engine.usb->rxBuffer;
    engine.usb->status.bits.rxDataReady = 0;
  }*/
}

void handleCurrentCommand(uint8_t currentState) {
  switch (currentCommand.fields.header.bits.commandCode)
  {
    //case :
      //break;
  
    default:
      break;
  }
}

void sendTelemetryPacket(uint32_t timestamp_ms) {
  telemetryPacket.fields.timestamp_ms = timestamp_ms;
  for (uint8_t i = 0; i < ENGINE_ADC_CHANNEL_AMOUNT; i++) {
    telemetryPacket.fields.adcValues[i] = engine.dmaAdcBuffer->values[i];
  }
  telemetryPacket.fields.crc = 0;
  engine.telecommunication->sendData((struct Telecommunication*)engine.telecommunication, telemetryPacket.data, sizeof(EngineTelemetryPacket));
  #ifdef USB_ENABLED
    engine.usb->transmit((struct USB*)engine.usb, telemetryPacket.data, sizeof(EngineTelemetryPacket));
  #endif
}

void sendStatusPacket(uint32_t timestamp_ms) {
  statusPacket.fields.timestamp_ms = timestamp_ms;
  statusPacket.fields.engineErrorStatus = engine.errorStatus.value;
  statusPacket.fields.engineStatus = engine.status.value;
  statusPacket.fields.pressureSensorErrorStatus[ENGINE_NOS_TANK_PRESSURE_SENSOR_INDEX] = engine.pressureSensors[ENGINE_NOS_TANK_PRESSURE_SENSOR_INDEX].errorStatus.value;
  statusPacket.fields.pressureSensorErrorStatus[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_INDEX] = engine.pressureSensors[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_INDEX].errorStatus.value;
  statusPacket.fields.temperatureSensorErrorStatus[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_INDEX] = engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_INDEX].errorStatus.value;
  statusPacket.fields.valvesStatus[ENGINE_NOS_VALVE_INDEX] = engine.valves[ENGINE_NOS_VALVE_INDEX].status.value;
  statusPacket.fields.valvesStatus[ENGINE_IPA_VALVE_INDEX] = engine.valves[ENGINE_IPA_VALVE_INDEX].status.value;
  statusPacket.fields.crc = 0;

  engine.telecommunication->sendData((struct Telecommunication*)engine.telecommunication, statusPacket.data, sizeof(EngineStatusPacket));
  #ifdef USB_ENABLED
    engine.usb->transmit((struct USB*)engine.usb, statusPacket.data, sizeof(EngineStatusPacket));
  #endif
}

void getReceivedCommand() {
  engine.telecommunication->receiveData((struct Telecommunication*)engine.telecommunication, currentCommand.data, sizeof(BoardCommand));
}

uint8_t checkCommandCrc() {
  return 0;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  if (engine.adc->status.bits.dmaHalfFull == 0) {
    engine.adc->status.bits.dmaHalfFull = 1;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (engine.adc->status.bits.dmaFull == 0) {
    engine.adc->status.bits.dmaFull = 1;
    /*memcpy(sdCardBuffer + engine.sdCardBufferPosition + sizeof(uint32_t), engine.dmaAdcBuffer->hex, sizeof(ADCBuffer));
    engine.sdCardBufferPosition+=sizeof(ADCBuffer);

    if (engine.sdCardBufferPosition >= sizeof(sdCardBuffer)) {
      engine.adc->status.bits.dmaFull = 1;
    }*/
  }
  
  /*if (engine.sdCardTimestampsBufferFull == 0) {
    sdCardTimestampBuffer[adcTimestampIndex++] = HAL_GetTick();
    if (adcTimestampIndex >= sizeof(ADCTimestampsBuffer)) {
      engine.sdCardTimestampsBufferFull = 1;
    }
  }*/
}