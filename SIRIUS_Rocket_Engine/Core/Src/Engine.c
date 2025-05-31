#include "../Inc/Engine.h"

static volatile Engine engine;

uint32_t lastTimestamp_ms;

uint16_t telemetryTimestampBufferIndex = 0;
uint16_t telemetryBufferIndex = 0;

static uint8_t uartRxBuffer[sizeof(BoardCommand)];

uint32_t incomingPacketHeader;
uint32_t incomingPacketBoardId;
uint32_t incomingPacketType;

volatile TelemetryTimestampBuffer timestampBuffer = {0};
TelemetryBuffer slowModeTelemetryBuffer = {0};

uint16_t filteredTelemetryValues[ENGINE_ADC_CHANNEL_AMOUNT] = {0};

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
static void initIgniters();

static void initStorageDevices();

static void tickValves(uint32_t timestamp_ms);
static void tickTemperatureSensors();
static void tickIgniters(uint32_t timestamp_ms);

static void handleDataStorage(uint32_t timestamp_ms);
static void handleTelecommunication(uint32_t timestamp_ms);
static void handleIncomingPacket();

static void parsePacket();

static void filterTelemetryValues(uint8_t index);

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
        .type = TELEMETRY_TYPE_CODE,
        .boardId = TELEMETRY_ENGINE_BOARD_ID
      }
    },
    .timestamp_ms = 0,
    .temperatureSensorErrorStatus = {0},
    .pressureSensorErrorStatus = {0},
    .errorStatus = 0,
    .status = 0,
    .crc = 0
  }
};

EngineTelemetryPacket telemetryPacket = {
  .fields = {
    .header = {
      .bits = {
        .type = TELEMETRY_TYPE_CODE,
        .boardId = TELEMETRY_ENGINE_BOARD_ID
      }
    },
    .timestamp_ms = 0,
    .adcValues = {0},
    .crc = 0
  }
};

void Engine_init(PWM* pwms, ADC12* adc, GPIO* gpios, UART* uart, volatile USB* usb, Valve* valves, TemperatureSensor* temperatureSensors, Telecommunication* telecom, Storage* storageDevices, ADCBuffer* dmaAdcBuffer, ADCTimestampsBuffer* dmaAdcTimestampsBuffer, Igniter* igniters) {
  engine.errorStatus.value  = 0;
  engine.status.value       = 0;
  engine.status.bits.state  = ENGINE_STATE_INIT;
  
  engine.dataGatheringMode = DATA_GATHERING_MODE_FAST;

  engine.pwms   = pwms;
  engine.adc    = adc;
  engine.gpios  = gpios;
  engine.uart   = uart;
  engine.usb    = usb;

  engine.valves = valves;
  engine.temperatureSensors = temperatureSensors;
  engine.telecommunication = telecom;
  engine.igniters = igniters;

  engine.storageDevices = storageDevices;
  engine.sdCardBufferPosition = 0;

  engine.dmaAdcBuffer = dmaAdcBuffer;
  engine.dmaAdcTimestampsBuffer = dmaAdcTimestampsBuffer;

  initValves();
  initTemperatureSensors();
  initTelecom();
  initIgniters();

  initStorageDevices();
  initADC();
  initPWMs();
  initGPIOs();
  initUART();
  initUSB();

  engine.uart->status.bits.txReady = 1;
}

void Engine_tick(uint32_t timestamp_ms) {
  tickTemperatureSensors(timestamp_ms);
  tickValves(timestamp_ms);
  engine.telecommunication->tick((struct Telecommunication*)engine.telecommunication, timestamp_ms);
  engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].tick(&engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], timestamp_ms);

  handleDataStorage(timestamp_ms);
  handleTelecommunication(timestamp_ms);
  handleIncomingPacket();

  Engine_execute(timestamp_ms);
  // TEST
  //executeTest(timestamp_ms);
}

void Engine_execute(uint32_t timestamp_ms) {
  switch (engine.status.bits.state) {
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
  engine.status.bits.state = ENGINE_STATE_IDLE;

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
  for (uint8_t i = 0; i < ENGINE_IGNITER_AMOUNT; i++) {
    engine.igniters[i].ignite(&engine.igniters[i], timestamp_ms);
  }
  //engine.valves[ENGINE_IPA_VALVE_INDEX].open((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], timestamp_ms);
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
  HAL_UART_Receive_DMA((UART_HandleTypeDef*)engine.uart->externalHandle, uartRxBuffer, sizeof(uartRxBuffer));
}

void initUSB() {
  if (engine.usb->init == FUNCTION_NULL_POINTER) {
    engine.usb->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  engine.usb->init((struct USB*)engine.usb);
}

void initIgniters() {
  engine.igniters[0].gpio = &engine.gpios[ENGINE_IGNITER_1_GPIO_INDEX];
  engine.igniters[1].gpio = &engine.gpios[ENGINE_IGNITER_2_GPIO_INDEX];

  for (uint8_t i = 0; i < ENGINE_IGNITER_AMOUNT; i++) {
    if (engine.igniters[i].init == FUNCTION_NULL_POINTER) {
      engine.igniters[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.igniters[i].init((struct Igniter*)&engine.igniters[i]);
  }
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

#pragma region TICKS
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

void tickIgniters(uint32_t timestamp_ms) {
  for (uint8_t i = 0; i < ENGINE_IGNITER_AMOUNT; i++) {
    engine.igniters[i].tick((struct Igniter*)&engine.igniters[i], timestamp_ms);
  }
}
#pragma endregion

void handleDataStorage(uint32_t timestamp_ms) {
  // FOR FAST MODE, REPLACE LAST 4 BYTES WITH TIMESTAMP
  if (engine.dataGatheringMode == DATA_GATHERING_MODE_FAST) {
    if (engine.adc->status.bits.dmaFull) {
      engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].store((struct Storage*)&engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], STORAGE_ADC_DESTINATION, engine.dmaAdcBuffer->hex + (sizeof(ADCBuffer) / 2), (sizeof(ADCBuffer) / 2));
      engine.adc->status.bits.dmaFull = 0;
    }
  
    if (engine.adc->status.bits.dmaHalfFull) {
      engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].store((struct Storage*)&engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], STORAGE_ADC_DESTINATION, engine.dmaAdcBuffer->hex, (sizeof(ADCBuffer) / 2));
      engine.adc->status.bits.dmaHalfFull = 0;
    }
  }
  else {
    if (engine.status.bits.timestampBufferReady == 1) {
      engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].store((struct Storage*)&engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], STORAGE_ADC_DESTINATION, timestampBuffer.hex, sizeof(TelemetryTimestampBuffer));
      engine.status.bits.timestampBufferReady = 0;
    }

    if (engine.status.bits.slowModeBufferReady == 1) {
      engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].store((struct Storage*)&engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], STORAGE_ADC_DESTINATION, slowModeTelemetryBuffer.hex, sizeof(TelemetryBuffer));
      engine.status.bits.slowModeBufferReady = 0;
    }
  }
}

void handleTelecommunication(uint32_t timestamp_ms) {
  if (engine.telecommunicationTimestampTarget_ms <= timestamp_ms) {
    if (engine.telecommunicationTelemetryPacketCount >= TELEMETRY_PACKETS_BETWEEN_STATUS_PACKETS) {
      engine.telecommunicationTelemetryPacketCount = 0;
      sendStatusPacket(timestamp_ms);
    }
    else {
      sendTelemetryPacket(timestamp_ms);
      engine.telecommunicationTelemetryPacketCount++;
    }
    engine.telecommunicationTimestampTarget_ms = timestamp_ms + TIME_BETWEEN_TELEMETRY_PACKETS_MS;
  }
  //getReceivedCommand();
}

void handleIncomingPacket() {
  if (engine.telecommunication->uart->status.bits.rxDataReady == 1) {
    // CHECK CRC
    parsePacket();
    engine.telecommunication->uart->status.bits.rxDataReady = 0;
    HAL_UART_Receive_DMA((UART_HandleTypeDef*)engine.uart->externalHandle, uartRxBuffer, sizeof(uartRxBuffer));
  }
}

void parsePacket() {
  if (incomingPacketType == BOARD_COMMAND_TYPE_CODE && incomingPacketBoardId == TELEMETRY_ENGINE_BOARD_ID) {
    for (uint8_t i = 0; i < sizeof(BoardCommand); i++) {
      currentCommand.data[i] = uartRxBuffer[i];
    }
  }
}

void sendTelemetryPacket(uint32_t timestamp_ms) {
  telemetryPacket.fields.timestamp_ms = timestamp_ms;
  for (uint8_t i = 0; i < ENGINE_ADC_CHANNEL_AMOUNT; i++) {
    // DEFER FILTER AWAY SO THAT THIS FUNCTION DOES ONLY PACKET STUFF
    filterTelemetryValues(i);
    telemetryPacket.fields.adcValues[i] = filteredTelemetryValues[i];
    slowModeTelemetryBuffer.data[telemetryBufferIndex + i] = filteredTelemetryValues[i];
  }
  telemetryBufferIndex+=ENGINE_ADC_CHANNEL_AMOUNT;
  if (telemetryBufferIndex >= sizeof(TelemetryBuffer)) {
    engine.status.bits.slowModeBufferReady = 1;
  }
  telemetryPacket.fields.crc = 0;

  if (engine.uart->status.bits.txReady == 1) {
    HAL_UART_Transmit_DMA((UART_HandleTypeDef*)engine.uart->externalHandle, telemetryPacket.data, sizeof(EngineTelemetryPacket));
    engine.uart->status.bits.txReady = 0;
  }
  //engine.telecommunication->sendData((struct Telecommunication*)engine.telecommunication, telemetryPacket.data, sizeof(EngineTelemetryPacket));
  #ifdef USB_ENABLED
    //engine.usb->transmit((struct USB*)engine.usb, telemetryPacket.data, sizeof(EngineTelemetryPacket));
  #endif
}

void sendStatusPacket(uint32_t timestamp_ms) {
  statusPacket.fields.timestamp_ms = timestamp_ms;
  statusPacket.fields.errorStatus = engine.errorStatus.value;
  statusPacket.fields.status = engine.status.value;
  //statusPacket.fields.pressureSensorErrorStatus[ENGINE_NOS_TANK_PRESSURE_SENSOR_INDEX] = engine.pressureSensors[ENGINE_NOS_TANK_PRESSURE_SENSOR_INDEX].errorStatus.value;
  //statusPacket.fields.pressureSensorErrorStatus[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_INDEX] = engine.pressureSensors[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_INDEX].errorStatus.value;
  //statusPacket.fields.temperatureSensorErrorStatus[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_INDEX] = engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_INDEX].errorStatus.value;
  statusPacket.fields.valveStatus[ENGINE_NOS_VALVE_INDEX] = engine.valves[ENGINE_NOS_VALVE_INDEX].status.value;
  statusPacket.fields.valveStatus[ENGINE_IPA_VALVE_INDEX] = engine.valves[ENGINE_IPA_VALVE_INDEX].status.value;
  statusPacket.fields.crc = 0;

  if (engine.uart->status.bits.txReady == 1) {
    HAL_UART_Transmit_DMA((UART_HandleTypeDef*)engine.uart->externalHandle, statusPacket.data, sizeof(EngineStatusPacket));
    engine.uart->status.bits.txReady = 0;
  }
  //engine.telecommunication->sendData((struct Telecommunication*)engine.telecommunication, statusPacket.data, sizeof(EngineStatusPacket));
  #ifdef USB_ENABLED
    //engine.usb->transmit((struct USB*)engine.usb, statusPacket.data, sizeof(EngineStatusPacket));
  #endif
}

void getReceivedCommand() {
  engine.telecommunication->receiveData((struct Telecommunication*)engine.telecommunication, currentCommand.data, sizeof(BoardCommand));
  #ifdef USB_ENABLED
    if (engine.usb->status.bits.rxDataReady == 1) {
      // header
      for (uint8_t i = 0; i < 4; i++) {
        currentCommand.data[i] = engine.usb->rxBuffer[i];
      }

      if (currentCommand.fields.header.bits.type == TELEMETRY_TYPE_CODE &&
          currentCommand.fields.header.bits.boardId == TELEMETRY_ENGINE_BOARD_ID) {
        for (uint8_t i = 4; i < sizeof(BoardCommand); i++) {
          currentCommand.data[i] = engine.usb->rxBuffer[i];
        }
      }
      
      engine.usb->status.bits.rxDataReady = 0;
    }
  #endif
}

void filterTelemetryValues(uint8_t index) {
  uint32_t filteredValue = 0;
  for (uint16_t i = 0; i < 64; i++) {
    filteredValue += engine.dmaAdcBuffer->values[index + i*FILTER_TELEMETRY_OFFSET];
  }
  filteredTelemetryValues[index] = (uint16_t)(filteredValue >> 6UL);
}

uint8_t checkCommandCrc() {
  return 0;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  if (engine.adc->status.bits.dmaHalfFull == 0) {
    engine.adc->status.bits.dmaHalfFull = 1;
    if (DATA_GATHERING_MODE_FAST == engine.dataGatheringMode && engine.status.bits.timestampBufferReady == 0) {
      timestampBuffer.data[telemetryTimestampBufferIndex] = HAL_GetTick();
      telemetryTimestampBufferIndex++;
      if (telemetryTimestampBufferIndex >= sizeof(TelemetryTimestampBuffer)) {
        engine.status.bits.timestampBufferReady = 1;
      }
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (engine.adc->status.bits.dmaFull == 0) {
    engine.adc->status.bits.dmaFull = 1;
    if (DATA_GATHERING_MODE_FAST == engine.dataGatheringMode) {
      timestampBuffer.data[telemetryTimestampBufferIndex] = HAL_GetTick();
      telemetryTimestampBufferIndex++;
      if (telemetryTimestampBufferIndex >= sizeof(TelemetryTimestampBuffer)) {
        engine.status.bits.timestampBufferReady = 1;
      }
    }
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    incomingPacketHeader = uartRxBuffer[0] & uartRxBuffer[1] << 8 & uartRxBuffer[2] << 16 & uartRxBuffer[3] << 24;
    incomingPacketType = incomingPacketHeader & 0xFFFFF000UL;
    incomingPacketBoardId = incomingPacketHeader & 0x000000E0;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    if (engine.uart->status.bits.rxDataReady == 0) {
      // CHECK CRC, IF NOT GOOD, RAISE FLAG FOR NEXT STATUS PACKET
      engine.uart->status.bits.rxDataReady = 1;
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    if (engine.uart->status.bits.txReady == 0) {
      engine.uart->status.bits.txReady = 1;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    HAL_UART_Receive_DMA((UART_HandleTypeDef*)engine.telecommunication->uart->externalHandle, uartRxBuffer, sizeof(uartRxBuffer));
  }
}