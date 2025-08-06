#include "../Inc/Engine.h"

static volatile Engine engine;

uint32_t adcHalfFullTimestamp_ms = 0;
uint32_t adcFullTimestamp_ms = 0;

uint16_t telemetryTimestampBufferIndex = 0;
uint16_t telemetryBufferIndex = 0;

uint32_t timeSinceLastCommand_ms = 0;
uint32_t communicationRestartTimer_ms = 0;
uint32_t lastCommandTimestamp_ms = 0;

uint8_t activateStorageFlag = 0;

uint16_t filteredTelemetryValues[16] = {0};

uint8_t uartRxBuffer[176] = {0};
uint8_t uartRxHalfReady = 0;
uint8_t uartRxCpltReady = 0;

static void executeInit(uint32_t timestamp_ms);
static void executeSafe(uint32_t timestamp_ms);
static void executeUnsafe(uint32_t timestamp_ms);
static void executeAbort(uint32_t timestamp_ms);
static void executeIgnition(uint32_t timestamp_ms);
static void executeLaunch(uint32_t timestamp_ms);

static void executeAbortCommand(uint32_t timestamp_ms);
static void executeIgnitionCommand(uint32_t timestamp_ms);
static void executeLaunchCommand(uint32_t timestamp_ms);


static void initPWMs();
static void initADC();
static void initGPIOs();
static void initUART();

static void initHeaters();
static void initValves();
static void initIgniter();
static void initTemperatureSensors();
static void initTelecom();

static void initStorageDevices();

static void tickValves(uint32_t timestamp_ms);
static void tickTemperatureSensors();

static void handleDataStorage(uint32_t timestamp_ms);
static void handleTelecommunication(uint32_t timestamp_ms);

static void handleCurrentCommand();
static void handleCurrentCommandSafe();
static void handleCommandAcknowledge();
static void handleCurrentCommandUnsafe();
static void handleCurrentCommandIgnite();
static void handleCurrentCommandLaunch();
static void handleCurrentCommandAbort();

static void filterTelemetryValues(uint8_t index);

static void sendTelemetryPacket(uint32_t timestamp_ms);
static void sendStatusPacket(uint32_t timestamp_ms);

static void getReceivedCommand();
static uint8_t checkCommandCrc();

BoardCommand currentCommand = {0};

EngineStatusPacket statusPacket = {
  .fields = {
    .header = {
      .bits = {
        .type = STATUS_TYPE_CODE,
        .boardId = ENGINE_BOARD_ID
      }
    },
    .timestamp_ms = 0,
    .errorStatus = {0},
    .status = {0},
    .valveStatus = {0},
    .crc = 0
  }
};

EngineTelemetryPacket telemetryPacket = {
  .fields = {
    .header = {
      .bits = {
        .type = TELEMETRY_TYPE_CODE,
        .boardId = ENGINE_BOARD_ID
      }
    },
    .timestamp_ms = 0,
    .adcValues = {0},
    .crc = 0
  }
};

void Engine_init(PWM* pwms, ADC12* adc, GPIO* gpios, UART* uart, Valve* valves, Heater* heaters, Igniter* igniter, TemperatureSensor* temperatureSensors, Telecommunication* telecom, Storage* storageDevices, volatile EngineSDCardBuffer* sdCardBuffer, CRC_HandleTypeDef* hcrc) {
  engine.errorStatus.value  = 0;
  engine.status.value       = 0;
  engine.currentState       = ENGINE_STATE_INIT;
  
  engine.isStoringData = 0;

  engine.pwms   = pwms;
  engine.adc    = adc;
  engine.gpios  = gpios;
  engine.uart   = uart;

  engine.valves = valves;
  engine.igniter = igniter;
  engine.heaters = heaters;
  engine.temperatureSensors = temperatureSensors;
  engine.telecommunication = telecom;

  engine.hcrc = hcrc;

  engine.storageDevices = storageDevices;
  engine.sdCardBufferPosition = 0;

  timeSinceLastCommand_ms = 0;
  lastCommandTimestamp_ms = 0;
  communicationRestartTimer_ms = 0;

  engine.sdCardBuffer = sdCardBuffer;

  initValves();
  initIgniter();
  initHeaters();
  initTemperatureSensors();
  initTelecom();

  initStorageDevices();
  initADC();
  initPWMs();
  initGPIOs();
  initUART();
}

void Engine_tick(uint32_t timestamp_ms) {
  timeSinceLastCommand_ms = timestamp_ms - lastCommandTimestamp_ms;
  if (timeSinceLastCommand_ms > 30000) {
    for (;;) {
      // Wait for watchdog to reset the system
    }
  }

  if (timeSinceLastCommand_ms > communicationRestartTimer_ms + 3000) {
    communicationRestartTimer_ms = timeSinceLastCommand_ms;
    if (__HAL_UART_GET_FLAG((UART_HandleTypeDef*)engine.uart->externalHandle, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_OREFLAG((UART_HandleTypeDef*)engine.uart->externalHandle);
    }
    HAL_UART_DMAStop(engine.uart->externalHandle);
    HAL_UART_Receive_DMA(engine.uart->externalHandle, uartRxBuffer, sizeof(uartRxBuffer));
  }

  tickTemperatureSensors(timestamp_ms);
  tickValves(timestamp_ms);
  engine.telecommunication->tick((struct Telecommunication*)engine.telecommunication, timestamp_ms);
  engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].tick((struct Storage*)&engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], timestamp_ms);
  engine.igniter->tick((struct Igniter*)engine.igniter, timestamp_ms);

  engine.status.bits.state = engine.currentState;
  handleDataStorage(timestamp_ms);
  handleTelecommunication(timestamp_ms);

  Engine_execute(timestamp_ms);
}

void Engine_execute(uint32_t timestamp_ms) {
  switch (engine.currentState) {
    case ENGINE_STATE_INIT:
      executeInit(timestamp_ms);
      break;
    case ENGINE_STATE_SAFE:
      executeSafe(timestamp_ms);
      break;
    case ENGINE_STATE_UNSAFE:
      executeUnsafe(timestamp_ms);
      break;
    case ENGINE_STATE_IGNITION:
      executeIgnition(timestamp_ms);
      break;
    case ENGINE_STATE_LAUNCH:
      executeLaunch(timestamp_ms);
      break;
    case ENGINE_STATE_ABORT:
      executeAbort(timestamp_ms);
      break;
    default:
      engine.errorStatus.bits.invalidState = 1;
      executeSafe(timestamp_ms);
      break;
  }
}

void executeInit(uint32_t timestamp_ms) {
  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    engine.valves[i].close((struct Valve*)&engine.valves[i], timestamp_ms);
  }
  engine.currentState = ENGINE_STATE_SAFE;

  engine.telecommunication->config((struct Telecommunication*) engine.telecommunication);
}

void executeSafe(uint32_t timestamp_ms) {
  activateStorageFlag = 0;
}

void executeUnsafe(uint32_t timestamp_ms) {

}

void executeIgnition(uint32_t timestamp_ms) {

}

void executeLaunch(uint32_t timestamp_ms) {
  if (timestamp_ms > statusPacket.fields.launchTimestamp_ms + (uint32_t)1800000) {
    engine.currentState = ENGINE_STATE_ABORT;
    engine.valves[ENGINE_NOS_VALVE_INDEX].close((struct Valve*)&engine.valves[ENGINE_NOS_VALVE_INDEX], timestamp_ms);
    engine.valves[ENGINE_IPA_VALVE_INDEX].close((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], timestamp_ms);
    activateStorageFlag = 0;
  }
}

void executeAbort(uint32_t timestamp_ms) {
  activateStorageFlag = 0;
}

void executeAbortCommand(uint32_t timestamp_ms) {
  engine.valves[ENGINE_NOS_VALVE_INDEX].close((struct Valve*)&engine.valves[ENGINE_NOS_VALVE_INDEX], timestamp_ms);
  engine.valves[ENGINE_IPA_VALVE_INDEX].close((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], timestamp_ms);
  engine.currentState = ENGINE_STATE_ABORT;
}

void executeIgnitionCommand(uint32_t timestamp_ms) {
  engine.igniter->ignite((struct Igniter*)engine.igniter, timestamp_ms);
  statusPacket.fields.igniteTimestamp_ms = timestamp_ms;
}

void executeLaunchCommand(uint32_t timestamp_ms) {
  engine.valves[ENGINE_NOS_VALVE_INDEX].open((struct Valve*)&engine.valves[ENGINE_NOS_VALVE_INDEX], timestamp_ms);
  engine.valves[ENGINE_IPA_VALVE_INDEX].open((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], timestamp_ms);
  engine.valves[ENGINE_NOS_VALVE_INDEX].heatpad->setDutyCycle_pct((struct Heater*)engine.valves[ENGINE_NOS_VALVE_INDEX].heatpad, 0);
  engine.valves[ENGINE_IPA_VALVE_INDEX].heatpad->setDutyCycle_pct((struct Heater*)engine.valves[ENGINE_IPA_VALVE_INDEX].heatpad, 0);
  statusPacket.fields.launchTimestamp_ms = timestamp_ms;
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
    engine.adc->init((struct ADC12*)engine.adc, engine.sdCardBuffer->values, sizeof(EngineSDCardBuffer), ENGINE_ADC_CHANNEL_AMOUNT);
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
  HAL_UART_Receive_DMA(engine.uart->externalHandle, uartRxBuffer, sizeof(uartRxBuffer));
}

void initValves() {
  engine.valves[ENGINE_IPA_VALVE_INDEX].pwm = &engine.pwms[ENGINE_IPA_VALVE_PWM_INDEX];
  engine.valves[ENGINE_IPA_VALVE_INDEX].gpio[VALVE_GPIO_OPENED_INDEX] = &engine.gpios[ENGINE_IPA_VALVE_OPENED_GPIO_INDEX];
  engine.valves[ENGINE_IPA_VALVE_INDEX].gpio[VALVE_GPIO_CLOSED_INDEX] = &engine.gpios[ENGINE_IPA_VALVE_CLOSED_GPIO_INDEX];
  engine.valves[ENGINE_IPA_VALVE_INDEX].heatpad = &engine.heaters[ENGINE_IPA_HEATPAD_INDEX];
  engine.valves[ENGINE_IPA_VALVE_INDEX].openDutyCycle_pct = IPA_VALVE_OPEN_DUTY_CYCLE_PCT;
  engine.valves[ENGINE_IPA_VALVE_INDEX].closeDutyCycle_pct = IPA_VALVE_CLOSED_DUTY_CYCLE_PCT;

  engine.valves[ENGINE_NOS_VALVE_INDEX].pwm = &engine.pwms[ENGINE_NOS_VALVE_PWM_INDEX];
  engine.valves[ENGINE_NOS_VALVE_INDEX].gpio[VALVE_GPIO_OPENED_INDEX] = &engine.gpios[ENGINE_NOS_VALVE_OPENED_GPIO_INDEX];
  engine.valves[ENGINE_NOS_VALVE_INDEX].gpio[VALVE_GPIO_CLOSED_INDEX] = &engine.gpios[ENGINE_NOS_VALVE_CLOSED_GPIO_INDEX];
  engine.valves[ENGINE_NOS_VALVE_INDEX].heatpad = &engine.heaters[ENGINE_NOS_HEATPAD_INDEX];
  engine.valves[ENGINE_NOS_VALVE_INDEX].openDutyCycle_pct = NOS_VALVE_OPEN_DUTY_CYCLE_PCT;
  engine.valves[ENGINE_NOS_VALVE_INDEX].closeDutyCycle_pct = NOS_VALVE_CLOSED_DUTY_CYCLE_PCT;

  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    if (engine.valves[i].init == FUNCTION_NULL_POINTER) {
      engine.valves[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.valves[i].init((struct Valve*)&engine.valves[i]);
  }
}

void initTemperatureSensors() {
  engine.temperatureSensors[ENGINE_IPA_MAIN_VALVE_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_IPA_MAIN_VALVE_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_NOS_MAIN_VALVE_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_NOS_MAIN_VALVE_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_THROAT_1_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_THROAT_1_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_THROAT_2_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_THROAT_2_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_THROAT_3_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_THROAT_3_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_NOZZLE_1_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_NOZZLE_1_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_NOZZLE_2_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_NOZZLE_2_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_THERMISTANCE_8_INDEX].adcChannel = &engine.adc->channels[ENGINE_THERMISTANCE_8_ADC_CHANNEL_INDEX];

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

void initIgniter() {
  engine.igniter->gpio = &engine.gpios[ENGINE_IGNITER_1_GPIO_INDEX];
  if (engine.igniter->init == FUNCTION_NULL_POINTER) {
    engine.igniter->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  engine.igniter->igniteDuration_ms = 3500;
  engine.igniter->init((struct Igniter*)engine.igniter);
}

void initHeaters() {
  engine.heaters[ENGINE_NOS_HEATPAD_INDEX].gpio = &engine.gpios[ENGINE_NOS_HEATPAD_GPIO_INDEX];
  engine.heaters[ENGINE_IPA_HEATPAD_INDEX].gpio = &engine.gpios[ENGINE_IPA_HEATPAD_GPIO_INDEX];

  for (uint8_t i = 0; i < ENGINE_HEATPAD_AMOUNT; i++) {
    if (engine.heaters[i].init == FUNCTION_NULL_POINTER) {
      engine.heaters[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.heaters[i].init((struct Heater*)&engine.heaters[i]);
    engine.heaters[i].period_s = 10;
    engine.heaters[i].lastSwitchTimestamp_ms = 0;
    engine.heaters[i].setDutyCycle_pct((struct Heater*)&engine.heaters[i], 0);
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

// REFACTOR THIS IS SUCKS DICK (the footer union and this code)
void handleDataStorage(uint32_t timestamp_ms) {
  if (engine.adc->status.bits.dmaFull) {
    if (engine.isStoringData) {
      engine.sdCardBuffer->sdData[1].footer.timestamp_ms = adcFullTimestamp_ms;
      engine.sdCardBuffer->sdData[1].footer.status = engine.status.value;
      engine.sdCardBuffer->sdData[1].footer.errorStatus = engine.errorStatus.value;
      engine.sdCardBuffer->sdData[1].footer.valveStatus[0] = engine.valves[0].status.value;
      engine.sdCardBuffer->sdData[1].footer.valveStatus[1] = engine.valves[1].status.value;
      engine.sdCardBuffer->sdData[1].footer.valveErrorStatus[0] = engine.valves[0].errorStatus.value;
      engine.sdCardBuffer->sdData[1].footer.valveErrorStatus[1] = engine.valves[1].errorStatus.value;
      engine.sdCardBuffer->sdData[1].footer.currentCommand[0] = currentCommand.fields.header.value;
      engine.sdCardBuffer->sdData[1].footer.currentCommand[0] = 0;
      engine.sdCardBuffer->sdData[1].footer.currentCommand[0] = 0;

      for (uint8_t i = 0; i < 32;i++) {
        engine.sdCardBuffer->sdData[1].footer.padding[i] = 0;
      }

      for (uint8_t i = 0; i < 16;i++) {
        engine.sdCardBuffer->sdData[1].footer.signature[i] = 0xFFFFFFFF;
      }

      engine.sdCardBuffer->sdData[1].footer.crc = HAL_CRC_Calculate(engine.hcrc, (uint32_t*)&engine.sdCardBuffer->sdData[1], (sizeof(EngineSDFormattedData) / sizeof(uint32_t)) -  sizeof(uint8_t));
      engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].store((struct Storage*)&engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], STORAGE_DATA_FAST_DESTINATION, (uint8_t*)(engine.sdCardBuffer->hex + (sizeof(EngineSDCardBuffer) / 2)), (sizeof(EngineSDCardBuffer) / 2));
    }
    
    engine.adc->status.bits.dmaFull = 0;
  }
  
  if (engine.adc->status.bits.dmaHalfFull) {
    if (engine.isStoringData) {
      engine.sdCardBuffer->sdData[0].footer.timestamp_ms = adcFullTimestamp_ms;
      engine.sdCardBuffer->sdData[0].footer.status = engine.status.value;
      engine.sdCardBuffer->sdData[0].footer.errorStatus = engine.errorStatus.value;
      engine.sdCardBuffer->sdData[0].footer.valveStatus[0] = engine.valves[0].status.value;
      engine.sdCardBuffer->sdData[0].footer.valveStatus[1] = engine.valves[1].status.value;
      engine.sdCardBuffer->sdData[0].footer.valveErrorStatus[0] = engine.valves[0].errorStatus.value;
      engine.sdCardBuffer->sdData[0].footer.valveErrorStatus[1] = engine.valves[1].errorStatus.value;
      // Those are the BoardCommand
      engine.sdCardBuffer->sdData[0].footer.currentCommand[0] = currentCommand.fields.header.value;
      engine.sdCardBuffer->sdData[0].footer.currentCommand[0] = 0;
      engine.sdCardBuffer->sdData[0].footer.currentCommand[0] = 0;

      for (uint8_t i = 0; i < 32;i++) {
        engine.sdCardBuffer->sdData[0].footer.padding[i] = 0;
      }

      for (uint8_t i = 0; i < 16;i++) {
        engine.sdCardBuffer->sdData[0].footer.signature[i] = 0xFFFFFFFF;
      }

      engine.sdCardBuffer->sdData[0].footer.crc = HAL_CRC_Calculate(engine.hcrc, (uint32_t*)&engine.sdCardBuffer->sdData[0], (sizeof(EngineSDFormattedData) / sizeof(uint32_t)) -  sizeof(uint8_t));
      engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].store((struct Storage*)&engine.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], STORAGE_DATA_FAST_DESTINATION, (uint8_t*)(engine.sdCardBuffer->hex), (sizeof(EngineSDCardBuffer) / 2));
    }
    
    engine.adc->status.bits.dmaHalfFull = 0;
  }
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
}

void handleCurrentCommand() {
  switch (currentCommand.fields.header.bits.commandCode) {
    case BOARD_COMMAND_CODE_ABORT:
      executeAbortCommand(HAL_GetTick());
      break;
    case BOARD_COMMAND_CODE_ACK:
      handleCommandAcknowledge();
      break;
    default:
      break;
  }

  switch (engine.currentState) {
    case ENGINE_STATE_INIT:
      break;
    case ENGINE_STATE_SAFE:
      handleCurrentCommandSafe();
      break;
    case ENGINE_STATE_UNSAFE:
      handleCurrentCommandUnsafe();
      break;
    case ENGINE_STATE_IGNITION:
      handleCurrentCommandIgnite();
      break;
    case ENGINE_STATE_LAUNCH:
      handleCurrentCommandLaunch();
      break;
    case ENGINE_STATE_ABORT:
      handleCurrentCommandAbort();
      break;
    default:
      break;
  }
}

void handleCurrentCommandAbort() {
  if (currentCommand.fields.header.bits.commandCode == BOARD_COMMAND_CODE_RESET) {
    engine.currentState = ENGINE_STATE_SAFE;
  }
}

void handleCommandAcknowledge() {
   CommandResponse commandResponse = {
    .fields = {
      .header = {
        .bits = {
          .type = COMMAND_RESPONSE_TYPE_CODE,
          .boardId = ENGINE_BOARD_ID,
          .commandIndex = 0,
          .response = RESPONSE_CODE_OK
        }
      },
      .crc = 0
    }
  };

  commandResponse.fields.crc = HAL_CRC_Calculate((CRC_HandleTypeDef*)engine.hcrc, commandResponse.data32, (sizeof(CommandResponse) / sizeof(uint32_t)) - sizeof(uint8_t));

  HAL_UART_Transmit_DMA(engine.uart->externalHandle, commandResponse.data, sizeof(CommandResponse));
}

void handleCurrentCommandSafe() {
  switch (currentCommand.fields.header.bits.commandCode) {
    case BOARD_COMMAND_CODE_UNSAFE:
      engine.currentState = ENGINE_STATE_UNSAFE;
      activateStorageFlag = 1;
      break;
    case ENGINE_COMMAND_CODE_SET_IPA_VALVE_HEATER_POWER_PCT:
      if (currentCommand.fields.value <= 100) {
        engine.valves[ENGINE_IPA_VALVE_INDEX].heatpad->setDutyCycle_pct((struct Heater*)engine.valves[ENGINE_IPA_VALVE_INDEX].heatpad, currentCommand.fields.value);
      }
      break;
    case ENGINE_COMMAND_CODE_SET_NOS_VALVE_HEATER_POWER_PCT:
      if (currentCommand.fields.value <= 100) {
        engine.valves[ENGINE_NOS_VALVE_INDEX].heatpad->setDutyCycle_pct((struct Heater*)engine.valves[ENGINE_NOS_VALVE_INDEX].heatpad, currentCommand.fields.value);
      }
      break;
    default:
      break;
  }
}

void handleCurrentCommandUnsafe() {
  switch (currentCommand.fields.header.bits.commandCode) {
    case BOARD_COMMAND_CODE_SAFE:
      engine.valves[ENGINE_NOS_VALVE_INDEX].close((struct Valve*)&engine.valves[ENGINE_NOS_VALVE_INDEX], HAL_GetTick());
      engine.valves[ENGINE_IPA_VALVE_INDEX].close((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], HAL_GetTick());
      engine.currentState = ENGINE_STATE_SAFE;
      break;
    case ENGINE_COMMAND_CODE_SET_IPA_VALVE_HEATER_POWER_PCT:
      if (currentCommand.fields.value <= 100) {
        engine.valves[ENGINE_IPA_VALVE_INDEX].heatpad->setDutyCycle_pct((struct Heater*)engine.valves[ENGINE_IPA_VALVE_INDEX].heatpad, currentCommand.fields.value);
      }
      break;
    case ENGINE_COMMAND_CODE_SET_NOS_VALVE_HEATER_POWER_PCT:
      if (currentCommand.fields.value <= 100) {
        engine.valves[ENGINE_NOS_VALVE_INDEX].heatpad->setDutyCycle_pct((struct Heater*)engine.valves[ENGINE_NOS_VALVE_INDEX].heatpad, currentCommand.fields.value);
      }
      break;
    case ENGINE_COMMAND_CODE_FIRE_IGNITER:
      engine.currentState = ENGINE_STATE_IGNITION;
      executeIgnitionCommand(HAL_GetTick());
      break;
    default:
      break;
  }
}

void handleCurrentCommandIgnite() {
  switch (currentCommand.fields.header.bits.commandCode) {
    case BOARD_COMMAND_CODE_SAFE:
      engine.valves[ENGINE_NOS_VALVE_INDEX].close((struct Valve*)&engine.valves[ENGINE_NOS_VALVE_INDEX], HAL_GetTick());
      engine.valves[ENGINE_IPA_VALVE_INDEX].close((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], HAL_GetTick());
      engine.currentState = ENGINE_STATE_SAFE;
      break;
    case ENGINE_COMMAND_CODE_OPEN_VALVE:
      engine.currentState = ENGINE_STATE_LAUNCH;
      executeLaunchCommand(HAL_GetTick());
      break;
    default:
      break;
  }
}

void handleCurrentCommandLaunch() {
  switch (currentCommand.fields.header.bits.commandCode) {
    case BOARD_COMMAND_CODE_SAFE:
      engine.valves[ENGINE_NOS_VALVE_INDEX].close((struct Valve*)&engine.valves[ENGINE_NOS_VALVE_INDEX], HAL_GetTick());
      engine.valves[ENGINE_IPA_VALVE_INDEX].close((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], HAL_GetTick());
      engine.currentState = ENGINE_STATE_SAFE;
      break;
    default:
      break;
  }
}

void sendTelemetryPacket(uint32_t timestamp_ms) {
  telemetryPacket.fields.timestamp_ms = timestamp_ms;
  for (uint8_t i = 0; i < ENGINE_ADC_CHANNEL_AMOUNT; i++) {
    filterTelemetryValues(i);
    telemetryPacket.fields.adcValues[i] = filteredTelemetryValues[i];
  }
  telemetryPacket.fields.crc = HAL_CRC_Calculate(engine.hcrc, (uint32_t*)telemetryPacket.data, (sizeof(EngineTelemetryPacket) / sizeof(uint32_t)) - sizeof(uint8_t));
  engine.telecommunication->sendData((struct Telecommunication*)engine.telecommunication, telemetryPacket.data, sizeof(EngineTelemetryPacket));
}

void sendStatusPacket(uint32_t timestamp_ms) {
  statusPacket.fields.timestamp_ms = timestamp_ms;
  statusPacket.fields.errorStatus = engine.errorStatus;
  statusPacket.fields.status = engine.status;
  statusPacket.fields.timeSinceLastCommand_ms = timeSinceLastCommand_ms;
  statusPacket.fields.valveStatus[ENGINE_NOS_VALVE_INDEX] = engine.valves[ENGINE_NOS_VALVE_INDEX].status;
  statusPacket.fields.valveStatus[ENGINE_IPA_VALVE_INDEX] = engine.valves[ENGINE_IPA_VALVE_INDEX].status;
  statusPacket.fields.crc = HAL_CRC_Calculate(engine.hcrc, (uint32_t*)statusPacket.data, (sizeof(EngineStatusPacket) / sizeof(uint32_t)) - sizeof(uint8_t));

  engine.telecommunication->sendData((struct Telecommunication*)engine.telecommunication, statusPacket.data, sizeof(EngineStatusPacket));
}

void getReceivedCommand() {
  for (uint16_t i = uartRxHalfReady ? 0 : sizeof(uartRxBuffer) / 2 - sizeof(currentCommand) - 1;
    i < uartRxHalfReady ? (sizeof(uartRxBuffer) / 2) - sizeof(currentCommand) - 1 : sizeof(uartRxBuffer) - sizeof(currentCommand) - 1; i++) {
    currentCommand.data[0] = uartRxBuffer[i];
    currentCommand.data[1] = uartRxBuffer[i + 1];
    currentCommand.data[2] = uartRxBuffer[i + 2];
    currentCommand.data[3] = uartRxBuffer[i + 3];
    if (currentCommand.fields.header.bits.type == BOARD_COMMAND_BROADCAST_TYPE_CODE || 
        (currentCommand.fields.header.bits.type == BOARD_COMMAND_UNICAST_TYPE_CODE &&
        currentCommand.fields.header.bits.boardId == ENGINE_BOARD_ID)) {
      for (uint16_t j = 4; j < sizeof(BoardCommand); j++) {
        currentCommand.data[j] = uartRxBuffer[i + j];
        if (checkCommandCrc()) {
          i += sizeof(BoardCommand) - 1;
          statusPacket.fields.lastReceivedCommandCode = currentCommand.fields.header.bits.commandCode;
          lastCommandTimestamp_ms = HAL_GetTick();
          timeSinceLastCommand_ms = 0;
          communicationRestartTimer_ms = 0;
          handleCurrentCommand();
          break;
        }
      }
    }
  }
  if (uartRxCpltReady) {
    uartRxCpltReady = 0;
  }
  else {
    uartRxHalfReady = 0;
  }
}

void filterTelemetryValues(uint8_t index) {
  uint32_t filteredValue = 0;
  for (uint16_t i = 0; i < 64; i++) {
    filteredValue += engine.sdCardBuffer->values[index + i*FILTER_TELEMETRY_OFFSET];
  }
  filteredTelemetryValues[index] = filteredValue >> 6;
}

uint8_t checkCommandCrc() {
  if (currentCommand.fields.crc != HAL_CRC_Calculate(engine.hcrc, (uint32_t*)currentCommand.data, (sizeof(BoardCommand) / sizeof(uint32_t)) - sizeof(uint8_t))) {
    return 0;
  }
  return 1;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  if (engine.adc->status.bits.dmaHalfFull == 0) {
    engine.adc->status.bits.dmaHalfFull = 1;
    adcHalfFullTimestamp_ms = HAL_GetTick();
    if (activateStorageFlag) {
      engine.isStoringData = 1;
    }
    else {
      engine.isStoringData = 0;
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (engine.adc->status.bits.dmaFull == 0) {
    engine.adc->status.bits.dmaFull = 1;
    adcFullTimestamp_ms = HAL_GetTick();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    uartRxCpltReady = 1;
    getReceivedCommand();
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
  if(huart->Instance == USART1) {
    uartRxHalfReady = 1;
    getReceivedCommand();
  }
}