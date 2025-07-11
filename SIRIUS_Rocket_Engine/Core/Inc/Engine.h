#pragma once

#include "../sirius-embedded-common/Inc/Sensor/PressureSensor/ETM375.h"
#include "../sirius-embedded-common/sirius-headers-common/PressureSensor/PressureSensorData.h"

#include "../sirius-embedded-common/Inc/Sensor/TemperatureSensor/NTC3950.h"
#include "../sirius-embedded-common/sirius-headers-common/TemperatureSensor/TemperatureSensorPacket.h"

#include "../sirius-embedded-common/Inc/Device/Valve/HBL388.h"

#include "../sirius-embedded-common/Inc/Device/Igniter/EstesC6.h"

#include "../sirius-embedded-common/Inc/Device/Storage/SDCard.h"
#include "../sirius-embedded-common/Inc/Device/Storage/ExternalFlash.h"

#include "../sirius-embedded-common/Inc/LowLevelDriver/GPIO/GPIOHAL.h"
#include "../sirius-embedded-common/Inc/LowLevelDriver/PWM/PWMHAL.h"
#include "../sirius-embedded-common/Inc/LowLevelDriver/ADC/ADC12HAL.h"
#include "../sirius-embedded-common/Inc/LowLevelDriver/UART/UARTHAL.h"
#include "../sirius-embedded-common/Inc/LowLevelDriver/USB/USBHAL.h"

#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineStatus.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineErrorStatus.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineSensors.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineState.h"
#include "../sirius-embedded-common/sirius-headers-common/Engine/EngineSDBuffers.h"

#include "../sirius-embedded-common/Inc/Device/Telecommunication/Telecommunication.h"
#include "../sirius-embedded-common/sirius-headers-common/Telecommunication/TelemetryPacket.h"
#include "../sirius-embedded-common/sirius-headers-common/Telecommunication/BoardCommand.h"
#include "../sirius-embedded-common/Inc/Device/Telecommunication/XBEE.h"
#include "../sirius-embedded-common/sirius-headers-common/Telecommunication/PacketHeaderVariable.h"

#include "stm32f4xx_hal.h"

#define FUNCTION_NULL_POINTER 0

#define TIME_BETWEEN_TELEMETRY_PACKETS_MS        (uint8_t)91
#define TELEMETRY_PACKETS_BETWEEN_STATUS_PACKETS (uint8_t)5

#define FILTER_TELEMETRY_OFFSET (((sizeof(EngineSDCardBuffer) / 2)/sizeof(uint16_t)) / 64)

#define NOS_VALVE_OPEN_DUTY_CYCLE_PCT   (uint8_t)54
#define NOS_VALVE_CLOSED_DUTY_CYCLE_PCT (uint8_t)26

#define IPA_VALVE_OPEN_DUTY_CYCLE_PCT   (uint8_t)54
#define IPA_VALVE_CLOSED_DUTY_CYCLE_PCT (uint8_t)26

typedef struct {
  EngineErrorStatus errorStatus;
  EngineStatus      status;

  uint8_t currentState;

  ADC12* adc;
  PWM*   pwms;
  GPIO*  gpios;
  UART*  uart;

  Valve* valves;

  TemperatureSensor* temperatureSensors;

  PressureSensor* pressureSensors;

  Telecommunication* telecommunication;
  uint32_t           telecommunicationTimestampTarget_ms;
  uint8_t            telecommunicationTelemetryPacketCount;

  CRC_HandleTypeDef* hcrc;

  uint8_t isStoringData;
  uint32_t storageTimestampTarget_ms;

  Storage* storageDevices;
  uint32_t sdCardBufferPosition;
  uint32_t sdCardTimestampBufferPosition;
  uint8_t  sdCardTimestampsBufferFull;

  volatile EngineSDCardBuffer* sdCardBuffer;
  uint8_t isTelemetryBufferFull;
  uint8_t isTelemetryTimestampsBufferFull;
}
Engine;

extern void Engine_init(PWM* pwms, ADC12* adc, GPIO* gpios, UART* uart, Valve* valves, TemperatureSensor* temperatureSensors, Telecommunication* telecom, Storage* storageDevices, EngineSDCardBuffer* sdCardBuffer, CRC_HandleTypeDef* hcrc);

extern void Engine_tick(uint32_t timestamp_ms);

extern void Engine_execute(uint32_t timestamp_ms);