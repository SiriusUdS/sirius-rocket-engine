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

#include "../sirius-embedded-common/Inc/Device/Telecommunication/Telecommunication.h"
#include "../sirius-embedded-common/sirius-headers-common/Telecommunication/TelemetryPacket.h"
#include "../sirius-embedded-common/sirius-headers-common/Telecommunication/BoardCommand.h"
#include "../sirius-embedded-common/Inc/Device/Telecommunication/XBEE.h"

#include "stm32f4xx_hal.h"

#define FUNCTION_NULL_POINTER 0

#define DATA_GATHERING_MODE_SLOW (uint8_t)0x00
#define DATA_GATHERING_MODE_FAST (uint8_t)0x01

#define TIME_BETWEEN_TELEMETRY_PACKETS_MS        (uint8_t)91
#define TELEMETRY_PACKETS_BETWEEN_STATUS_PACKETS (uint8_t)5

#define STORAGE_DELAY_BETWEEN_SLOW_SAVES_MS 250

#define FILTER_TELEMETRY_OFFSET (((sizeof(ADCBuffer) / 2)/sizeof(uint16_t)) / 64)
#define TELEMETRY_TIMESTAMP_BUFFER_SIZE_2BYTES 0x200
#define TELEMETRY_BUFFER_SIZE_2BYTES 0x200

#define USB_ENABLED

typedef struct {
  EngineErrorStatus errorStatus;
  EngineStatus      status;

  uint8_t currentState;

  ADC12* adc;
  PWM*   pwms;
  GPIO*  gpios;
  UART*  uart;

  volatile USB* usb;

  Valve* valves;

  TemperatureSensor* temperatureSensors;

  PressureSensor* pressureSensors;

  Telecommunication* telecommunication;
  uint32_t           telecommunicationTimestampTarget_ms;
  uint8_t            telecommunicationTelemetryPacketCount;

  uint8_t dataGatheringMode;
  uint32_t storageTimestampTarget_ms;

  Storage* storageDevices;
  uint32_t sdCardBufferPosition;
  uint32_t sdCardTimestampBufferPosition;
  uint8_t  sdCardTimestampsBufferFull;

  volatile ADCBuffer* dmaAdcBuffer;
  uint8_t isTelemetryBufferFull;
  uint8_t isTelemetryTimestampsBufferFull;
  volatile ADCTimestampsBuffer* dmaAdcTimestampsBuffer;
}
Engine;

extern void Engine_init(PWM* pwms, ADC12* adc, GPIO* gpios, UART* uart, volatile USB* usb, Valve* valves, TemperatureSensor* temperatureSensors, Telecommunication* telecom, Storage* storageDevices, ADCBuffer* dmaAdcBuffer, ADCTimestampsBuffer* dmaAdcTimestampsBuffer);

extern void Engine_tick(uint32_t timestamp_ms);

extern void Engine_execute(uint32_t timestamp_ms);