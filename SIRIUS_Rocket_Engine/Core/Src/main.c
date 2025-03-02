/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Engine.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
GPIO gpios[ENGINE_GPIO_AMOUNT]        = {0};
ADC12 adc                             = {0};
PWM pwms[ENGINE_PWM_AMOUNT]           = {0};
UART uarts[ENGINE_UART_AMOUNT]        = {0};

Valve valves[ENGINE_VALVE_AMOUNT]                                = {0};
PressureSensor pressuresensors[ENGINE_PRESSURE_SENSOR_AMOUNT]    = {0};
TemperatureSensor temperatureSensors[ENGINE_THERMISTANCE_AMOUNT] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

static void setupGPIOs();
static void setupPWMs();
static void setupADC();
static void setupUART();

static void setupValves();
static void setupIgniter();
static void setupTemperatureSensors();
static void setupPressureSensors();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  setupADC();
  setupPWMs();
  setupGPIOs();
  setupUART();

  setupValves();
  setupIgniter();
  setupPressureSensors();
  setupTemperatureSensors();

  Engine_init(pwms, &adc, valves, temperatureSensors);

  //if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  //{
    /* Starting Error */
    //Error_Handler();
  //}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Engine_execute();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 16;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 16;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// These should only link HAL to instance and set base function pointers

void setupGPIOs() {
  gpios[ENGINE_IGNITER_GPIO_INDEX].errorStatus.bits.notInitialized = 1;
  gpios[ENGINE_IGNITER_GPIO_INDEX].init = (GPIO_init)GPIOHAL_init;
  gpios[ENGINE_IGNITER_GPIO_INDEX].read = (GPIO_read)GPIOHAL_read;
  gpios[ENGINE_IGNITER_GPIO_INDEX].write = (GPIO_write)GPIOHAL_write;
}

void setupPWMs() {
  pwms[ENGINE_IPA_VALVE_PWM_INDEX].errorStatus.bits.notInitialized = 1;
  pwms[ENGINE_IPA_VALVE_PWM_INDEX].init = (PWM_init)PWMHAL_init;
  pwms[ENGINE_IPA_VALVE_PWM_INDEX].setDutyCycle = (PWM_setDutyCycle)PWMHAL_setDutyCycle;
  pwms[ENGINE_IPA_VALVE_PWM_INDEX].externalHandle = &htim4;
  pwms[ENGINE_IPA_VALVE_PWM_INDEX].timer = TIM4;
  pwms[ENGINE_IPA_VALVE_PWM_INDEX].channel = TIM_CHANNEL_3;

  pwms[ENGINE_NOS_VALVE_PWM_INDEX].errorStatus.bits.notInitialized = 1;
  pwms[ENGINE_NOS_VALVE_PWM_INDEX].init = (PWM_init)PWMHAL_init;
  pwms[ENGINE_NOS_VALVE_PWM_INDEX].setDutyCycle = (PWM_setDutyCycle)PWMHAL_setDutyCycle;
  pwms[ENGINE_NOS_VALVE_PWM_INDEX].externalHandle = &htim4;
  pwms[ENGINE_NOS_VALVE_PWM_INDEX].timer = TIM4;
  pwms[ENGINE_NOS_VALVE_PWM_INDEX].channel = TIM_CHANNEL_2;
}

void setupADC() {
  adc.errorStatus.bits.notInitialized = 1;
  adc.init = (ADC12_init)ADC12HAL_init;
  adc.tick = (ADC12_tick)ADC12HAL_tick;
  adc.externalHandle = &hadc1;

  adc.channels[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_ADC_CHANNEL_INDEX].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_ADC_CHANNEL_INDEX].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_ADC_CHANNEL_INDEX].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;

  adc.channels[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_ADC_CHANNEL_INDEX].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_ADC_CHANNEL_INDEX].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_ADC_CHANNEL_INDEX].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_ADC_CHANNEL_INDEX].rank = 2;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_ADC_CHANNEL_INDEX].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_ADC_CHANNEL_INDEX].channelNumber = 1;

  adc.channels[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_ADC_CHANNEL_INDEX].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_ADC_CHANNEL_INDEX].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_ADC_CHANNEL_INDEX].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_ADC_CHANNEL_INDEX].rank = 3;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_ADC_CHANNEL_INDEX].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_ADC_CHANNEL_INDEX].channelNumber = 2;

  adc.channels[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_ADC_CHANNEL_INDEX].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_ADC_CHANNEL_INDEX].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_ADC_CHANNEL_INDEX].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_ADC_CHANNEL_INDEX].rank = 4;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_ADC_CHANNEL_INDEX].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_ADC_CHANNEL_INDEX].channelNumber = 3;

  adc.channels[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_ADC_CHANNEL_INDEX].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_ADC_CHANNEL_INDEX].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_ADC_CHANNEL_INDEX].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_ADC_CHANNEL_INDEX].rank = 5;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_ADC_CHANNEL_INDEX].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_ADC_CHANNEL_INDEX].channelNumber = 4;

  adc.channels[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_ADC_CHANNEL_INDEX].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_ADC_CHANNEL_INDEX].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_ADC_CHANNEL_INDEX].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_ADC_CHANNEL_INDEX].rank = 6;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_ADC_CHANNEL_INDEX].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_ADC_CHANNEL_INDEX].channelNumber = 5;

  adc.channels[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_ADC_CHANNEL_INDEX].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_ADC_CHANNEL_INDEX].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_ADC_CHANNEL_INDEX].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_ADC_CHANNEL_INDEX].rank = 7;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_ADC_CHANNEL_INDEX].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_ADC_CHANNEL_INDEX].channelNumber = 6;

  adc.channels[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_ADC_CHANNEL_INDEX].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_ADC_CHANNEL_INDEX].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_ADC_CHANNEL_INDEX].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_ADC_CHANNEL_INDEX].rank = 8;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_ADC_CHANNEL_INDEX].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_ADC_CHANNEL_INDEX].channelNumber = 7;

  adc.channels[ENGINE_NOS_TANK_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_NOS_TANK_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_NOS_TANK_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_NOS_TANK_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].rank = 9;
  adc.channels[ENGINE_NOS_TANK_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_NOS_TANK_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].channelNumber = 8;

  adc.channels[ENGINE_NOS_TANK_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_NOS_TANK_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_NOS_TANK_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_NOS_TANK_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].rank = 10;
  adc.channels[ENGINE_NOS_TANK_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_NOS_TANK_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].channelNumber = 9;

  adc.channels[ENGINE_NOS_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_NOS_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_NOS_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_NOS_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].rank = 11;
  adc.channels[ENGINE_NOS_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_NOS_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].channelNumber = 10;

  adc.channels[ENGINE_NOS_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_NOS_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_NOS_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_NOS_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].rank = 12;
  adc.channels[ENGINE_NOS_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_NOS_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].channelNumber = 11;

  adc.channels[ENGINE_IPA_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_IPA_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_IPA_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_IPA_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].rank = 13;
  adc.channels[ENGINE_IPA_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_IPA_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].channelNumber = 12;

  adc.channels[ENGINE_IPA_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_IPA_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_IPA_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_IPA_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].rank = 14;
  adc.channels[ENGINE_IPA_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_IPA_MANIFOLD_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].channelNumber = 13;

  adc.channels[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].rank = 15;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_1].channelNumber = 14;

  adc.channels[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].errorStatus.bits.notInitialized = 1;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].init = (ADC12Channel_init)ADC12ChannelHAL_init;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].tick = (ADC12Channel_tick)ADC12ChannelHAL_tick;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].rank = 16;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].sampleTime_adcClockCyles = ADC_SAMPLETIME_28CYCLES;
  adc.channels[ENGINE_COMBUSTION_CHAMBER_PRESSURE_SENSOR_ADC_CHANNEL_INDEX_2].channelNumber = 15;
}

void setupUART() {
  uarts[ENGINE_TELECOMMUNICATION_UART_INDEX].errorStatus.bits.notInitialized = 1;
  uarts[ENGINE_TELECOMMUNICATION_UART_INDEX].init = (UART_init)UARTHAL_init;
  uarts[ENGINE_TELECOMMUNICATION_UART_INDEX].sendData = (UART_sendData)UARTHAL_sendData;
  uarts[ENGINE_TELECOMMUNICATION_UART_INDEX].receiveData = (UART_receiveData)UARTHAL_receiveData;
  uarts[ENGINE_TELECOMMUNICATION_UART_INDEX].externalHandle = &huart1;
}

void setupValves() {
  valves[ENGINE_IPA_VALVE_INDEX].errorStatus.bits.notInitialized = 1;
  valves[ENGINE_IPA_VALVE_INDEX].init = (Valve_init)HBL388_init;
  valves[ENGINE_IPA_VALVE_INDEX].setDutyCycle = (Valve_setDutyCycle)HBL388_setDutyCycle;
  valves[ENGINE_IPA_VALVE_INDEX].gatherData = (Valve_gatherData)HBL388_gatherData;
  valves[ENGINE_IPA_VALVE_INDEX].tick = (Valve_tick)HBL388_tick;

  valves[ENGINE_NOS_VALVE_INDEX].errorStatus.bits.notInitialized = 1;
  valves[ENGINE_NOS_VALVE_INDEX].init = (Valve_init)HBL388_init;
  valves[ENGINE_NOS_VALVE_INDEX].setDutyCycle = (Valve_setDutyCycle)HBL388_setDutyCycle;
  valves[ENGINE_NOS_VALVE_INDEX].gatherData = (Valve_gatherData)HBL388_gatherData;
  valves[ENGINE_NOS_VALVE_INDEX].tick = (Valve_tick)HBL388_tick;
}

void setupIgniter() {

}

void setupTemperatureSensors() {
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_INDEX].errorStatus.bits.notInitialized = 1;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_INDEX].init = (TemperatureSensor_init)NTC3950_init;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_INDEX].readData = (TemperatureSensor_readData)NTC3950_readData;

  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_INDEX].errorStatus.bits.notInitialized = 1;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_INDEX].init = (TemperatureSensor_init)NTC3950_init;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_INDEX].readData = (TemperatureSensor_readData)NTC3950_readData;

  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_INDEX].errorStatus.bits.notInitialized = 1;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_INDEX].init = (TemperatureSensor_init)NTC3950_init;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_INDEX].readData = (TemperatureSensor_readData)NTC3950_readData;

  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_INDEX].errorStatus.bits.notInitialized = 1;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_INDEX].init = (TemperatureSensor_init)NTC3950_init;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_INDEX].readData = (TemperatureSensor_readData)NTC3950_readData;

  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_INDEX].errorStatus.bits.notInitialized = 1;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_INDEX].init = (TemperatureSensor_init)NTC3950_init;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_INDEX].readData = (TemperatureSensor_readData)NTC3950_readData;

  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_INDEX].errorStatus.bits.notInitialized = 1;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_INDEX].init = (TemperatureSensor_init)NTC3950_init;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_INDEX].readData = (TemperatureSensor_readData)NTC3950_readData;

  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_INDEX].errorStatus.bits.notInitialized = 1;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_INDEX].init = (TemperatureSensor_init)NTC3950_init;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_INDEX].readData = (TemperatureSensor_readData)NTC3950_readData;

  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_INDEX].errorStatus.bits.notInitialized = 1;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_INDEX].init = (TemperatureSensor_init)NTC3950_init;
  temperatureSensors[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_INDEX].readData = (TemperatureSensor_readData)NTC3950_readData;
}

void setupPressureSensors() {

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
