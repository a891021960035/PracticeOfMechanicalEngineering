/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_PULSE_LENGTH 20000 * 0.05 // Minimum pulse length : 1000µs
#define MAX_PULSE_LENGTH 20000 * 0.1  // Maximum pulse length : 2000µs
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// int i = 0;
int value0 = -1; // ADC value
int value1 = -1; // ADC value
int value2 = -1; // ADC value
int value3 = -1; // ADC value
int inloop = 0;
int statecode = 0b0000;
int tmp = 0;
int pulse_servo1 = 0;  // Servo 1 PWM pulse
int pulse_servo2 = 0;  // Servo 2 PWM pulse
int pulse_servo3 = 0;  // Servo 3 PWM pulse
int pulse_BLDC = 0;    // PWM pulse
float power_BLDC = 10; // BLDC throttle // 最優參數：13
float degree_servo = 90;
int mode = 0; // ESC mode
int timestart = 0;
int sec = 0;
int ms = 0;
int trigger = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
uint32_t Board_Get_ADCChannelValue(ADC_HandleTypeDef *hadc, uint32_t channel);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
static void writeServo(float angle);
static void setPower(float power);
static void brake();
static void unbrake();
static void TRS();  //微右轉
static void TRL();  //急右轉
static void TLS();  //微左轉
static void TLL();  //急左轉
static void TRSB(); //微右轉
static void TRLB(); //急右轉
static void TLSB(); //微左轉
static void TLLB(); //急左轉
static void DRS();  //微右飄
static void DRL();  //急右飄
static void DLS();  //微左飄
static void DLL();  //急左飄
static void BLDC_test();
static void waitBlack(int ch);
static void lineFollower(float operationTime, float power, int *tg);
static void lineFollowerBackward(float operationTime, float power, int *tg);
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
  pulse_servo1 = MIN_PULSE_LENGTH;
  pulse_servo2 = MIN_PULSE_LENGTH;
  pulse_servo3 = MIN_PULSE_LENGTH;
  pulse_BLDC = MIN_PULSE_LENGTH;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                 // Servo 1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);                 // Servo 2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);                 // Servo 3
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);                 //無刷馬達
  HAL_TIM_Base_Start_IT(&htim3);                            //開啟中斷
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_BLDC); //無刷馬達下限轉速
  writeServo(90);
  brake();
  // unbrake();
  HAL_Delay(3000);
  mode = 3;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 中斷測試
    // HAL_TIM_PeriodElapsedCallback(&htim3);

    // 類比輸入
    // HAL_ADC_Start(&hadc1);
    // HAL_ADC_PollForConversion(&hadc1, 1);
    // value0 = Board_Get_ADCChannelValue(&hadc1, 0);
    // value1 = Board_Get_ADCChannelValue(&hadc1, 1);
    // value2 = Board_Get_ADCChannelValue(&hadc1, 2);
    // value3 = Board_Get_ADCChannelValue(&hadc1, 3);
    // HAL_Delay(500);

    // 全域變數測試
    // statecode = 0;

    // ESC test
    switch (mode)
    {
    case 0:
      setPower(0);
      break;

    case 1:
      setPower(100);
      break;

    case 2:
      HAL_Delay(3000);
      setPower(power_BLDC);
      // BLDC_test();
      break;

    case 3:
      // 起步
      setPower(power_BLDC);
      HAL_Delay(2000);
      unbrake();

      // 第 1 段，轉彎至第一循跡線
      writeServo(71.3);
      while (trigger < 3)
      {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1);
        value2 = Board_Get_ADCChannelValue(&hadc1, 2);
        if (value2 > 1000) //感測到黑色
        {
          while (value2 > 1000) //變成白色之前狀態不變
          {
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 1);
            value2 = Board_Get_ADCChannelValue(&hadc1, 2);
          }
          trigger += 1;
        }
      }

      // 第 4 段，第一段循跡
      lineFollower(3, 6, &trigger);
      lineFollower(1, 17, &trigger);

      // 第 5 段，變換車道
      setPower(16); // 根據電量調整
      writeServo(60);
      // HAL_Delay(950); //轉回黑線_往前
      HAL_Delay(900); //轉回黑線_往後，900~830
      // 轉彎法
      setPower(13.5);
      writeServo(90);
      // 飄移法
      // pulse_servo2 = 500 + 2000 * 55 / 180;
      // pulse_servo3 = pulse_servo2;
      // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
      // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
      // writeServo(55);
      // HAL_Delay(1500);

      // 等到黑線停下
      waitBlack(2);
      // // 轉回黑線_往前
      // writeServo(138);
      // waitBlack(1);
      // 轉回黑線_往後
      setPower(0);
      brake();
      HAL_Delay(800);
      // pulse_servo2 = 500 + 2000 * 115 / 180;
      // pulse_servo3 = 500 + 2000 * 120 / 180; // 130?
      pulse_servo2 = 500 + 2000 * 115 / 180;
      pulse_servo3 = 500 + 2000 * 130 / 180; // 130?
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
      waitBlack(1);
      brake();
      HAL_Delay(500);
      unbrake();

      // 第 7 段，第二循跡線上坡
      trigger = 0;
      // 向前轉正模式
      // while (trigger < 2)
      // {
      //   lineFollower(100, 16.5, &trigger);
      // }
      // 向後轉正模式
      while (trigger < 2)
      {
        lineFollower(100, 18, &trigger);
      }
      writeServo(90);
      HAL_Delay(500);

      // 第 8 段，第一停止區
      setPower(0);
      brake();
      HAL_Delay(3500);
      unbrake();

      // 第 9 段，循機至第二停止區
      lineFollowerBackward(3, 6, &trigger);
      lineFollowerBackward(4, 0, &trigger);
      // brake();
      mode = 0;

      // case 4:
      //   pulse_servo2 = 500 + 2000 * 90 / 180;
      //   pulse_servo3 = pulse_servo2;
      //   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
      //   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
      //   writeServo(90);
    }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /**Common config
   */
  hadc1.Instance = ADC1;
  //  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE; //扫描（多通道）模式=ENABLE、单次（单通道）模式=DISABLE 因为同时只采集一个通道 所以设置为DISABLE
                                              //  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
                                              //  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;    //连续=ENABLE、单次=DISABLE
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  //  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  //  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  //  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  //  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  //  hadc1.Init.BoostMode = DISABLE;
  //  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /**Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  //  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  //  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  //  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 1000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  htim3.Instance = TIM3;
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint32_t Board_Get_ADCChannelValue(ADC_HandleTypeDef *hadc, uint32_t channel)
{
  ADC_ChannelConfTypeDef ADC_ChanConf;

  ADC_ChanConf.Channel = channel;
  ADC_ChanConf.Rank = ADC_REGULAR_RANK_1;
  ADC_ChanConf.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  //    ADC_ChanConf.SingleDiff = ADC_SINGLE_ENDED;
  //    ADC_ChanConf.OffsetNumber = ADC_OFFSET_NONE;
  //    ADC_ChanConf.Offset = 0;

  HAL_ADC_ConfigChannel(hadc, &ADC_ChanConf);

  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, 10); //轮询转换
  return (uint16_t)HAL_ADC_GetValue(hadc);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    ms++;
    if (ms == 1000)
    {
      ms = 0;
      sec++;
    }
  }
}

static void writeServo(float angle)
{
  pulse_servo1 = 500 + 2000 * angle / 180;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_servo1);
}

static void setPower(float power)
{
  pulse_BLDC = (MAX_PULSE_LENGTH - MIN_PULSE_LENGTH) * power / 100 + MIN_PULSE_LENGTH;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_BLDC);
}

static void brake()
{
  pulse_servo2 = 500 + 2000 * (180 - 60) / 180;
  pulse_servo3 = 500 + 2000 * 60 / 180;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
}

static void unbrake()
{
  pulse_servo2 = 500 + 2000 * 90 / 180;
  pulse_servo3 = 500 + 2000 * 90 / 180;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
}

static void TRS(void) //微右轉
{
  writeServo(112.5);
}

static void TRL(void) //急右轉
{
  writeServo(135);
}

static void TLS(void) //微左轉
{
  writeServo(67.5);
}

static void TLL(void) //急左轉
{
  writeServo(45);
}

static void TRSB(void) //後輪微右轉
{
  pulse_servo2 = 500 + 2000 * 105 / 180;
  pulse_servo3 = pulse_servo2;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
}

static void TRLB(void) //後輪急右轉
{
  pulse_servo2 = 500 + 2000 * 120 / 180;
  pulse_servo3 = pulse_servo2;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
}

static void TLSB(void) //後輪微左轉
{
  pulse_servo2 = 500 + 2000 * 75 / 180;
  pulse_servo3 = pulse_servo2;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
}

static void TLLB(void) //後輪急左轉
{
  pulse_servo2 = 500 + 2000 * 60 / 180;
  pulse_servo3 = pulse_servo2;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
}

static void DRS(void) //微右飄
{
  pulse_servo2 = 500 + 2000 * 100 / 180;
  pulse_servo3 = pulse_servo2;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
  writeServo(100);
}

static void DRL(void) //急右飄
{
  pulse_servo2 = 500 + 2000 * 110 / 180;
  pulse_servo3 = pulse_servo2;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
  writeServo(110);
}

static void DLS(void) //微左飄
{
  pulse_servo2 = 500 + 2000 * 80 / 180;
  pulse_servo3 = pulse_servo2;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
  writeServo(80);
}

static void DLL(void) //急左飄
{
  pulse_servo2 = 500 + 2000 * 70 / 180;
  pulse_servo3 = pulse_servo2;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
  writeServo(70);
}

static void BLDC_test(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MIN_PULSE_LENGTH);
  for (pulse_BLDC = MIN_PULSE_LENGTH; pulse_BLDC <= MAX_PULSE_LENGTH; pulse_BLDC += 5)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_BLDC);
    HAL_Delay(20);
  }
  mode = 0;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MIN_PULSE_LENGTH);
}

static void waitBlack(int ch)
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1);
  int value = Board_Get_ADCChannelValue(&hadc1, ch);
  while (value < 1000) //變成白色之前狀態不變
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    value = Board_Get_ADCChannelValue(&hadc1, ch);
  }
}

static void lineFollower(float operationTime, float power, int *tg)
{
  setPower(power);
  sec = 0;
  while (sec <= operationTime)
  {
    // HAL_TIM_PeriodElapsedCallback(&htim3);
    statecode = 0;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    value0 = Board_Get_ADCChannelValue(&hadc1, 0);
    value1 = Board_Get_ADCChannelValue(&hadc1, 1);
    value2 = Board_Get_ADCChannelValue(&hadc1, 2);
    value3 = Board_Get_ADCChannelValue(&hadc1, 3);

    if (value0 > 1000)
      statecode = statecode | 0b1000; // 8
    if (value1 > 1000)
      statecode = statecode | 0b0100; // 4
    if (value2 > 1000)
      statecode = statecode | 0b0010; // 2
    if (value3 > 1000)
      statecode = statecode | 0b0001; // 1

    tmp = (statecode & 0b1000) >> 3;
    tmp += (statecode & 0b0100) >> 2;
    tmp += (statecode & 0b0010) >> 1;
    tmp += (statecode & 0b0001);
    if (tmp > 2)
    {
      statecode = 0b1111;
    }

    switch (statecode)
    {
    case 0b1000:
      TLL(); //急左轉
      break;
    case 0b0100:
      TLS(); //微左轉
      break;
    case 0b0010:
      TRS(); //微右轉
      break;
    case 0b0001:
      TRL(); //急右轉
      break;
    case 0b0000:
      writeServo(90);
      break;
    case 0b1111:
      writeServo(90);
      while (tmp > 1) //變成白色之前狀態不變
      {
        inloop = 1;
        statecode = 0;
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1);
        value0 = Board_Get_ADCChannelValue(&hadc1, 0);
        value1 = Board_Get_ADCChannelValue(&hadc1, 1);
        value2 = Board_Get_ADCChannelValue(&hadc1, 2);
        value3 = Board_Get_ADCChannelValue(&hadc1, 3);

        if (value0 > 1000)
          statecode = statecode | 0b1000; // 8
        if (value1 > 1000)
          statecode = statecode | 0b0100; // 4
        if (value2 > 1000)
          statecode = statecode | 0b0010; // 2
        if (value3 > 1000)
          statecode = statecode | 0b0001; // 1

        tmp = (statecode & 0b1000) >> 3;
        tmp += (statecode & 0b0100) >> 2;
        tmp += (statecode & 0b0010) >> 1;
        tmp += (statecode & 0b0001);
      }
      *tg += 1;
      break;
    }

    if (operationTime == 100)
      break;
  }
}

static void lineFollowerBackward(float operationTime, float power, int *tg)
{
  setPower(power);
  sec = 0;
  while (sec <= operationTime)
  {
    // HAL_TIM_PeriodElapsedCallback(&htim3);
    statecode = 0;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    value0 = Board_Get_ADCChannelValue(&hadc1, 0);
    value1 = Board_Get_ADCChannelValue(&hadc1, 1);
    value2 = Board_Get_ADCChannelValue(&hadc1, 2);
    value3 = Board_Get_ADCChannelValue(&hadc1, 3);

    if (value0 > 1000)
      statecode = statecode | 0b0001; // 8
    if (value1 > 1000)
      statecode = statecode | 0b0010; // 4
    if (value2 > 1000)
      statecode = statecode | 0b0100; // 2
    if (value3 > 1000)
      statecode = statecode | 0b1000; // 1

    switch (statecode)
    {
    case 0b1000:
      DLL(); //急左飄
      break;
    case 0b0100:
      DLS(); //微左飄
      break;
    case 0b0010:
      DRS(); //微右飄
      break;
    case 0b0001:
      DRL(); //急右飄
      break;
    case 0b0000:
      writeServo(90);
      break;
    case 0b0110:
      pulse_servo2 = 500 + 2000 * 90 / 180;
      pulse_servo3 = pulse_servo2;
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
      writeServo(90);
      *tg += 1;
      break;
    }

    if (operationTime == 100)
      brake;
  }
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
