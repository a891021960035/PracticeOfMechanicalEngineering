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
#include "ftoa.h"
#include "math.h"
#include "fonts.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define _USE_MATH_DEFINES
#define MIN_PULSE_LENGTH 20000 * 0.05 // Minimum pulse length : 1000µs
#define MAX_PULSE_LENGTH 20000 * 0.1  // Maximum pulse length : 2000µs
// #define CYCLE 46
#define UNIT_DISTANCE 102 / 155
#define R 5.5 / 2
#define REARTRACK 14.06
#define WHEELBASE 17.75

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
char buffer[5];
int value0 = -1; // ADC value 最右
int value1 = -1; // ADC value 右
int value2 = -1; // ADC value 左
int value3 = -1; // ADC value 最左
int encoderRA = 0;
int encoderRB = 0;
int encoderLA = 0;
int encoderLB = 0;
int position_encoderR = 0;
int position_encoderL = 0;
int statecode = 0b0000;
int encoderRid = 0b0000;
int encoderLid = 0b0000;
// int encodercode[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
int encodercode[16] = {0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, -1, 1, 0, 0};
float disR;
float disL;
float steering_degree;
int pulse_servo1 = 0; // Servo 1 PWM pulse
int pulse_servo2 = 0; // Servo 2 PWM pulse
int pulse_servo3 = 0; // Servo 3 PWM pulse
int pulse_BLDC = 0;   // PWM pulse
float degree_servo = 90;
int mode = 0; // ESC mode
float tmp = 0;
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
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint32_t Board_Get_ADCChannelValue(ADC_HandleTypeDef *hadc, uint32_t channel);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
float distanceR();
float distanceL();
float steeringDegree(float angle, int orientation);
static void writeServo(float angle);
static void setPower(float power);
static void brake();
static void unbrake();
static void TRS(); //微右轉
static void TRL(); //急右轉
static void TLS(); //微左轉
static void TLL(); //急左轉
static void DRS(); //微右飄
static void DRL(); //急右飄
static void DLS(); //微左飄
static void DLL(); //急左飄
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
  // encoderRA = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
  // encoderRB = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
  // encoderRid = (encoderRA << 3) + (encoderRB << 2) + (encoderRA << 1) + encoderRB;
  // encoderLA = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
  // encoderLB = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
  // encoderLid = (encoderLA << 3) + (encoderLB << 2) + (encoderLA << 1) + encoderLB;
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                 // Servo 1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);                 // Servo 2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);                 // Servo 3
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);                 // 無刷馬達
  HAL_TIM_Base_Start_IT(&htim3);                            // 開啟中斷
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_BLDC); // 無刷馬達下限轉速
  writeServo(90);
  brake();
  // unbrake();
  HAL_Delay(3000);
  // HAL_NVIC_SystemReset();
  SSD1306_Init();
  mode = 2;
  // SSD1306_GotoXY(0, 0);
  // SSD1306_Puts("Start!", &Font_16x26, 1);
  SSD1306_GotoXY(10, 10);                // goto 10, 10
  SSD1306_Puts("HELLO", &Font_11x18, 1); // print Hello
  SSD1306_GotoXY(10, 30);
  SSD1306_Puts("WORLD !!", &Font_11x18, 1);
  SSD1306_UpdateScreen();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    switch (mode)
    {
    case 0:
      setPower(0);
      break;

    case 1:
      setPower(10);
      break;

    case 2:
      // 起步
      setPower(6);
      HAL_Delay(2000);

      // 第 1 段，轉彎至第一循跡線
      position_encoderR = 0;
      position_encoderL = 0;
      degree_servo = 77.5;
      writeServo(degree_servo); // 18.7-> 9.35 -> 14.035 -> 13.5 -> 12.5
      unbrake();
      SSD1306_Clear();
      while (steeringDegree(degree_servo, 1) < 90)
      {
        ftoa(steeringDegree(degree_servo, 1), buffer, 2);
        SSD1306_GotoXY(0, 0);
        SSD1306_Puts(buffer, &Font_11x18, 1);
        SSD1306_UpdateScreen();
      }
      SSD1306_Clear();
      ftoa(steeringDegree(degree_servo, 1), buffer, 2);
      SSD1306_GotoXY(0, 0);
      SSD1306_Puts(buffer, &Font_16x26, 1);
      SSD1306_UpdateScreen();

      // 第 4 段，第一段循跡
      lineFollower(4, 6, &trigger);
      trigger = 0;
      while (trigger < 1)
      {
        lineFollower(100, 25, &trigger);
      }

      // // 第 5 段，變換車道
      // setPower(20); // 小電池：20 大電池 24
      // position_encoderR = 0;
      degree_servo = 60;
      writeServo(degree_servo);
      // SSD1306_Clear();
      // while (steeringDegree(degree_servo, 1) < 35)
      // {
      // }
      // SSD1306_Clear();
      // ftoa(steeringDegree(degree_servo, 1), buffer, 2);
      // SSD1306_GotoXY(0, 0);
      // SSD1306_Puts(buffer, &Font_16x26, 1);
      // SSD1306_UpdateScreen();

      // // 等到黑線停下
      // for (int i = 60; i <= 90; i += 2)
      // {
      //   writeServo(i);
      // }
      pulse_servo2 = 500 + 2000 * degree_servo / 180;
      pulse_servo3 = 500 + 2000 * degree_servo / 180; // 130?
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
      setPower(30); // 大顆電池 25 度：30 小顆電池 25 度：30
      waitBlack(1);

      // 第 6 段，修正路徑
      // 轉回黑線_往前
      // writeServo(138);
      // waitBlack(1);
      // position_encoderR = 0;
      // degree_servo = 120;
      // for (int i = 90; i <= degree_servo; i += 2)
      // {
      //   writeServo(i);
      // }
      // setPower(25); // 小電池：25 大電池 30
      // SSD1306_Clear();
      // while (steeringDegree(degree_servo, 0) < 35)
      // {
      // }
      // SSD1306_Clear();
      // ftoa(steeringDegree(degree_servo, 0), buffer, 2);
      // SSD1306_GotoXY(0, 0);
      // SSD1306_Puts(buffer, &Font_16x26, 1);
      // SSD1306_UpdateScreen();
      // 轉回黑線_往後
      // setPower(0);
      // brake();
      // HAL_Delay(800);
      // writeServo(82);
      // pulse_servo2 = 500 + 2000 * 115 / 180;
      // pulse_servo3 = 500 + 2000 * 130 / 180; // 130?
      // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
      // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
      // waitBlack(1);
      // brake();
      // HAL_Delay(500);
      // unbrake();

      // // 第 7 段，第二循跡線上坡
      // trigger = 0;
      // // 向後轉正模式
      // while (trigger < 2)
      // {
      //   lineFollower(100, 17.5, &trigger);
      // }
      // writeServo(90);
      // HAL_Delay(300);

      // // 第 8 段，第一停止區
      // setPower(0);
      // brake();
      // HAL_Delay(3500);
      // unbrake();

      // // 第 9 段，循跡至第二停止區
      // lineFollowerBackward(5, 6.3, &trigger);
      // lineFollowerBackward(4, 0, &trigger);
      brake();
      mode = 0;
      break;

    case 3: // 伺服測試
      writeServo(120);
      break;

    case 4: // 無刷測試
      HAL_Delay(3000);
      setPower(13);
      // BLDC_test();
      break;

    case 5: // 類比輸入
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 1);
      value0 = Board_Get_ADCChannelValue(&hadc1, 0);
      value1 = Board_Get_ADCChannelValue(&hadc1, 1);
      value2 = Board_Get_ADCChannelValue(&hadc1, 2);
      value3 = Board_Get_ADCChannelValue(&hadc1, 3);
      HAL_Delay(500);
      break;

    case 6: // Encoder
      //             CW --->
      // A          ¯|___|¯¯¯¯|___|¯¯¯¯
      // Interrupts  ^   ^    ^   ^
      // B          ¯¯¯|___|¯¯¯¯¯|___|¯
      // Interrupts    ^   ^     ^   ^
      //             CCW <---
      break;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // if (GPIO_Pin == GPIO_PIN_15)
  // {
  //   encoderRA = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
  //   encoderRB = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
  //   encoderRid = ((encoderRid << 2) & 0b1111) + (encoderRA << 1) + encoderRB;
  //   position_encoderR = position_encoderR + encodercode[encoderRid];
  // }

  // if (GPIO_Pin == GPIO_PIN_13)
  // {
  //   encoderLA = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
  //   encoderLB = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
  //   encoderLid = ((encoderLid << 2) & 0x0F) + (encoderLA << 1) + encoderLB;
  //   position_encoderL = position_encoderL - encodercode[encoderLid];
  // }

  position_encoderR++;
}

float distanceR()
{
  // disR = position_encoderR / CYCLE * 2 * M_PI * R;
  disR = position_encoderR * UNIT_DISTANCE;
  return disR;
}

float distanceL()
{
  // disL = position_encoderL / CYCLE * 2 * M_PI * R;
  return disL;
}

float steeringDegree(float angle, int orientation)
{
  float result;
  if (orientation == 1) //左轉
  {
    angle = 90 - angle + 5;
    result = (distanceR() / (WHEELBASE / tan(angle * M_PI / 180) + 7.03)) / M_PI * 180;
  }
  if (orientation == 0) //右
  {
    angle = angle - 90;
    result = (distanceR() / (WHEELBASE / tan(angle * M_PI / 180) - 7.03)) / M_PI * 180;
  }

  return result;
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
  writeServo(101.25); // 112.5
}

static void TRL(void) //急右轉
{
  writeServo(112.5); // 135
}

static void TLS(void) //微左轉
{
  writeServo(78.75); // 67.5
}

static void TLL(void) //急左轉
{
  writeServo(67.5); // 45
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
      pulse_servo2 = 500 + 2000 * 90 / 180;
      pulse_servo3 = pulse_servo2;
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_servo2);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_servo3);
      writeServo(90);
      break;
    }

    if (operationTime == 100)
      break;
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

#ifdef USE_FULL_ASSERT
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
