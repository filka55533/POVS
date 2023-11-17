/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <malloc.h>
#include <stdlib.h>
#include "main.h"
 
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
 
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
RTC_HandleTypeDef hrtc;
 
 
volatile int isPressD1 = 0;
volatile int isPressD2 = 0;
volatile int isPressD3 = 0;

volatile int state = WAIT_GAME;
volatile int bestRecord = 0;
 
RTC_TimeTypeDef alarmTime, currentTime;
/* USER CODE END PV */
 
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
// static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
IRQn_Type GetButtonIRQ(uint16_t GPIO_Pin);
/* USER CODE END PFP */
 
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 
void WriteLed(int ledNumber, int value){
    if (ledNumber == 0)
    {
        HAL_GPIO_WritePin(D1_Port, D1_Pin, value);
    }
    else if (ledNumber == 1)
    {
        HAL_GPIO_WritePin(D2_Port, D2_Pin, value);
    }
    else if (ledNumber == 2)
    {
        HAL_GPIO_WritePin(D3_Port, D3_Pin, value);
    }
    else if (ledNumber == 3)
    {
        HAL_GPIO_WritePin(D4_Port, D4_Pin, value);
    }
}

void ResetLeds()
{
  for (int i = 0; i < 4; i++)
  {
    WriteLed(i, GPIO_PIN_SET);
  }
}


IRQn_Type GetButtonIRQ(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case SB1_Pin:
            return EXTI1_IRQn;
        case SB2_Pin:
            return EXTI4_IRQn;
        case SB3_Pin:
            return EXTI0_IRQn;
        default:
            return 0;
    }
}

const char digits[] = {
  0b11000000, //0
  0b11111001, //1
  0b10100100, //2
  0b10110000, //3
  0b10011001, //4
  0b10010010, //5
  0b10000010, //6
  0b11111000, //7
  0b10000000, //8
  0b10010000, //9
  0b11111111, // None
  0b10000110, //E
  0b10101111  //r
};

const char segments[] = {
  0xF1,
  0xF2,
  0xF4,
  0xF8
};

void ShiftingRegister(char byte)
{
  for (int i = 0; i < 8; i++)
  {
    HAL_GPIO_WritePin(Data_Port, Data_Pin, byte & 0x80 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Clock_Port, Clock_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Clock_Port, Clock_Pin, GPIO_PIN_SET);
    byte <<= 1;
  }
}

void SetLatch()
{
  HAL_GPIO_WritePin(Latch_Port, Latch_Pin, GPIO_PIN_SET);
}

void ResetLatch()
{
  HAL_GPIO_WritePin(Latch_Port, Latch_Pin, GPIO_PIN_RESET);
}

void PrintDigit(int segment, int digit)
{
  ResetLatch();
  ShiftingRegister(digits[digit]);
  ShiftingRegister(segments[segment]);
  SetLatch();
}

void PrintNumber(int number)
{
  for (int i = 3; i >= 0; i--)
  {
    PrintDigit(i, number % 10);
    number /= 10;
  }
}

void ClearDisplay()
{
  for (int i = 0; i < 4; i++)
  {
    PrintDigit(i, 10);
  }
}

void PrintError()
{
  PrintDigit(0, E_BIG);
  PrintDigit(1, R_LITTLE);
  PrintDigit(2, R_LITTLE);
  PrintDigit(3, NONE);
}

volatile int isNewGame = 0;
void ProcessWaitState() 
{
  for (int i = 0; i <= 2; i++)
  {
    WriteLed(i, GPIO_PIN_SET);
  }

  WriteLed(3, GPIO_PIN_RESET);
  PrintNumber(bestRecord);
  
  if (isPressD1)
  {
    isPressD1 = 0;
    WriteLed(3, GPIO_PIN_SET);
    ClearDisplay();
    isNewGame = 1;
    state = SHOW_LEVEL;
    HAL_Delay(500);
  }
}

volatile uint8_t levels[3][64];
void InitLevels()
{
  srand(HAL_GetTick());
  for (int i = 0; i < 64; i++)
  {
    uint8_t valueD1 = rand() % 0xFF;
    uint8_t tempRand = rand() % 0xFF;
    levels[0][i] = valueD1;
    levels[1][i] = 0;
    levels[2][i] = 0;
    for(int j = 0; j < 8; j++)
    {
      const uint8_t mask = 0x1 << j;
      if (!(valueD1 & mask))
      {
        levels[1][i] |= tempRand & mask;
        levels[2][i] |= ((tempRand & mask) ^ mask);
      }
    }
  }
}

volatile int currentLevel;
volatile int isOnShowing;
volatile int currentShowingIndex;
volatile uint32_t time;

volatile int isShowLed;
volatile int isOffLed;

void ClearPressedD()
{
  isPressD1 = isPressD2 = isPressD3 = 0;
}

void ProcessShowLevel()
{
  //Inititalization
  if (isNewGame)
  {
    currentLevel = 1;
    isOnShowing = 0;
    isNewGame = 0;
    ResetLeds();
  }

  PrintNumber(currentLevel);
  if (!isOnShowing)
  {
    currentShowingIndex = 0;
    isOnShowing = 1;
    time = HAL_GetTick();
    isShowLed = 0;
    isOffLed = 0;
  }

  uint32_t currentTime = HAL_GetTick();

  if (currentShowingIndex >= currentLevel)
  {
    ResetLeds();
    state = INPUT_LEVEL;
    isOnShowing = 0;
  } 
  else if (currentTime - time < BLINK_DELAY && !isOffLed)
  {
    for (int i = 0; i < 3; i++)
    {
      WriteLed(i, GPIO_PIN_SET);
    }
    isOffLed = 1;
    isShowLed = 0;
  }
  else if (currentTime - time >= BLINK_DELAY && currentTime - time < 2 * BLINK_DELAY && !isShowLed)
  {
    for (int i = 0; i < 3; i++)
    {
      WriteLed(
        i, 
        levels[i][currentShowingIndex / 8] & (0x1 << (currentShowingIndex % 8))? 
          GPIO_PIN_RESET : 
          GPIO_PIN_SET
      );
    }
    isShowLed = 1;
    isOffLed = 0;
  }
  else if (currentTime- time >= 2 * BLINK_DELAY)
  {
    currentShowingIndex++;
    time = currentTime;
  }
}

int isCorrectInput(int level)
{
  return !(isPressD1 ^ ((levels[0][level / 8] >> (level % 8)) & 0x1))
    && !(isPressD2 ^ ((levels[1][level / 8] >> (level % 8)) & 0x1))
    && !(isPressD3 ^ ((levels[2][level / 8] >> (level % 8)) & 0x1));
}

volatile int currentInputIndex;
volatile int isOnInput = 0;
volatile int isDisableInput;
volatile int isSuccess;
volatile uint32_t inpTime;
volatile int isErrorInput;
void ProcessInputLevel()
{
  if (!isOnInput)
  {
    currentInputIndex = 0;
    isDisableInput = 0;
    inpTime = HAL_GetTick();
    isOnInput = 1;
    isErrorInput = 0;
    isSuccess = 0;
    ClearPressedD();
  }

  PrintNumber(currentLevel);

  if (!isDisableInput)
  {
    if (isErrorInput || isSuccess)
    {
      isOnInput = 0;
      state = RESULTS_LEVEL;
    }
    else
    {
      if (isPressD1 || isPressD2 || isPressD3)
      {
        WriteLed(0, isPressD1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        WriteLed(1, isPressD2 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        WriteLed(2, isPressD3 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        if (isCorrectInput(currentInputIndex))
        {
          if (++currentInputIndex >= currentLevel)
          {
            //Next show
            isSuccess = 1;
          }
        }
        else
        {
          isErrorInput = 1;
        }
        isDisableInput = 1;
        inpTime = HAL_GetTick();
      }
    }
  }
  else
  {
    const uint32_t currentTime = HAL_GetTick();
    if (currentTime - inpTime > BLINK_DELAY)
    {
      inpTime = currentTime;
      isDisableInput = 0;
      ResetLeds();
      ClearPressedD();
    }
  }
}

volatile int isSuccessBlink;
volatile int isErrorBlink;
volatile int isOnResults = 0;
void ProcessResultsLevel()
{
  if (!isOnResults)
  { 
    isSuccessBlink = 0;
    isErrorBlink = 0;
    inpTime = HAL_GetTick();
    ResetLeds();
    isOnResults = 1;
  }

  if (isSuccess)
  {
    PrintNumber(currentLevel);
    if (isSuccessBlink)
    {
      const uint32_t currentTime = HAL_GetTick();
      if (currentTime - inpTime >= BLINK_DELAY)
      {
        ResetLeds();
        currentLevel++;
        isOnResults = 0;
        state = SHOW_LEVEL;
      }
    }
    else 
    {
      inpTime = HAL_GetTick();
      for (int i = 0; i < 3; i++)
      {
        WriteLed(i, GPIO_PIN_RESET);
      }
      isSuccessBlink = 1;
    }
  }
  else if (isErrorInput)
  {
    PrintError();
    if (isErrorBlink)
    {
      const uint32_t currentTime = HAL_GetTick();
      if (currentTime - inpTime >= BLINK_DELAY)
      {
        bestRecord = MAX(currentLevel - 1, bestRecord);
        currentLevel = 1;
        isOnResults = 0;
        state = WAIT_GAME;
      }
    }
    else
    {
      inpTime = HAL_GetTick();
      isErrorBlink = 1;
    }
  }
}

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
  // MX_TIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  InitLevels();
 
  while (1)
  {
    switch (state)
    {
      case WAIT_GAME:
        ProcessWaitState();
        break;
      case SHOW_LEVEL:
        ProcessShowLevel();
        break;
      case INPUT_LEVEL:
        ProcessInputLevel();
        break;
      case RESULTS_LEVEL:
        ProcessResultsLevel();
        break;
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
 
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}
 
/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
 
  /* USER CODE BEGIN RTC_Init 0 */
 
  /* USER CODE END RTC_Init 0 */
 
  /* USER CODE BEGIN RTC_Init 1 */
 
  /* USER CODE END RTC_Init 1 */
 
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
 
  /* USER CODE END RTC_Init 2 */
 
}
 
/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */


// static void MX_TIM1_Init(void)
// {
 
//   /* USER CODE BEGIN TIM1_Init 0 */
 
//   /* USER CODE END TIM1_Init 0 */
 
//   // TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//   // TIM_MasterConfigTypeDef sMasterConfig = {0};
 
//   // /* USER CODE BEGIN TIM1_Init 1 */
 
//   // /* USER CODE END TIM1_Init 1 */
//   // htim1.Instance = TIM1;
//   // htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//   // htim1.Init.Prescaler = 6399;
//   // htim1.Init.Period = 10000;
//   // htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//   // htim1.Init.RepetitionCounter = 0;
//   // htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//   // if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//   // {
//   //   Error_Handler();
//   // }
//   // sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//   // if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
//   // {
//   //   Error_Handler();
//   // }
//   // sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//   // sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//   // if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//   // {
//   //   Error_Handler();
//   // }
//   /* USER CODE BEGIN TIM1_Init 2 */
 
//   /* USER CODE END TIM1_Init 2 */
 
// }
 
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */
 
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
 
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D1_Pin|D3_Pin|D2_Pin|Clock_Pin
                          |Data_Pin, GPIO_PIN_RESET);
 
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Latch_Pin|D4_Pin, GPIO_PIN_RESET);
 
  /*Configure GPIO pins : BTN_A1_Pin BTN_A2_Pin */
  GPIO_InitStruct.Pin = SB1_Pin|SB2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
  /*Configure GPIO pins : LED4_Pin LED3_Pin LED2_Pin SHCP_Pin
                           DS_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D3_Pin|D2_Pin|Clock_Pin
                          |Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
  /*Configure GPIO pin : BTN_A3_Pin */
  GPIO_InitStruct.Pin = SB3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SB3_Port, &GPIO_InitStruct);
 
  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
 
  /*Configure GPIO pins : STCP_Pin LED1_Pin */
  GPIO_InitStruct.Pin = Latch_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
 
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
 
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
 
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
 
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
 
/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//  blickLed(0);
 
    HAL_NVIC_DisableIRQ(GetButtonIRQ(GPIO_Pin));
 
    if (GPIO_Pin == SB1_Pin){
        isPressD1 = 1;
    }
    if (GPIO_Pin == SB2_Pin){
        isPressD2 = 1;
    }
    if (GPIO_Pin == SB3_Pin){
        isPressD3 = 1;
    }
 
    HAL_NVIC_EnableIRQ(GetButtonIRQ(GPIO_Pin));
}
 
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
 
//     showNumberOnDisplay(0, 1);
 
// }
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