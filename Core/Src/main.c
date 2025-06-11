/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Ultrasonic distance meter with LCD1602 and PWM alert system
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * Licensed under the terms found in the LICENSE file in the root directory
  * of this software component. If no LICENSE file is present, this software is
  * provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "lcd1602_i2c_lib.h"
#define TRIG_PIN GPIO_PIN_0
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_1
#define ECHO_PORT GPIOA

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

/* USER CODE BEGIN PV */
float distance;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void DWT_Delay_Init(void);
void DWT_Delay_us(uint32_t us);
float GetDistance(void);

/* DWT delay initialization for microsecond precision */
void DWT_Delay_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Microsecond delay using DWT */
void DWT_Delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < delayTicks);
}

/* Measures distance using HC-SR04 ultrasonic sensor */
float GetDistance(void) {
    uint32_t startTime, endTime, duration;

    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    DWT_Delay_us(2);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    DWT_Delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET);
    startTime = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET);
    endTime = DWT->CYCCNT;

    duration = endTime - startTime;
    float time_us = duration / (SystemCoreClock / 1000000.0f);
    return time_us * 0.0343f / 2.0f; // Convert to centimeters
}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();
  lcd1602_Init();
  lcd1602_SetCursor( 0, 0);
  lcd1602_Print_text("Distance:");

  /* Start PWM channels */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PA6
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PA7
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // PB0
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // PB1
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // PA15 (buzzer)
  uint32_t lastLCDUpdate = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  distance = GetDistance();
	  /* Update LCD every 400 ms */
	  if (HAL_GetTick() - lastLCDUpdate >= 400)
	 {
	  lastLCDUpdate = HAL_GetTick(); // Remembering the time of the last update

	  char buffer[16];
	  snprintf(buffer, sizeof(buffer), "%.2f cm", distance);
	  lcd1602_SetCursor( 0, 1);
	  lcd1602_Clean();  // Clear second line
	  lcd1602_Print_text(buffer);
	 }

	  /* PWM control logic */
      uint32_t duty1 = 0, duty2 = 0, duty3 = 0, duty4 = 0;
      uint32_t buzzerDuty = 0;  // Borehole for the buzzer

      if (distance <= 40.0f) {
          float step = 40.0f / 4.0f;  // divide 40 cm into 4 zones of 10 cm each.

          if (distance > step * 3) {
              duty1 = (400 / step) * (40 - distance); // Smooth build-up CH1
              duty2 = 0;
              duty3 = 0;
              duty4 = 0;
              buzzerDuty = 30; // 3% at 40 cm
          } else if (distance > step * 2) {
              duty1 = 400;
              duty2 = (400 / step) * (30 - distance); // Smooth build-up CH2
              duty3 = 0;
              duty4 = 0;
              buzzerDuty = 150; // 15% at 30 cm
          } else if (distance > step * 1) {
              duty1 = 400;
              duty2 = 400;
              duty3 = (1000 / step) * (20 - distance); // Smooth build-up CH3
              duty4 = 0;
              buzzerDuty = 300; // 30% at 20 cm
          } else {
              duty1 = 400;
              duty2 = 400;
              duty3 = 1000;
              duty4 = (1000 / step) * (10 - distance); // Smooth build-up CH4
              buzzerDuty = 700; // 70% at 10 cm
          }
         } else {
              buzzerDuty = 0; // Turn off buzzer if > 40 см
      }

      /* Update PWM outputs */
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty1);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty2);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty3);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty4);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, buzzerDuty);

      HAL_Delay(50);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
}

/* USER CODE BEGIN 4 */


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
  while (1){
	  // Stay here on error
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
