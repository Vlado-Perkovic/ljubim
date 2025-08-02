/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "comp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include <stdint.h>
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
extern uint32_t open_loop_period_us;
extern volatile motor_step_t current_motor_step;
extern volatile uint8_t go;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void handle_zero_crossing(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern osThreadId_t motor_control_hndl;
extern TIM_HandleTypeDef htim3;
extern volatile OneShotSource_t g_one_shot_source;
extern volatile GPIO_PinState cmp1, cmp2, cmp3;
volatile GPIO_PinState old_cmp1, old_cmp2, old_cmp3;
volatile uint8_t safeguard = 0;
// volatile uint8_t is_zero_crossing_detected = 0;

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_COMP1_Init();
  MX_TIM17_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void handle_zero_crossing(TIM_HandleTypeDef *htim) {
  int32_t elapsed_cnt = __HAL_TIM_GET_COUNTER(htim);
  int32_t current_period = __HAL_TIM_GET_AUTORELOAD(htim);

  int32_t err = current_period / 2 - elapsed_cnt;

  open_loop_period_us = current_period - err/20;
  __HAL_TIM_SET_AUTORELOAD(htim, open_loop_period_us);
  if (elapsed_cnt + err/2 >= open_loop_period_us) {
  __HAL_TIM_SET_COUNTER(htim, open_loop_period_us-1);
  } else {
  __HAL_TIM_SET_COUNTER(htim, elapsed_cnt + err/2);
  }
  safeguard = 1;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_TIM_Base_Stop_IT(&htim15);

    __HAL_TIM_SET_COUNTER(&htim15, 0);

    htim15.Instance->CR1 |= TIM_CR1_OPM; // One Pulse Mode

    HAL_TIM_Base_Start_IT(&htim15);
  }
  if (htim->Instance == TIM15 && go) {
    switch (current_motor_step) {
    case MOTOR_STEP_1: // A->B, C is floating ---> down
      cmp3 = -1;
      old_cmp1 = cmp1;
      cmp1 = HAL_GPIO_ReadPin(CMP3_GPIO_Port, CMP3_Pin);
      if (cmp1 == GPIO_PIN_RESET && old_cmp1 == GPIO_PIN_SET &&
          safeguard == 0) {
        handle_zero_crossing(&htim3);
      }
      break;

    case MOTOR_STEP_2: // A->C, B is floating ----> up
      cmp1 = -1;
      old_cmp2 = cmp2;
      cmp2 = HAL_GPIO_ReadPin(CMP1_GPIO_Port, CMP1_Pin);
      if (cmp2 == GPIO_PIN_SET && old_cmp2 == GPIO_PIN_RESET &&
          safeguard == 0) {
        handle_zero_crossing(&htim3);
      }
      break;

    case MOTOR_STEP_3: // B->C, A is floating -----> down
      cmp2 = -1;
      old_cmp3 = cmp3;
      cmp3 = HAL_GPIO_ReadPin(CMP2_GPIO_Port, CMP2_Pin);
      if (cmp3 == GPIO_PIN_RESET && old_cmp3 == GPIO_PIN_SET &&
          safeguard == 0) {
        handle_zero_crossing(&htim3);
      }
      break;

    case MOTOR_STEP_4: // B->A, C is floating ----> up
      cmp3 = -1;
      old_cmp1 = cmp1;
      cmp1 = HAL_GPIO_ReadPin(CMP3_GPIO_Port, CMP3_Pin);
      if (cmp1 == GPIO_PIN_SET && old_cmp1 == GPIO_PIN_RESET &&
          safeguard == 0) {
        handle_zero_crossing(&htim3);
      }
      break;

    case MOTOR_STEP_5: // C->A, B is floating -----> down
      cmp1 = -1;
      old_cmp2 = cmp2;
      cmp2 = HAL_GPIO_ReadPin(CMP1_GPIO_Port, CMP1_Pin);
      if (cmp2 == GPIO_PIN_RESET && old_cmp2 == GPIO_PIN_SET &&
          safeguard == 0) {
        handle_zero_crossing(&htim3);
      }
      break;

    case MOTOR_STEP_6: // C->B, A is floating ----> up
      cmp2 = -1;
      old_cmp3 = cmp3;
      cmp3 = HAL_GPIO_ReadPin(CMP2_GPIO_Port, CMP2_Pin);
      if (cmp3 == GPIO_PIN_SET && old_cmp3 == GPIO_PIN_RESET &&
          safeguard == 0) {
        handle_zero_crossing(&htim3);
      }
      break;

    case MOTOR_STOP:
    default:
      break;
    }
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM3) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the motor control task that it is time to commutate
    if (motor_control_hndl != NULL) {
      vTaskNotifyGiveFromISR((TaskHandle_t)motor_control_hndl,
                             &xHigherPriorityTaskWoken);
    }

    // If the notification woke up a higher-priority task, yield the CPU
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
