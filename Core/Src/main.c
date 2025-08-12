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
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "tim.h"
#include "timer_utils.h"
#include <stdint.h>
#include <stdio.h>
#include "utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define PERIOD 200
// #define DUTY_CYCLE 12
// #define KI 20
// #define KP 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// const uint32_t step_times_us[] = {
//     34500, 14291, 10966, 9244, 8144, 7363, 6771, 6302, 5919, 5599, 5325,
//     5088, 4880,  4696,  4531,  4382, 4247, 4124, 4011, 3907, 3810, 3721,
//     3637, 3559, 3485,  3416,  3351,  3290, 3231, 3176, 3124, 3074, 3026,
//     2980, 2937, 2895, 2855,  2817,  2780,  2745, 2711, 2678, 2646, 2616,
//     2586, 2557, 2530, 2503, 2477,  2452,  2427,  2404, 2381, 2358, 2337,
//     2316, 2295, 2275, 2255, 2236, 2218,  2200,  2182,  2165, 2148, 2131,
//     2115, 2100, 2084, 2069, 2054, 2040, 2026,  2012,  1999,  1985, 1972,
//     1960, 1947, 1935, 1923, 1911, 1899, 1888, 1877,  1866,  1855,  1844,
//     1834, 1823, 1813, 1803, 1794, 1784, 1775, 1765, 1756,  1747,  1738, 1729,
//     1721, 1712, 1704, 1696, 1687, 1679, 1672, 1664, 1656,  1648,  1641, 1634,
//     1626, 1619, 1612, 1605, 1598, 1591, 1585, 1578, 1571,  1565,  1559, 1552,
//     1546, 1540, 1534, 1528, 1522, 1516, 1510, 1504, 1499,  1493,  1487, 1482,
//     1476, 1471, 1466, 1461, 1455, 1450, 1445, 1440, 1435,  1430,  1425, 1420,
//     1416, 1411, 1406, 1401, 1397, 1392, 1388, 1383, 1379,  1375,  1370, 1366,
//     1362, 1357, 1353, 1349, 1345, 1341, 1337, 1333, 1329,  1325,  1321, 1317,
//     1313, 1310, 1306, 1302, 1298, 1295, 1291, 1288, 1284,  1280,  1277, 1273,
//     1270, 1267, 1263, 1260, 1256, 1253, 1250, 1247, 1243,  1240,  1237, 1234,
//     1231, 1227, 1224, 1221, 1218, 1215, 1212, 1209, 1206,  1203,  1200, 1198,
//     1195, 1192, 1189, 1186, 1183, 1181, 1178, 1175, 1172,  1170,  1167, 1164,
//     1162, 1159, 1156, 1154, 1151, 1149, 1146, 1144, 1141,  1139,  1136, 1134,
//     1131, 1129, 1126, 1124, 1122, 1119, 1117, 1115, 1112,  1110,  1108, 1105,
//     1103, 1101, 1099, 1096, 1094, 1092, 1090, 1088, 1086,  1083,  1081, 1079,
//     1077, 1075, 1073, 1071, 1069, 1067, 1065, 1063, 1061,  1059,  1057, 1055,
//     1053, 1051, 1049, 1047, 1045, 1043, 1041, 1039, 1037,  1036,  1034, 1032,
//     1030, 1028, 1026, 1025, 1023, 1021, 1019, 1017, 1016,  1014,  1012, 1010,
//     1009, 1007, 1005, 1003, 1002, 1000, 998,  997, 995,   993,   992,   990,
//     989,  987,  985,  984,  982,  981,  979,  977, 976,   974,   973,   971,
//     970,  968,  967,  965,  964,  962,  961,  959, 958,   956,   955,   953,
//     952,  950,  949,  947,  946,  945,  943,  942,
// };
// // extern volatile GPIO_PinState cmp1, cmp2, cmp3;
// // extern volatile uint8_t safeguard;
// // extern void handle_zero_crossing(TIM_HandleTypeDef *htim);
// // extern volatile uint32_t zc_cnt;
//
// uint32_t open_loop_period_us = ALIGN_TIMEOUT_US;
// volatile motor_step_t current_motor_step = MOTOR_STEP_4;
// volatile uint8_t go = 0;
// volatile bemf_case_t state = BEMF_ERROR;
// volatile bemf_case_t prev_state = BEMF_ERROR;
// volatile ctx_t context = {.elapsed_cnt_at_bemf = 0,
//                           .last_elapsed_cnt_at_bemf = 0,
//                           .current_period = 0,
//                           .back_to_back = 0,
//                           .last_period = 0};
// uint32_t zc_times[200] = {0};
// // extern volatile uint8_t zc_flag;
// volatile uint8_t commutate_flag = 0;
// volatile uint32_t zc_period = 0;
// volatile uint32_t zc_period_filt = 0;
// volatile uint32_t zc_period_prev = 0;
// volatile uint32_t half = 0;
// volatile float coef_half = 0.375f;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint32_t open_loop_period_us;
extern volatile motor_step_t current_motor_step;
extern volatile uint8_t go;
extern volatile bemf_case_t state;
extern volatile ctx_t context;
extern volatile uint8_t enable_bemf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
// void motor_handle_bemf();
void handle_zero_crossing(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern osThreadId_t motor_control_hndl;
extern TIM_HandleTypeDef htim3;
// extern volatile OneShotSource_t g_one_shot_source;
extern volatile GPIO_PinState cmp1, cmp2, cmp3;
volatile GPIO_PinState old_cmp1, old_cmp2, old_cmp3;
volatile uint8_t safeguard = 0;
volatile uint32_t zc_cnt = 0;
volatile uint8_t zc_flag = 0;
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
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  // uint32_t step_counter = 0;
  // motor_init();
  //
  // set_commutation_period_us_255(open_loop_period_us);
  // HAL_TIM_Base_Start_IT(&htim3);
  // // HAL_COMP_Start(&hcomp1);
  // float duty_cycle = DUTY_CYCLE;
  // int cnt = 0;
  // uint32_t err;
  //
  // for (;;) {
  //   // Wait here indefinitely for a notification from the timer ISR
  //   if (zc_flag == 1) {
  //     zc_flag = 0;
  //     zc_period = context.last_period - context.last_elapsed_cnt_at_bemf +
  //                 context.elapsed_cnt_at_bemf;
  //     if (zc_cnt == 1) {
  //       zc_period_prev = zc_period;
  //     }
  //
  //     // zc_period_filt = zc_period/2 + zc_period_prev/2;
  //     zc_period_filt = (zc_period >> 1) + (zc_period_prev >> 1);
  //
  //     half = (uint32_t)(coef_half * (float)zc_period_filt);
  //
  //     __HAL_TIM_SET_AUTORELOAD(&htim3, context.elapsed_cnt_at_bemf + half);
  //     // printf("per: %ld prev: %ld filt: %ld \r\n", (long)zc_period,
  //     // (long)zc_period_prev, (long)zc_period_filt);
  //     zc_period_prev = zc_period;
  //   }
  //
  //   if (commutate_flag) {
  //
  //     // #TODO: safeguard == is_bemf_detected
  //     if (go) {
  //
  //       switch (current_motor_step) {
  //       case MOTOR_STEP_1: // A->B, C is floating ---> down
  //
  //         /* because of the inverse comparator outputs
  //          * as the C is floating down:
  //          * UNDERSHOOT (change happened before the window)-> !LOW -> HIGH
  //          * OVERSHOOT (change happened after the window)-> !HIGH -> LOW
  //          * ERROR (NEXT BEMF TRIGGER HAPPENED BEFORE THIS POINT)
  //          */
  //         if (state != BEMF_VALID) {
  //           if (cmp1 == GPIO_PIN_SET)
  //             state = BEMF_UNDERSHOOT;
  //           else if (cmp1 == GPIO_PIN_RESET)
  //             state = BEMF_OVERSHOOT;
  //           else
  //             state = BEMF_ERROR;
  //         }
  //
  //         motor_handle_bemf();
  //         break;
  //
  //       case MOTOR_STEP_2: // A->C, B is floating ----> up
  //         /* because of the inverse comparator outputs
  //          * as the B is floating up:
  //          * UNDERSHOOT (change happened before the window)-> !HIGH -> LOW
  //          * OVERSHOOT (change happened after the window)-> !LOW -> HIGH
  //          * ERROR (NEXT BEMF TRIGGER HAPPENED BEFORE THIS POINT)
  //          */
  //         if (state != BEMF_VALID) {
  //           if (cmp2 == GPIO_PIN_RESET)
  //             state = BEMF_UNDERSHOOT;
  //           else if (cmp2 == GPIO_PIN_SET)
  //             state = BEMF_OVERSHOOT;
  //           else
  //             state = BEMF_ERROR;
  //         }
  //         motor_handle_bemf();
  //         break;
  //
  //       case MOTOR_STEP_3: // B->C, A is floating -----> down
  //         /* because of the inverse comparator outputs
  //          * as the A is floating DOWN:
  //          * UNDERSHOOT (change happened before the window)-> !LOW -> HIGH
  //          * OVERSHOOT (change happened after the window)-> !HIGH -> LOW
  //          * ERROR (NEXT BEMF TRIGGER HAPPENED BEFORE THIS POINT)
  //          */
  //         if (state != BEMF_VALID) {
  //           if (cmp3 == GPIO_PIN_SET)
  //             state = BEMF_UNDERSHOOT;
  //           else if (cmp3 == GPIO_PIN_RESET)
  //             state = BEMF_OVERSHOOT;
  //           else
  //             state = BEMF_ERROR;
  //         }
  //         motor_handle_bemf();
  //         break;
  //
  //       case MOTOR_STEP_4: // B->A, C is floating ----> up
  //         /* because of the inverse comparator outputs
  //          * as the C is floating up:
  //          * UNDERSHOOT (change happened before the window)-> !HIGH -> LOW
  //          * OVERSHOOT (change happened after the window)-> !LOW -> HIGH
  //          * ERROR (NEXT BEMF TRIGGER HAPPENED BEFORE THIS POINT)
  //          */
  //         if (state != BEMF_VALID) {
  //           if (cmp1 == GPIO_PIN_RESET)
  //             state = BEMF_UNDERSHOOT;
  //           else if (cmp1 == GPIO_PIN_SET)
  //             state = BEMF_OVERSHOOT;
  //           else
  //             state = BEMF_ERROR;
  //         }
  //         motor_handle_bemf();
  //         break;
  //
  //       case MOTOR_STEP_5: // C->A, B is floating -----> down
  //         /* because of the inverse comparator outputs
  //          * as the B is floating DOWN:
  //          * UNDERSHOOT (change happened before the window)-> !LOW -> HIGH
  //          * OVERSHOOT (change happened after the window)-> !HIGH -> LOW
  //          * ERROR (NEXT BEMF TRIGGER HAPPENED BEFORE THIS POINT)
  //          */
  //         if (state != BEMF_VALID) {
  //           if (cmp2 == GPIO_PIN_SET)
  //             state = BEMF_UNDERSHOOT;
  //           else if (cmp2 == GPIO_PIN_RESET)
  //             state = BEMF_OVERSHOOT;
  //           else
  //             state = BEMF_ERROR;
  //         }
  //         motor_handle_bemf();
  //         break;
  //
  //       case MOTOR_STEP_6: // C->B, A is floating ----> up
  //         /* because of the inverse comparator outputs
  //          * as the A is floating up:
  //          * UNDERSHOOT (change happened before the window)-> !HIGH -> LOW
  //          * OVERSHOOT (change happened after the window)-> !LOW -> HIGH
  //          * ERROR (NEXT BEMF TRIGGER HAPPENED BEFORE THIS POINT)
  //          */
  //         if (state != BEMF_VALID) {
  //           if (cmp3 == GPIO_PIN_RESET)
  //             state = BEMF_UNDERSHOOT;
  //           else if (cmp3 == GPIO_PIN_SET)
  //             state = BEMF_OVERSHOOT;
  //           else
  //             state = BEMF_ERROR;
  //         }
  //         motor_handle_bemf();
  //         break;
  //
  //       case MOTOR_STOP:
  //       default:
  //         break;
  //       }
  //     }
  //     state = BEMF_ERROR;
  //     safeguard = 0;
  //     current_motor_step =
  //         (motor_step_t)((current_motor_step + 1) % MOTOR_STEP_COUNT);
  //     motor_step(current_motor_step, duty_cycle);
  //     step_counter++;
  //
  //     if (step_counter < 3)
  //       continue;
  //     if (step_counter == 3) {
  //       timer_update_prescaler(&htim3, 63, TIMER_UPDATE_IMMEDIATE);
  //       open_loop_period_us = step_times_us[step_counter - 3];
  //       set_commutation_period_us_63(open_loop_period_us);
  //       continue;
  //     }
  //
  //     if (open_loop_period_us > 3300 && go == 0) {
  //
  //       open_loop_period_us = step_times_us[step_counter - 3];
  //       set_commutation_period_us_63(open_loop_period_us);
  //     } else {
  //       go = 1;
  //       open_loop_period_us =
  //           (open_loop_period_us > 3000) ? 3000 : 2 * zc_period_filt;
  //       context.last_period = context.current_period;
  //       context.current_period = open_loop_period_us;
  //
  //       __HAL_TIM_SET_AUTORELOAD(&htim3, open_loop_period_us);
  //     }
  //
  //     if (step_counter == 2000)
  //       duty_cycle += 2;
  //     // if (step_counter == 3000) duty_cycle += 5;
  //     commutate_flag = 0;
  //   }
  // }
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
  //
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void handle_zero_crossing(TIM_HandleTypeDef *htim) {


  context.last_elapsed_cnt_at_bemf = context.elapsed_cnt_at_bemf - 100;
  context.last_period = context.current_period;
  context.elapsed_cnt_at_bemf = __HAL_TIM_GET_COUNTER(htim);
  context.current_period = __HAL_TIM_GET_AUTORELOAD(htim);

  if ((context.elapsed_cnt_at_bemf > (context.current_period >> 4)) &&
      (context.elapsed_cnt_at_bemf <
       (context.current_period - (context.current_period >> 4)))) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
    safeguard = 1;
    state = BEMF_VALID;
    zc_flag = 1;
    if (zc_cnt < 10000)
      zc_cnt++;
  }
  // int32_t err = current_period / 2 - elapsed_cnt;
  //
  // open_loop_period_us = current_period - err/20;
  // __HAL_TIM_SET_AUTORELOAD(htim, open_loop_period_us);
  // if (elapsed_cnt + err/2 >= open_loop_period_us) {
  // __HAL_TIM_SET_COUNTER(htim, open_loop_period_us-1);
  // } else {
  // __HAL_TIM_SET_COUNTER(htim, elapsed_cnt + err/2);
  // }
  // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
}
// void motor_handle_bemf() {
//   int32_t err;
//
//   if (state == BEMF_UNDERSHOOT) {
//     /* DECREASE PERIOD */
//     context.elapsed_cnt_at_bemf = 0;
//     zc_period = context.last_period - context.last_elapsed_cnt_at_bemf +
//                 context.elapsed_cnt_at_bemf;
//     zc_period_filt = zc_period / 2 + zc_period_prev / 2;
//     half = (uint32_t)(coef_half * (float)zc_period_filt);
//     zc_period_prev = zc_period;
//     context.last_elapsed_cnt_at_bemf = context.elapsed_cnt_at_bemf;
//
//     // context.back_to_back = 0;
//     // err = open_loop_period_us / 2;
//     // open_loop_period_us -= err / KI;
//     // timer_update_period(&htim3, open_loop_period_us,
//     TIMER_UPDATE_IMMEDIATE);
//     // if ((__HAL_TIM_GET_COUNTER(&htim3) + err / KP) >= open_loop_period_us)
//     {
//     //   timer_set_counter(&htim3, open_loop_period_us - 1);
//     // } else {
//     //   timer_set_counter(&htim3, err / KP);
//     // }
//   } else if (state == BEMF_VALID) {
//     /* ERR CORRECTION */
//     // if (zc_cnt < 2) return;
//     if (zc_cnt < 200) {
//
//       zc_times[zc_cnt] = context.last_period -
//                          context.last_elapsed_cnt_at_bemf +
//                          context.elapsed_cnt_at_bemf;
//       // printf("bemfper: %d lastper: %d currper: %d\r\n", zc_times[zc_cnt],
//       // context.last_period, context.current_period);
//     }
//
//     // zc_period_n = context.last_period - context.last_elapsed_cnt_at_bemf +
//     //               context.elapsed_cnt_at_bemf;
//
//     // err = context.current_period / 2 - context.elapsed_cnt_at_bemf;
//     // open_loop_period_us = context.current_period - err / KI;
//     // timer_update_period(&htim3, open_loop_period_us,
//     TIMER_UPDATE_IMMEDIATE);
//     // if (err > 0) {
//     //   if ((context.elapsed_cnt_at_bemf + err / KP) >= open_loop_period_us)
//     {
//     //     timer_set_counter(&htim3, open_loop_period_us - 1);
//     //   } else {
//     //     // timer_set_counter(&htim3, context.elapsed_cnt_at_bemf + err /
//     KP);
//     //     timer_set_counter(&htim3, context.elapsed_cnt_at_bemf + err / KP);
//     //   }
//     // }
//   } else if (state == BEMF_OVERSHOOT) {
//     /* INCREASE PERIOD */
//
//     context.elapsed_cnt_at_bemf = context.current_period;
//     zc_period = context.last_period - context.last_elapsed_cnt_at_bemf +
//                 context.elapsed_cnt_at_bemf;
//     zc_period_filt = (zc_period >> 1) + (zc_period_prev >> 1);
//     // half = coef_half * zc_period_filt;
//     half = (uint32_t)(coef_half * (float)zc_period_filt);
//     zc_period_prev = zc_period;
//     context.last_elapsed_cnt_at_bemf = context.elapsed_cnt_at_bemf;
//     // err = open_loop_period_us / 2;
//     // open_loop_period_us += err / KI;
//     // timer_update_period(&htim3, open_loop_period_us,
//     TIMER_UPDATE_IMMEDIATE);
//     // if ((__HAL_TIM_GET_COUNTER(&htim3) + err / KP) >= open_loop_period_us)
//     {
//     //   timer_set_counter(&htim3, open_loop_period_us - 1);
//     // } else {
//     //   timer_set_counter(&htim3, err / KP);
//     // }
//   } else {
//     /*ERROR*/
//   }
//   prev_state = state;
// }

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM16 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_TIM_Base_Stop_IT(&htim15);

    __HAL_TIM_SET_COUNTER(&htim15, 0);

    htim15.Instance->CR1 |= TIM_CR1_OPM; // One Pulse Mode

    HAL_TIM_Base_Start_IT(&htim15);
  }
  if (htim->Instance == TIM15 && enable_bemf) {
    switch (current_motor_step) {
    case MOTOR_STEP_1: // A->B, C is floating ---> down
      // cmp3 = -1;
      // old_cmp1 = cmp1;
      // cmp1 = HAL_GPIO_ReadPin(CMP3_GPIO_Port, CMP3_Pin);
      //   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, cmp1);
      // if (cmp1 == GPIO_PIN_RESET && old_cmp1 == GPIO_PIN_SET &&
      //     safeguard == 0) {
      //   handle_zero_crossing(&htim3);
      //   // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
      // }
      cmp3 = -1;
      old_cmp1 = cmp1;
      cmp1 = HAL_GPIO_ReadPin(CMP1_GPIO_Port, CMP1_Pin);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, cmp1);
      if (cmp1 == GPIO_PIN_SET && old_cmp1 == GPIO_PIN_RESET &&
          safeguard == 0) {
        handle_zero_crossing(&htim3);
      }
      break;

    case MOTOR_STEP_2: // A->C, B is floating ----> up
      // cmp1 = -1;
      // old_cmp2 = cmp2;
      // cmp2 = HAL_GPIO_ReadPin(CMP1_GPIO_Port, CMP1_Pin);
      // if (cmp2 == GPIO_PIN_SET && old_cmp2 == GPIO_PIN_RESET &&
      //     safeguard == 0) {
      //   handle_zero_crossing(&htim3);
      // }
      cmp1 = -1;
      old_cmp2 = cmp2;
      cmp2 = HAL_GPIO_ReadPin(CMP2_GPIO_Port, CMP2_Pin);
      if (cmp2 == GPIO_PIN_RESET && old_cmp2 == GPIO_PIN_SET &&
          safeguard == 0) {
        handle_zero_crossing(&htim3);
      }
      break;

    case MOTOR_STEP_3: // B->C, A is floating -----> down
      // cmp2 = -1;
      // old_cmp3 = cmp3;
      // cmp3 = HAL_GPIO_ReadPin(CMP2_GPIO_Port, CMP2_Pin);
      // if (cmp3 == GPIO_PIN_RESET && old_cmp3 == GPIO_PIN_SET &&
      //     safeguard == 0) {
      //   handle_zero_crossing(&htim3);
      // }
      cmp2 = -1;
      old_cmp3 = cmp3;
      cmp3 = HAL_GPIO_ReadPin(CMP3_GPIO_Port, CMP3_Pin);
      if (cmp3 == GPIO_PIN_SET && old_cmp3 == GPIO_PIN_RESET &&
          safeguard == 0) {
        handle_zero_crossing(&htim3);
      }
      break;

    case MOTOR_STEP_4: // B->A, C is floating ----> up
      // cmp3 = -1;
      // old_cmp1 = cmp1;
      // cmp1 = HAL_GPIO_ReadPin(CMP3_GPIO_Port, CMP3_Pin);
      // if (cmp1 == GPIO_PIN_SET && old_cmp1 == GPIO_PIN_RESET &&
      //     safeguard == 0) {
      //   handle_zero_crossing(&htim3);
      //   // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
      // }
      cmp3 = -1;
      old_cmp1 = cmp1;
      cmp1 = HAL_GPIO_ReadPin(CMP1_GPIO_Port, CMP1_Pin);
      if (cmp1 == GPIO_PIN_RESET && old_cmp1 == GPIO_PIN_SET &&
          safeguard == 0) {
        handle_zero_crossing(&htim3);
        // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
      }
      break;

    case MOTOR_STEP_5: // C->A, B is floating -----> down
      // cmp1 = -1;
      // old_cmp2 = cmp2;
      // cmp2 = HAL_GPIO_ReadPin(CMP1_GPIO_Port, CMP1_Pin);
      // if (cmp2 == GPIO_PIN_RESET && old_cmp2 == GPIO_PIN_SET &&
      //     safeguard == 0) {
      //   handle_zero_crossing(&htim3);
      // }
      cmp1 = -1;
      old_cmp2 = cmp2;
      cmp2 = HAL_GPIO_ReadPin(CMP2_GPIO_Port, CMP2_Pin);
      if (cmp2 == GPIO_PIN_SET && old_cmp2 == GPIO_PIN_RESET &&
          safeguard == 0) {
        handle_zero_crossing(&htim3);
      }
      break;

    case MOTOR_STEP_6: // C->B, A is floating ----> up
      // cmp2 = -1;
      // old_cmp3 = cmp3;
      // cmp3 = HAL_GPIO_ReadPin(CMP2_GPIO_Port, CMP2_Pin);
      // if (cmp3 == GPIO_PIN_SET && old_cmp3 == GPIO_PIN_RESET &&
      //     safeguard == 0) {
      //   handle_zero_crossing(&htim3);
      // }
      cmp2 = -1;
      old_cmp3 = cmp3;
      cmp3 = HAL_GPIO_ReadPin(CMP3_GPIO_Port, CMP3_Pin);
      if (cmp3 == GPIO_PIN_RESET && old_cmp3 == GPIO_PIN_SET &&
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
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM3) {

    // commutate_flag = 1;

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
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
