/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "stm32g0xx_hal_gpio.h"
#include "stm32g0xx_hal_tim.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "comp.h"
#include "motor_control.h"
#include "tim.h"
#include "timer_utils.h"
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PERIOD 200
#define DUTY_CYCLE 10.5f
#define KI 20
#define KP 2
#define ZC_CNT_MIN 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
const uint32_t step_times_us[] = {
    34500, 14291, 10966, 9244, 8144, 7363, 6771, 6302, 5919, 5599, 5325, 5088,
    4880,  4696,  4531,  4382, 4247, 4124, 4011, 3907, 3810, 3721, 3637, 3559,
    3485,  3416,  3351,  3290, 3231, 3176, 3124, 3074, 3026, 2980, 2937, 2895,
    2855,  2817,  2780,  2745, 2711, 2678, 2646, 2616, 2586, 2557, 2530, 2503,
    2477,  2452,  2427,  2404, 2381, 2358, 2337, 2316, 2295, 2275, 2255, 2236,
    2218,  2200,  2182,  2165, 2148, 2131, 2115, 2100, 2084, 2069, 2054, 2040,
    2026,  2012,  1999,  1985, 1972, 1960, 1947, 1935, 1923, 1911, 1899, 1888,
    1877,  1866,  1855,  1844, 1834, 1823, 1813, 1803, 1794, 1784, 1775, 1765,
    1756,  1747,  1738,  1729, 1721, 1712, 1704, 1696, 1687, 1679, 1672, 1664,
    1656,  1648,  1641,  1634, 1626, 1619, 1612, 1605, 1598, 1591, 1585, 1578,
    1571,  1565,  1559,  1552, 1546, 1540, 1534, 1528, 1522, 1516, 1510, 1504,
    1499,  1493,  1487,  1482, 1476, 1471, 1466, 1461, 1455, 1450, 1445, 1440,
    1435,  1430,  1425,  1420, 1416, 1411, 1406, 1401, 1397, 1392, 1388, 1383,
    1379,  1375,  1370,  1366, 1362, 1357, 1353, 1349, 1345, 1341, 1337, 1333,
    1329,  1325,  1321,  1317, 1313, 1310, 1306, 1302, 1298, 1295, 1291, 1288,
    1284,  1280,  1277,  1273, 1270, 1267, 1263, 1260, 1256, 1253, 1250, 1247,
    1243,  1240,  1237,  1234, 1231, 1227, 1224, 1221, 1218, 1215, 1212, 1209,
    1206,  1203,  1200,  1198, 1195, 1192, 1189, 1186, 1183, 1181, 1178, 1175,
    1172,  1170,  1167,  1164, 1162, 1159, 1156, 1154, 1151, 1149, 1146, 1144,
    1141,  1139,  1136,  1134, 1131, 1129, 1126, 1124, 1122, 1119, 1117, 1115,
    1112,  1110,  1108,  1105, 1103, 1101, 1099, 1096, 1094, 1092, 1090, 1088,
    1086,  1083,  1081,  1079, 1077, 1075, 1073, 1071, 1069, 1067, 1065, 1063,
    1061,  1059,  1057,  1055, 1053, 1051, 1049, 1047, 1045, 1043, 1041, 1039,
    1037,  1036,  1034,  1032, 1030, 1028, 1026, 1025, 1023, 1021, 1019, 1017,
    1016,  1014,  1012,  1010, 1009, 1007, 1005, 1003, 1002, 1000, 998,  997,
    995,   993,   992,   990,  989,  987,  985,  984,  982,  981,  979,  977,
    976,   974,   973,   971,  970,  968,  967,  965,  964,  962,  961,  959,
    958,   956,   955,   953,  952,  950,  949,  947,  946,  945,  943,  942,
};
extern volatile GPIO_PinState cmp1, cmp2, cmp3;
extern volatile uint8_t safeguard;
extern void handle_zero_crossing(TIM_HandleTypeDef *htim);
extern volatile uint32_t zc_cnt;

uint32_t open_loop_period_us = ALIGN_TIMEOUT_US;
volatile motor_step_t current_motor_step = MOTOR_STEP_4;
volatile uint8_t go = 0;
volatile bemf_case_t state = BEMF_ERROR;
volatile bemf_case_t prev_state = BEMF_ERROR;
volatile ctx_t context = {.elapsed_cnt_at_bemf = 0,
                          .last_elapsed_cnt_at_bemf = 0,
                          .current_period = 0,
                          .step_periods = {0},
                          .last_steps = {.queue = {0}},
                          .last_period = 0};
uint32_t zc_times[200] = {0};
extern volatile uint8_t zc_flag;

volatile uint32_t zc_period = 0;
volatile uint32_t zc_period_filt = 0;
volatile uint32_t zc_period_prev = 0;
volatile uint32_t half = 0;
// float coef_half = 0.375f; // 7.5deg advance
float coef_half = 0.3f; // 12deg advance
float duty_cycle = DUTY_CYCLE;
// volatile float coef_half = 0.35f;
volatile uint8_t enable_bemf = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t motor_control_hndl;
const osThreadAttr_t motor_control_attributes = {
    .name = "motor control",
    .priority = (osPriority_t)osPriorityRealtime,
    .stack_size = 256 * 4};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 4};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void motor_control_task(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  motor_control_hndl =
      osThreadNew(motor_control_task, NULL, &motor_control_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void motor_handle_bemf() {
  int32_t err;

  if (state == BEMF_UNDERSHOOT) {
    /* DECREASE PERIOD */
    context.elapsed_cnt_at_bemf = (context.current_period >> 4);
    zc_period = context.last_period - context.last_elapsed_cnt_at_bemf +
                context.elapsed_cnt_at_bemf;
    queue_put(&context.last_steps, zc_period);
    // zc_period_filt = zc_period / 2 + zc_period_prev / 2;
    // zc_period_filt = (zc_period >> 2) + 3*(zc_period_prev >> 2);
    // zc_period_filt = (zc_period >> 1) + (zc_period_prev >> 1);
    zc_period_filt = 3*(context.last_steps.queue[0] >> 2) +
                     (context.last_steps.queue[1] >> 2);
    // zc_period_filt = (context.last_steps.queue[0] >> 2) +
    //                  (context.last_steps.queue[1] >> 2) +
    //                  (context.last_steps.queue[2] >> 2) +
    //                  (context.last_steps.queue[3] >> 2);
    // zc_period_filt *= 0.8;
    half = (uint32_t)(coef_half * (float)zc_period_filt);
    zc_period_prev = zc_period;
    context.last_elapsed_cnt_at_bemf = context.elapsed_cnt_at_bemf;

    // context.back_to_back = 0;
    // err = open_loop_period_us / 2;
    // open_loop_period_us -= err / KI;
    // timer_update_period(&htim3, open_loop_period_us, TIMER_UPDATE_IMMEDIATE);
    // if ((__HAL_TIM_GET_COUNTER(&htim3) + err / KP) >= open_loop_period_us) {
    //   timer_set_counter(&htim3, open_loop_period_us - 1);
    // } else {
    //   timer_set_counter(&htim3, err / KP);
    // }
  } else if (state == BEMF_VALID) {
    /* ERR CORRECTION */
    // if (zc_cnt < 2) return;
    if (zc_cnt < 200) {

      zc_times[zc_cnt] = context.last_period -
                         context.last_elapsed_cnt_at_bemf +
                         context.elapsed_cnt_at_bemf;
      // printf("bemfper: %d lastper: %d currper: %d\r\n", zc_times[zc_cnt],
      // context.last_period, context.current_period);
    }

    // zc_period_n = context.last_period - context.last_elapsed_cnt_at_bemf +
    //               context.elapsed_cnt_at_bemf;

    // err = context.current_period / 2 - context.elapsed_cnt_at_bemf;
    // open_loop_period_us = context.current_period - err / KI;
    // timer_update_period(&htim3, open_loop_period_us, TIMER_UPDATE_IMMEDIATE);
    // if (err > 0) {
    //   if ((context.elapsed_cnt_at_bemf + err / KP) >= open_loop_period_us) {
    //     timer_set_counter(&htim3, open_loop_period_us - 1);
    //   } else {
    //     // timer_set_counter(&htim3, context.elapsed_cnt_at_bemf + err / KP);
    //     timer_set_counter(&htim3, context.elapsed_cnt_at_bemf + err / KP);
    //   }
    // }
  } else if (state == BEMF_OVERSHOOT) {
    /* INCREASE PERIOD */

    context.elapsed_cnt_at_bemf =
        context.current_period - (context.current_period >> 4);
    zc_period = context.last_period - context.last_elapsed_cnt_at_bemf +
                context.elapsed_cnt_at_bemf;
    queue_put(&context.last_steps, zc_period);
    // zc_period_filt = (zc_period >> 1) + (zc_period_prev >> 1);
    // zc_period_filt = (context.last_steps.queue[0] >> 2) +
    //                  (context.last_steps.queue[1] >> 2) +
    //                  (context.last_steps.queue[2] >> 2) +
    //                  (context.last_steps.queue[3] >> 2);
    zc_period_filt = 3*(context.last_steps.queue[0] >> 2) +
                     (context.last_steps.queue[1] >> 2);
    // zc_period_filt = (zc_period >> 2) + 3*(zc_period_prev >> 2);
    // zc_period_filt = 3*(zc_period >> 2) + (zc_period_prev >> 2);
    // half = coef_half * zc_period_filt;
    half = (uint32_t)(coef_half * (float)zc_period_filt);
    zc_period_prev = zc_period;
    context.last_elapsed_cnt_at_bemf = context.elapsed_cnt_at_bemf;
    // err = open_loop_period_us / 2;
    // open_loop_period_us += err / KI;
    // timer_update_period(&htim3, open_loop_period_us, TIMER_UPDATE_IMMEDIATE);
    // if ((__HAL_TIM_GET_COUNTER(&htim3) + err / KP) >= open_loop_period_us) {
    //   timer_set_counter(&htim3, open_loop_period_us - 1);
    // } else {
    //   timer_set_counter(&htim3, err / KP);
    // }
  } else {
    /*ERROR*/
  }
  prev_state = state;
}
void motor_control_task(void *argument) {
  uint32_t step_counter = 0;
  motor_init();

  set_commutation_period_us_255(open_loop_period_us);
  HAL_TIM_Base_Start_IT(&htim3);
  // HAL_COMP_Start(&hcomp1);
  int cnt = 0;
  uint32_t err;

  for (;;) {
    // Wait here indefinitely for a notification from the timer ISR
    if (zc_flag == 1) {
      zc_flag = 0;
      zc_period = context.last_period - context.last_elapsed_cnt_at_bemf +
                  context.elapsed_cnt_at_bemf;
      queue_put(&context.last_steps, zc_period);

      // zc_period_filt = zc_period/2 + zc_period_prev/2;
      // zc_period_filt = (zc_period >> 1) + (zc_period_prev >> 1);
      zc_period_filt = (context.last_steps.queue[0] >> 2) +
                       (context.last_steps.queue[1] >> 2) +
                       (context.last_steps.queue[2] >> 2) +
                       (context.last_steps.queue[3] >> 2);
    // zc_period_filt = (context.last_steps.queue[0] >> 1) +
    //                  (context.last_steps.queue[1] >> 1);
      // zc_period_filt = 3*(zc_period >> 2) + (zc_period_prev >> 2);

      half = (uint32_t)(coef_half * (float)zc_period_filt);

      if (zc_cnt >= ZC_CNT_MIN) {
        context.current_period = context.elapsed_cnt_at_bemf + half;
        __HAL_TIM_SET_AUTORELOAD(&htim3, context.current_period);
      }
      // printf("per: %ld prev: %ld filt: %ld \r\n", (long)zc_period,
      // (long)zc_period_prev, (long)zc_period_filt);
      zc_period_prev = zc_period;
    }

    if (ulTaskNotifyTake(pdTRUE, 0)) {
      // if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {

      // #TODO: safeguard == is_bemf_detected
      if (go) {

        switch (current_motor_step) {
        case MOTOR_STEP_1: // A->B, C is floating ---> down

          /* because of the inverse comparator outputs
           * as the C is floating down:
           * UNDERSHOOT (change happened before the window)-> !LOW -> HIGH
           * OVERSHOOT (change happened after the window)-> !HIGH -> LOW
           * ERROR (NEXT BEMF TRIGGER HAPPENED BEFORE THIS POINT)
           */
          if (state != BEMF_VALID) {
            if (cmp1 == GPIO_PIN_SET)
              state = BEMF_UNDERSHOOT;
            else if (cmp1 == GPIO_PIN_RESET)
              state = BEMF_OVERSHOOT;
            else
              state = BEMF_ERROR;
          }

          motor_handle_bemf();
          break;

        case MOTOR_STEP_2: // A->C, B is floating ----> up
          /* because of the inverse comparator outputs
           * as the B is floating up:
           * UNDERSHOOT (change happened before the window)-> !HIGH -> LOW
           * OVERSHOOT (change happened after the window)-> !LOW -> HIGH
           * ERROR (NEXT BEMF TRIGGER HAPPENED BEFORE THIS POINT)
           */
          if (state != BEMF_VALID) {
            if (cmp2 == GPIO_PIN_RESET)
              state = BEMF_UNDERSHOOT;
            else if (cmp2 == GPIO_PIN_SET)
              state = BEMF_OVERSHOOT;
            else
              state = BEMF_ERROR;
          }
          motor_handle_bemf();
          break;

        case MOTOR_STEP_3: // B->C, A is floating -----> down
          /* because of the inverse comparator outputs
           * as the A is floating DOWN:
           * UNDERSHOOT (change happened before the window)-> !LOW -> HIGH
           * OVERSHOOT (change happened after the window)-> !HIGH -> LOW
           * ERROR (NEXT BEMF TRIGGER HAPPENED BEFORE THIS POINT)
           */
          if (state != BEMF_VALID) {
            if (cmp3 == GPIO_PIN_SET)
              state = BEMF_UNDERSHOOT;
            else if (cmp3 == GPIO_PIN_RESET)
              state = BEMF_OVERSHOOT;
            else
              state = BEMF_ERROR;
          }
          motor_handle_bemf();
          break;

        case MOTOR_STEP_4: // B->A, C is floating ----> up
          /* because of the inverse comparator outputs
           * as the C is floating up:
           * UNDERSHOOT (change happened before the window)-> !HIGH -> LOW
           * OVERSHOOT (change happened after the window)-> !LOW -> HIGH
           * ERROR (NEXT BEMF TRIGGER HAPPENED BEFORE THIS POINT)
           */
          if (state != BEMF_VALID) {
            if (cmp1 == GPIO_PIN_RESET)
              state = BEMF_UNDERSHOOT;
            else if (cmp1 == GPIO_PIN_SET)
              state = BEMF_OVERSHOOT;
            else
              state = BEMF_ERROR;
          }
          motor_handle_bemf();
          break;

        case MOTOR_STEP_5: // C->A, B is floating -----> down
          /* because of the inverse comparator outputs
           * as the B is floating DOWN:
           * UNDERSHOOT (change happened before the window)-> !LOW -> HIGH
           * OVERSHOOT (change happened after the window)-> !HIGH -> LOW
           * ERROR (NEXT BEMF TRIGGER HAPPENED BEFORE THIS POINT)
           */
          if (state != BEMF_VALID) {
            if (cmp2 == GPIO_PIN_SET)
              state = BEMF_UNDERSHOOT;
            else if (cmp2 == GPIO_PIN_RESET)
              state = BEMF_OVERSHOOT;
            else
              state = BEMF_ERROR;
          }
          motor_handle_bemf();
          break;

        case MOTOR_STEP_6: // C->B, A is floating ----> up
          /* because of the inverse comparator outputs
           * as the A is floating up:
           * UNDERSHOOT (change happened before the window)-> !HIGH -> LOW
           * OVERSHOOT (change happened after the window)-> !LOW -> HIGH
           * ERROR (NEXT BEMF TRIGGER HAPPENED BEFORE THIS POINT)
           */
          if (state != BEMF_VALID) {
            if (cmp3 == GPIO_PIN_RESET)
              state = BEMF_UNDERSHOOT;
            else if (cmp3 == GPIO_PIN_SET)
              state = BEMF_OVERSHOOT;
            else
              state = BEMF_ERROR;
          }
          motor_handle_bemf();
          break;

        case MOTOR_STOP:
        default:
          break;
        }
      }
      state = BEMF_ERROR;
      safeguard = 0;
      current_motor_step =
          (motor_step_t)((current_motor_step + 1) % MOTOR_STEP_COUNT);
      motor_step(current_motor_step, duty_cycle);
      step_counter++;

      if (step_counter < 2)
        continue;
      if (step_counter == 2) {
        // timer_update_prescaler(&htim3, 31, TIMER_UPDATE_IMMEDIATE);

        __HAL_TIM_SET_PRESCALER(&htim3, 31);
        continue;
      }

      if (step_counter == 3) {
        open_loop_period_us = 2 * step_times_us[step_counter - 3];
        set_commutation_period_us_63(open_loop_period_us);
        // __HAL_TIM_SET_AUTORELOAD(&htim3, open_loop_period_us);
        __HAL_TIM_SET_PRESCALER(&htim3, 15);
        continue;
      }
      if (step_counter == 4) {
        open_loop_period_us = 4 * step_times_us[step_counter - 3];
        set_commutation_period_us_63(open_loop_period_us);
        // timer_update_prescaler(&htim3, 15, TIMER_UPDATE_IMMEDIATE);
        // __HAL_TIM_SET_AUTORELOAD(&htim3, open_loop_period_us);
        continue;
      }
      // if (step_counter == 8) {
      //   open_loop_period_us = 8 * step_times_us[step_counter - 3];
      //   set_commutation_period_us_63(open_loop_period_us);
      //   timer_update_prescaler(&htim3, 7, TIMER_UPDATE_IMMEDIATE);
      //   continue;
      // }
      if (step_counter == 13)
        enable_bemf = 1;

      // if (open_loop_period_us > 3300 && go == 0) {
      if (zc_cnt < ZC_CNT_MIN && go == 0) {

        open_loop_period_us = 4 * step_times_us[step_counter - 3];
        set_commutation_period_us_63(open_loop_period_us);
      } else {
        go = 1;
        open_loop_period_us =
            (open_loop_period_us > 12000) ? 12000 : zc_period_filt;
        context.last_period = context.current_period;
        context.current_period = open_loop_period_us;

        __HAL_TIM_SET_AUTORELOAD(&htim3, open_loop_period_us);
      }

      // if (step_counter > 2000 && step_counter < 3000 && step_counter % 2 ==
      // 0)
      // if (step_counter > 2000 && step_counter < 3000)
      //   duty_cycle += 0.005;
      if (step_counter == 2000) {
        duty_cycle += 2;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
      }
      if (step_counter == 3000) {
        duty_cycle += 1;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
      }
      if (step_counter == 4000) {
        duty_cycle += 1;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
      }
      // if (step_counter == 4000) duty_cycle += 1;
    }
  }
}
/* USER CODE END Application */
