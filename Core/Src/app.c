/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "comp.h"
#include "motor_control.h"
#include "pid.h"
#include "stm32g071xx.h"
#include "stm32g0xx_hal_gpio.h"
#include "stm32g0xx_hal_tim.h"
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DUTY_CYCLE 1300
// #define KI 0.01f
#define KI 0.006f
#define KP 4
#define ZC_CNT_MIN 3
#define COEF_HALF 0.375f
#define DUTY_MIN 1300
#define DUTY_MAX 7000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
const uint32_t step_times_us[] = {
    34500, 14291, 10966, 9244, 8144, 7363, 6771, 6302, 5919, 5599, 5325,
    5088, 4880,  4696,  4531,  4382, 4247, 4124, 4011, 3907, 3810, 3721,
    3637, 3559, 3485,  3416,  3351,  3290, 3231, 3176, 3124, 3074, 3026,
    2980, 2937, 2895, 2855,  2817,  2780,  2745, 2711, 2678, 2646, 2616,
    2586, 2557, 2530, 2503, 2477,  2452,  2427,  2404, 2381, 2358, 2337,
};
extern volatile GPIO_PinState cmp1, cmp2, cmp3;
extern volatile uint8_t safeguard;
extern volatile uint8_t pwm_cnt;
extern void handle_zero_crossing(TIM_HandleTypeDef *htim);
extern volatile uint32_t zc_cnt;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim17;

extern UART_HandleTypeDef huart2;
extern uint8_t rx_buf[2];
extern uint8_t tx_buf[2];

uint32_t open_loop_period_us = 65000;
volatile motor_step_t current_motor_step = MOTOR_STEP_4;
volatile uint8_t go = 0;
volatile uint8_t running = 0;
volatile bemf_case_t state = BEMF_ERROR;
volatile bemf_case_t prev_state = BEMF_ERROR;
volatile ctx_t context = {.elapsed_cnt_at_bemf = 0,
                          .last_elapsed_cnt_at_bemf = 0,
                          .current_period = 0,
                          .step_periods = {0},
                          .last_steps = {.queue = {0}},
                          .last_period = 0};
uint32_t zc_times[200] = {0};
uint32_t duty_cycles[200] = {0};
uint32_t speeds[1000] = {0};
uint32_t dc_cnt = 0, cs = 0;

extern volatile uint8_t zc_flag;
extern volatile uint8_t undershoot_flag;
extern volatile uint8_t commutate_flag;
extern volatile uint8_t PI_flag;

volatile uint32_t zc_period = 0;
volatile uint32_t zc_period_filt = 0;
volatile uint32_t zc_period_prev = 0;
volatile uint32_t half = 0;
float coef_half = COEF_HALF;
uint32_t duty_cycle = DUTY_CYCLE;
uint8_t multiplier = 1;

volatile uint8_t enable_bemf = 0;
volatile uint8_t closed_loop = 0;
volatile uint8_t scan = 0;

static PIDController spid;
volatile uint16_t current_speed;
volatile uint16_t target_speed = 600;

const uint32_t RPM_CONSTANT = 11428571;
uint32_t current_com_period = 0;
/* USER CODE END PM */

void motor_control_task(/*void *argument*/);
void transmit_current_speed(uint16_t speed) {
  tx_buf[0] = speed & 0xFF;
  tx_buf[1] = (speed >> 8) & 0xFF;

  HAL_UART_Transmit_DMA(&huart2, tx_buf, 2);
}
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void motor_handle_bemf() {

  if (state == BEMF_OVERSHOOT) {
    /* INCREASE PERIOD */

    context.elapsed_cnt_at_bemf =
        context.current_period - (context.current_period >> 4);
    zc_period = context.last_period - context.last_elapsed_cnt_at_bemf +
                context.elapsed_cnt_at_bemf;
    queue_put(&context.last_steps, zc_period);
    zc_period_filt =
        (context.last_steps.queue[0] >> 2) + (context.last_steps.queue[1] >> 2);
    current_com_period =
        (context.last_steps.queue[0]) + (context.last_steps.queue[1]) +
        (context.last_steps.queue[2]) + (context.last_steps.queue[3]) +
        (context.last_steps.queue[4]) + (context.last_steps.queue[5]);
    current_com_period /= 6;
    current_speed = (uint32_t)(RPM_CONSTANT / current_com_period);
    zc_period_prev = zc_period;
    context.last_elapsed_cnt_at_bemf = context.elapsed_cnt_at_bemf;
  } else {
    /*ERROR*/
  }
  prev_state = state;
}
void motor_control_task(/*void *argument*/) {
  uint32_t step_counter = 0;
  motor_init();

  // Set the new Auto-Reload Register value for TIM3.
  __HAL_TIM_SET_AUTORELOAD(&htim3, open_loop_period_us);
  HAL_TIM_Base_Start_IT(&htim3);
  int32_t Kp_Q12 = (int32_t)(KP * (1 << Q));
  int32_t Ki_Q12 = (int32_t)(KI * (1 << Q));
  int32_t Kd_Q12 = 0;

  PID_init(&spid, Kp_Q12, Ki_Q12, Kd_Q12, DUTY_MAX, DUTY_MIN);
  HAL_TIM_Base_Start_IT(&htim17);
  HAL_UART_Receive_DMA(&huart2, rx_buf, 2); // expecting 2 bytes

  for (;;) {
    if (undershoot_flag == 1) {
      undershoot_flag = 0;
      zc_period = context.last_period - context.last_elapsed_cnt_at_bemf +
                  context.elapsed_cnt_at_bemf;
      queue_put(&context.last_steps, zc_period);

      zc_period_filt = (context.last_steps.queue[0] >> 1) +
                       (context.last_steps.queue[1] >> 1);
      current_speed = RPM_CONSTANT / zc_period_filt;

      half = (uint32_t)(coef_half * (float)zc_period_filt);

      context.current_period = context.elapsed_cnt_at_bemf + half;
      __HAL_TIM_SET_AUTORELOAD(&htim3, context.current_period);
      zc_period_prev = zc_period;
    }

    if (zc_flag == 1) {
      zc_flag = 0;
      zc_period = context.last_period - context.last_elapsed_cnt_at_bemf +
                  context.elapsed_cnt_at_bemf;
      queue_put(&context.last_steps, zc_period);
      zc_period_filt = (context.last_steps.queue[0] >> 1) +
                       (context.last_steps.queue[1] >> 1);
      current_com_period =
          (context.last_steps.queue[0]) + (context.last_steps.queue[1]) +
          (context.last_steps.queue[2]) + (context.last_steps.queue[3]) +
          (context.last_steps.queue[4]) + (context.last_steps.queue[5]);
      current_com_period /= 6;

      current_speed = RPM_CONSTANT / current_com_period;

      half = (uint32_t)(coef_half * (float)zc_period_filt);

      if (zc_cnt >= ZC_CNT_MIN) {
        context.current_period = context.elapsed_cnt_at_bemf + half;
        __HAL_TIM_SET_AUTORELOAD(&htim3, context.current_period);
      }
      zc_period_prev = zc_period;
    }

    if (PI_flag) {
      if (running) {
        duty_cycle = PID_calculate(&spid, target_speed, current_speed);

        switch (current_motor_step) {

        case MOTOR_STEP_1:
          set_pwm_duty_cycle(PHASE_A, duty_cycle);
          break;
        case MOTOR_STEP_2:
          set_pwm_duty_cycle(PHASE_A, duty_cycle);
          break;
        case MOTOR_STEP_3:
          set_pwm_duty_cycle(PHASE_B, duty_cycle);
          break;
        case MOTOR_STEP_4:
          set_pwm_duty_cycle(PHASE_B, duty_cycle);
          break;
        case MOTOR_STEP_5:
          set_pwm_duty_cycle(PHASE_C, duty_cycle);
          break;
        case MOTOR_STEP_6:
          set_pwm_duty_cycle(PHASE_C, duty_cycle);
          break;
        default:
          break;
        }
      }
    }
    if (commutate_flag) {
      commutate_flag = 0;
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
      // state = BEMF_ERROR;
      // safeguard = 0;
      // pwm_cnt = 0;
      current_motor_step =
          (motor_step_t)((current_motor_step + 1) % MOTOR_STEP_COUNT);
      motor_step(current_motor_step, duty_cycle);
      state = BEMF_ERROR;
      safeguard = 0;
      pwm_cnt = 0;
      if (step_counter > 2 << 20) {
        step_counter = 100;
      }
      step_counter++;

      if (step_counter < 2)
        continue;
      if (step_counter == 2) {
        __HAL_TIM_SET_PRESCALER(&htim3, 31);
        continue;
      }

      if (step_counter == 3) {
        multiplier = 2;
        __HAL_TIM_SET_PRESCALER(&htim3, 15);
      }
      if (step_counter == 4) {
        multiplier = 4;
        __HAL_TIM_SET_PRESCALER(&htim3, 7);
      }
      if (step_counter == 5) {
        multiplier = 8;
      }
      if (step_counter == 13)
        enable_bemf = 1;

      context.last_period = context.current_period;

      if (zc_cnt < ZC_CNT_MIN && go == 0) {

        context.current_period = multiplier * step_times_us[step_counter - 3];
        if (context.current_period > 65535) {
          context.current_period = 65535;
        }

        __HAL_TIM_SET_AUTORELOAD(&htim3, context.current_period);
      } else {
        go = 1;
        context.current_period = (zc_period_filt > multiplier * 4000)
                                     ? multiplier * 4000
                                     : 2 * zc_period_filt;

        __HAL_TIM_SET_AUTORELOAD(&htim3, context.current_period);
      }

      if (zc_cnt > 400) {
        running = 1;
      }

      if (step_counter % 420 == 0) {
        transmit_current_speed(current_speed);
      }
    }
  }
}
/* USER CODE END Application */
