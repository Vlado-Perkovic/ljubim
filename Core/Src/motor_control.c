#include "motor_control.h"
#include "cmsis_os.h" // For osDelay
#include "main.h"
#include "stm32g0xx.h"
#include "stm32g0xx_ll_tim.h"
#include "tim.h"
#include <stdint.h>
// Define the motor phases using the HAL TIM channel definitions
#define PHASE_A TIM_CHANNEL_1
#define PHASE_B TIM_CHANNEL_2
#define PHASE_C TIM_CHANNEL_3

// Initial duty cycle for open loop startup
#define STARTUP_DUTY_CYCLE 12

typedef uint32_t phase_channel_t;

volatile OneShotSource_t g_one_shot_source = SOURCE_NONE;
volatile GPIO_PinState cmp1, cmp2, cmp3 = GPIO_PIN_RESET;

static void set_pwm_duty_cycle(phase_channel_t phase, uint32_t per10k) {
  if (per10k > 10000) {
    per10k = 10000;
  }
  // The Period is the ARR value, so the max value is Period, not Period+1
  uint32_t duty_cycle = (htim1.Instance->ARR * per10k) / 10000;
  __HAL_TIM_SET_COMPARE(&htim1, phase, duty_cycle);
}

static void pwm_comp(phase_channel_t phase, uint32_t duty_cycle) {
  set_pwm_duty_cycle(phase, duty_cycle);
  // HAL_TIM_PWM_Start(&htim1, phase);
  // HAL_TIMEx_PWMN_Start(&htim1, phase);
  switch (phase) {

  case PHASE_A:
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    break;
  case PHASE_B:
    TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
    break;
  case PHASE_C:
    TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);
    break;
  }
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->CR1 |= TIM_CR1_CEN;
}

static void pwm_lo(phase_channel_t phase) { pwm_comp(phase, 0); }
static void pwm_hi(phase_channel_t phase) { pwm_comp(phase, 10000); }

static void pwm_off(phase_channel_t phase) {
  // HAL_TIM_PWM_Stop(&htim1, phase);
  // HAL_TIMEx_PWMN_Stop(&htim1, phase);
  switch (phase) {
  case PHASE_A: // Corresponds to TIM_CHANNEL_1
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    break;
  case PHASE_B: // Corresponds to TIM_CHANNEL_2
    TIM1->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);
    break;
  case PHASE_C: // Corresponds to TIM_CHANNEL_3
    TIM1->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);
    break;
  }
  TIM1->BDTR &= ~( TIM_BDTR_MOE );
  TIM1->CR1 &= ~( TIM_CR1_CEN );
    // __HAL_TIM_MOE_DISABLE(htim);
    // __HAL_TIM_DISABLE(htim);
}

void motor_init(void) {
  // Ensure all phases are off initially
  pwm_off(PHASE_A);
  pwm_off(PHASE_B);
  pwm_off(PHASE_C);

  // setup for step 5
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  // Start the master timer counter
  // HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim1); // for period elapsed
  // HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1); // for PWM pulse finished
}

// void motor_align(void) {
//   // Energize the rotor into a known position (step 5)
//   // osDelay is acceptable here as it's a one-time, non-real-time operation.
//   motor_step(MOTOR_STEP_5, STARTUP_DUTY_CYCLE);
//   osDelay(1000);
//   motor_step(MOTOR_STEP_6, STARTUP_DUTY_CYCLE);
//   osDelay(1000);
// }
// (0, 0, 1): 1,
// (1, 0, 1): 2,
// (1, 0, 0): 3,
// (1, 1, 0): 4,
// (0, 1, 0): 5,
// (0, 1, 1): 6
//
//
// ljubicasto h1
// plavi h2
// zuti h3

void motor_step(motor_step_t step, uint32_t duty_cycle) {
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  switch (step) {
  case MOTOR_STEP_1: // A->B, C is floating ---> down
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    pwm_off(PHASE_C);
    pwm_lo(PHASE_B);
    pwm_comp(PHASE_A, duty_cycle);
    break;

  case MOTOR_STEP_2: // A->C, B is floating ----> up
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    pwm_off(PHASE_B);
    pwm_lo(PHASE_C);
    pwm_comp(PHASE_A, duty_cycle);
    break;

  case MOTOR_STEP_3: // B->C, A is floating -----> down
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    pwm_off(PHASE_A);
    pwm_lo(PHASE_C);
    pwm_comp(PHASE_B, duty_cycle);
    break;

  case MOTOR_STEP_4: // B->A, C is floating ----> up
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    pwm_off(PHASE_C);
    pwm_lo(PHASE_A);
    pwm_comp(PHASE_B, duty_cycle);
    break;

  case MOTOR_STEP_5: // C->A, B is floating -----> down
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    pwm_off(PHASE_B);
    pwm_lo(PHASE_A);
    pwm_comp(PHASE_C, duty_cycle);
    break;

  case MOTOR_STEP_6: // C->B, A is floating ----> up
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    pwm_off(PHASE_A);
    pwm_lo(PHASE_B);
    pwm_comp(PHASE_C, duty_cycle);
    break;

  case MOTOR_STOP:
  default:
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    pwm_off(PHASE_A);
    pwm_off(PHASE_B);
    pwm_off(PHASE_C);
    break;
  }
}
/**
 * @brief  Sets the commutation timer period in microseconds.
 * @param  period_us The desired period between commutations in microseconds.
 * @note   This function assumes htim3 is clocked at 64MHz with a prescaler of
 * 63, resulting in a 1MHz counter clock (1 tick = 1us). The timer's ARR
 * register is 16-bit, so the max period is 65535 us.
 */
void set_commutation_period_us_63(uint32_t period_us) {
  // Ensure the period is within the valid range for a 16-bit timer (0-65535)
  if (period_us > 65535) {
    period_us = 65535;
  }

  // A very short period can be unstable. Enforce a practical minimum.
  // For example, 50us corresponds to a commutation frequency of 20kHz.
  const uint32_t MINIMUM_PERIOD_US = 50;
  if (period_us < MINIMUM_PERIOD_US) {
    period_us = MINIMUM_PERIOD_US;
  }

  // With a 1MHz counter clock, the ARR value is simply the period in us
  // minus 1.
  uint32_t new_arr_value = period_us - 1;

  // Set the new Auto-Reload Register value for TIM3.
  // The change will take effect at the next update event (counter overflow).
  __HAL_TIM_SET_AUTORELOAD(&htim3, new_arr_value);
}
/**
 * @brief Sets the commutation timer period.
 * @note This function is configured for a timer with PSC = 127 and a 64MHz
 * clock, resulting in a 2 microsecond tick time (64MHz / (127+1) = 0.5MHz).
 * @param period_us The desired period in microseconds.
 */
void set_commutation_period_us_255(uint32_t period_us) {
  // With a 2us tick, the max period is 65536 * 2us = 131,072 us.
  const uint32_t MAX_PERIOD_US = 131072 * 2;
  if (period_us > MAX_PERIOD_US) {
    period_us = MAX_PERIOD_US;
  }

  // A very short period can be unstable. Enforce a practical minimum.
  const uint32_t MINIMUM_PERIOD_US = 50; // e.g., 50us (20kHz commutation)
  if (period_us < MINIMUM_PERIOD_US) {
    period_us = MINIMUM_PERIOD_US;
  }

  // Each timer tick is 2us, so we need half the number of ticks as the period
  // in us. ARR = (Number of Ticks) - 1
  uint32_t new_arr_value = (period_us / 4) - 1;

  // Set the new Auto-Reload Register value for TIM3.
  __HAL_TIM_SET_AUTORELOAD(&htim3, new_arr_value);
}

/**
 * @brief Sets the commutation period for TIM3 in microseconds.
 * @note This function is robust and adapts to changes in the timer's prescaler.
 * It dynamically calculates the required ARR value based on the timer's
 * input clock frequency and its current PSC setting. It is specifically
 * adapted for the STM32G0 series.
 * @param period_us The desired commutation period in microseconds.
 */
void set_commutation_period_us(uint32_t period_us) {
  // --- 1. Get the timer's input clock frequency for STM32G0 ---
  // On STM32G0, timers are on the APB1 bus. Their clock is doubled if the APB
  // prescaler is not 1.
  uint32_t pclk1_freq = HAL_RCC_GetPCLK1Freq();
  uint32_t timer_clk_freq = pclk1_freq;

  // FIX: Use the correct register definitions for STM32G0 series.
  // It has a single APB prescaler field named 'PPRE' and the DIV1 value is
  // named differently.
  if ((RCC->CFGR & RCC_CFGR_PPRE) != RCC_CFGR_PPRE_1) {
    timer_clk_freq *= 2;
  }

  // --- 2. Get the timer's current prescaler value ---
  uint32_t prescaler = htim3.Instance->PSC;

  // --- 3. Calculate dynamic constraints based on the current configuration ---
  const uint32_t MAX_ARR = 0xFFFF; // Max ARR for a 16-bit timer is 65535

  // Calculate the maximum period this timer can achieve with its current
  // prescaler. max_period_us = (MAX_ARR + 1) * (PSC + 1) * 1,000,000 /
  // timer_clk_freq
  uint64_t max_period_us_64 =
      (uint64_t)(MAX_ARR + 1) * (prescaler + 1) * 1000000 / timer_clk_freq;
  uint32_t max_period_us =
      (max_period_us_64 > 0xFFFFFFFF) ? 0xFFFFFFFF : (uint32_t)max_period_us_64;

  // Clamp the requested period to the timer's capabilities and a safe minimum.
  if (period_us > max_period_us) {
    period_us = max_period_us;
  }
  const uint32_t MINIMUM_PERIOD_US =
      50; // A practical minimum (e.g., 20kHz commutation)
  if (period_us < MINIMUM_PERIOD_US) {
    period_us = MINIMUM_PERIOD_US;
  }

  // --- 4. Calculate the new ARR value ---
  // Formula: ARR = (period_us * Timer_Clock_Hz) / (1,000,000 * (PSC + 1)) - 1
  // We use 64-bit integers to prevent overflow during calculation.
  uint64_t new_arr_64 = (uint64_t)period_us * timer_clk_freq;
  new_arr_64 /= 1000000;
  new_arr_64 /= (prescaler + 1);

  uint32_t new_arr_value = 0;
  if (new_arr_64 > 0) {
    new_arr_value = (uint32_t)(new_arr_64 - 1);
  }

  // Final check to ensure the value fits within the 16-bit register.
  if (new_arr_value > MAX_ARR) {
    new_arr_value = MAX_ARR;
  }

  // --- 5. Set the new Auto-Reload Register value for TIM3 ---
  __HAL_TIM_SET_AUTORELOAD(&htim3, new_arr_value);
}
