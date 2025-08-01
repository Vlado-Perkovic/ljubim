#include "timer_utils.h"
#include "stm32g0xx_hal_tim.h"

/* Exported functions --------------------------------------------------------*/

int timer_start(TIM_HandleTypeDef* timer) {
    if (!timer) return TIMER_ERROR_INVALID_PARAM;
    timer->Instance->CR1 |= TIM_CR1_CEN;
    return TIMER_OK;
}

int timer_stop(TIM_HandleTypeDef* timer) {
    if (!timer) return TIMER_ERROR_INVALID_PARAM;
    timer->Instance->CR1 &= ~TIM_CR1_CEN;
    return TIMER_OK;
}

int timer_update_prescaler(TIM_HandleTypeDef* timer, uint16_t prescaler, timer_update_mode_t mode) {
    if (!timer) return TIMER_ERROR_INVALID_PARAM;

    

    // Set the new prescaler value. The PSC register is always buffered and
    // is updated at the next update event.
    timer->Instance->PSC = prescaler;

    if (mode == TIMER_UPDATE_IMMEDIATE) {
        // For an immediate update, we manually generate an Update Event (UG bit)
        // to force the new prescaler value to be loaded from its buffer.
        timer->Instance->EGR = TIM_EGR_UG;
    }
    // For TIMER_UPDATE_SAFE, we do nothing extra. The value will be loaded
    // automatically at the next overflow/underflow event.

    return TIMER_OK;
}

int timer_update_period(TIM_HandleTypeDef* timer, uint16_t period, timer_update_mode_t mode) {
    if (!timer) return TIMER_ERROR_INVALID_PARAM;

    if (mode == TIMER_UPDATE_SAFE) {
        // In safe mode, we enable the Auto-Reload Preload (ARPE bit).
        // This means the new ARR value is written to a preload register and
        // is only transferred to the active shadow register at the next update event.
        timer->Instance->CR1 |= TIM_CR1_ARPE;
        timer->Instance->ARR = period;
    } else { // TIMER_UPDATE_IMMEDIATE
        // In immediate mode, we disable the ARPE bit. This causes the new value
        // to be written directly to the active register.
        timer->Instance->CR1 &= ~TIM_CR1_ARPE;
        timer->Instance->ARR = period;
    }

    return TIMER_OK;
}
int timer_set_timeout_us(TIM_HandleTypeDef* timer, uint32_t timeout_us, uint32_t apb_clock_hz) {
    if (!timer || timeout_us == 0 || apb_clock_hz == 0) {
        return TIMER_ERROR_INVALID_PARAM;
    }

    // Calculate the total number of timer ticks required for the timeout.
    // Use 64-bit arithmetic to prevent overflow during calculation.
    uint64_t total_ticks = ((uint64_t)apb_clock_hz * timeout_us) / 1000000ULL;

    uint32_t psc = 0;
    uint32_t arr = 0;

    // Find the best prescaler and auto-reload values.
    // We prioritize a larger ARR value for better resolution.
    if (total_ticks <= 65536) {
        // If the total ticks fit within a 16-bit ARR, no prescaler is needed.
        psc = 0;
        arr = total_ticks - 1;
    } else {
        // Calculate a prescaler that brings the period into the 16-bit range.
        psc = (uint32_t)((total_ticks / 65536U));
        arr = (uint32_t)(total_ticks / (psc + 1)) - 1;
    }

    // Check if the calculated values are within the 16-bit limits.
    if (psc > 65535 || arr > 65535) {
        return TIMER_ERROR_TIME_TOO_LONG;
    }

    // Atomically update the timer registers
    timer->Instance->CR1 |= TIM_CR1_UDIS; // Disable update events
    timer->Instance->PSC = (uint16_t)psc;
    timer->Instance->ARR = (uint16_t)arr;
    timer->Instance->CR1 &= ~TIM_CR1_UDIS; // Re-enable update events
    timer->Instance->EGR = TIM_EGR_UG;     // Generate an update event to load new values
    timer->Instance->SR &= ~TIM_SR_UIF;    // Clear the update flag

    return TIMER_OK;
}

uint32_t timer_get_counter(TIM_HandleTypeDef* timer) {
    return timer ? timer->Instance->CNT : 0;
}
int timer_set_counter(TIM_HandleTypeDef* timer, uint32_t value) {
    if (!timer) return TIMER_ERROR_INVALID_PARAM;
    timer->Instance->CNT = value;
    return TIMER_OK;
}
