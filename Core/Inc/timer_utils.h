#ifndef STM32_TIMER_UTILS_H
#define STM32_TIMER_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h" // Assumes main.h includes the correct device header
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

typedef enum {
    /**
     * @brief The new value is applied on the next update event (UEV).
     * This is the "safe" method to prevent glitches while the timer is running.
     * For ARR, this requires ARPE bit to be set. For PSC, this is the default behavior.
     */
    TIMER_UPDATE_SAFE,

    /**
     * @brief The new value is applied immediately by forcing an update event (UEV).
     * For ARR, this requires ARPE bit to be cleared.
     */
    TIMER_UPDATE_IMMEDIATE,
} timer_update_mode_t;


/* Exported constants --------------------------------------------------------*/
#define TIMER_OK                    0
#define TIMER_ERROR_INVALID_PARAM  -1
#define TIMER_ERROR_TIME_TOO_LONG  -2


/* Exported functions --------------------------------------------------------*/

/* --- Basic Timer Control --- */
int timer_start(TIM_HandleTypeDef* timer);
int timer_stop(TIM_HandleTypeDef* timer);

/* --- Register Update Functions --- */
int timer_update_prescaler(TIM_HandleTypeDef* timer, uint16_t prescaler, timer_update_mode_t mode);
int timer_update_period(TIM_HandleTypeDef* timer, uint16_t period, timer_update_mode_t mode);

/* --- Timing Functions --- */
int timer_set_timeout_us(TIM_HandleTypeDef* timer, uint32_t timeout_us, uint32_t apb_clock_hz);

/* --- Counter Functions --- */
uint32_t timer_get_counter(TIM_HandleTypeDef* timer);
int timer_set_counter(TIM_HandleTypeDef* timer, uint32_t value);


#ifdef __cplusplus
}
#endif

#endif /* STM32_TIMER_UTILS_H */
