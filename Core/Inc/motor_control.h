#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stdint.h"

typedef enum {
  MOTOR_STEP_1, // A->B
  MOTOR_STEP_2, // A->C
  MOTOR_STEP_3, // B->C
  MOTOR_STEP_4, // B->A
  MOTOR_STEP_5, // C->A
  MOTOR_STEP_6, // C->B
  MOTOR_STEP_COUNT,
  MOTOR_STOP
} motor_step_t;

void motor_init(void);
void motor_align(void);
void motor_step(motor_step_t step, float duty_cycle);
void set_commutation_period_us(uint32_t period_us);
void set_commutation_period_us_63(uint32_t period_us);
void set_commutation_period_us_255(uint32_t period_us);

#endif /* INC_MOTOR_CONTROL_H_ */
