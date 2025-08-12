#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stdint.h"

typedef enum {
  MOTOR_STEP_1, // A->B ----- C 
  MOTOR_STEP_2, // A->C
  MOTOR_STEP_3, // B->C
  MOTOR_STEP_4, // B->A
  MOTOR_STEP_5, // C->A
  MOTOR_STEP_6, // C->B
  MOTOR_STEP_COUNT,
  MOTOR_STOP
} motor_step_t;

typedef enum {
  SOURCE_NONE,
  SOURCE_MOTOR_PHASE_A,
  SOURCE_MOTOR_PHASE_B,
  SOURCE_MOTOR_PHASE_C,
} OneShotSource_t;

typedef enum {
  BEMF_ERROR = -1,
  BEMF_UNDERSHOOT = 0,
  BEMF_VALID = 1,
  BEMF_OVERSHOOT = 2
} bemf_case_t;

typedef struct {
  uint32_t elapsed_cnt_at_bemf;
  uint32_t last_elapsed_cnt_at_bemf;
  uint32_t current_period;
  uint32_t last_period;
  uint8_t back_to_back;
} ctx_t;


void motor_init(void);
void motor_align(void);
void motor_step(motor_step_t step, float duty_cycle);
void set_commutation_period_us(uint32_t period_us);
void set_commutation_period_us_63(uint32_t period_us);
void set_commutation_period_us_255(uint32_t period_us);

#endif /* INC_MOTOR_CONTROL_H_ */
