/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PID_H
#define PID_H

#include <math.h>

#include <stdint.h>
typedef struct {
  uint32_t Kp; // Proportional gain
  uint32_t Ki; // Integral gain
  uint32_t Kd; // Differential gain

  uint32_t tau;

  uint32_t error_sum;  // Accumulator for integral term
  uint32_t last_error; // For calculating derivative term
  uint32_t last_filtered_output;
  uint32_t derivative_filtered;
  uint32_t last_measurement;
  uint32_t output_limit;
  uint32_t integral_limit;
} PIDController;

typedef struct {
  uint32_t kp;
  uint32_t ki;
  uint32_t kd;
} PIDgain;


// Function to initialize a PID controller
void PID_init(PIDController *pid, float Kp, float Ki, uint32_t Kd,
               uint32_t output_limit);

// Function to calculate PID output
uint32_t calculatePID(PIDController *pid, uint32_t target_angle,
                    uint32_t current_angle, float dt);

#endif
