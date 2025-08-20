#ifndef PID_H
#define PID_H

#include <stdint.h>

// Fixed-point scale (Q12: 1.0 = 4096)
#define Q   12
#define ONE_Q12 (1 << Q)

// Normalization constants
#define INPUT_SCALE   3800    // counts (range of your sensor)

#define DUTY_MIN     1050
#define DUTY_MAX     8000
#define DUTY_RANGE   (DUTY_MAX - DUTY_MIN)
//
// PID controller struct
typedef struct {
    int32_t Kp;        // Proportional gain (Q12, dimensionless)
    int32_t Ki;        // Integral gain per tick (Q12, dimensionless)
    int32_t Kd;        // Derivative gain (Q12, optional)

    int32_t error_sum; // Integral accumulator (Q12 per-unit)
    int32_t last_error;// For derivative (per-unit Q12)

    int32_t output_limit;   // Output max in duty units
    int32_t integral_limit; // Clamp for integrator (Q12 per-unit)
} PIDController;

// Init
void PID_init(PIDController *pid, int32_t Kp_Q12, int32_t Ki_Q12, int32_t Kd_Q12,
              int32_t output_limit);

// Step
int32_t PID_calculate(PIDController *pid, int32_t setpoint, int32_t measurement);

#endif
