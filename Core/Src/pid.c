#include "pid.h"

void PID_init(PIDController *pid, int32_t Kp_Q12, int32_t Ki_Q12,
              int32_t Kd_Q12, int32_t output_limit) {
  pid->Kp = Kp_Q12;
  pid->Ki = Ki_Q12;
  pid->Kd = Kd_Q12;

  pid->error_sum = 0;
  pid->last_error = 0;

  pid->output_limit = output_limit;
  pid->integral_limit = ONE_Q12; // ±1.0 per-unit
}

int32_t PID_calculate(PIDController *pid, int32_t setpoint,
                      int32_t measurement) {
  // Error in counts
  int32_t error_counts = setpoint - measurement;

  // Normalize error to per-unit Q12
  int32_t error_q12 = ((int64_t)error_counts << Q) / INPUT_SCALE;

  // --- Proportional ---
  int32_t P_q12 = ((int64_t)pid->Kp * error_q12) >> Q;

  // --- Integral ---
  pid->error_sum += ((int64_t)pid->Ki * error_q12) >> Q;
  // Clamp integrator (±1.0 per-unit)
  if (pid->error_sum > pid->integral_limit)
    pid->error_sum = pid->integral_limit;
  if (pid->error_sum < -pid->integral_limit)
    pid->error_sum = -pid->integral_limit;
  int32_t I_q12 = pid->error_sum;

  // --- Derivative (optional, currently off) ---
  // int32_t dErr = error_q12 - pid->last_error;
  // pid->last_error = error_q12;
  // int32_t D_q12 = ((int64_t)pid->Kd * dErr) >> Q;
  int32_t D_q12 = 0;

  // --- Total output in per-unit Q12 ---
  int32_t u_q12 = P_q12 + I_q12 + D_q12;

  int32_t raw =
      (int32_t)(((int64_t)u_q12 * DUTY_RANGE) >> Q); // 0..DUTY_RANGE (Q12->int)
  int32_t output = raw + DUTY_MIN;

  // Clamp
  if (output > DUTY_MAX)
    output = DUTY_MAX;
  if (output < DUTY_MIN)
    output = DUTY_MIN;
  return output;
}
