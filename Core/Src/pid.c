#include "pid.h"

// Function to initialize a PID controller
void PID_init(PIDController *pid, float Kp, float Ki, uint32_t Kd,
              uint32_t output_limit) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->error_sum = 0.0f;
  pid->last_error = 0.0f;
  pid->output_limit = output_limit;
}

/**
 * @brief Calculates the PID controller output.
 *
 * This function computes the output of a PID controller by applying the
 * proportional, integral, and derivative terms. It also limits the output to
 * a specified maximum value to prevent excessive control actions.
 *
 * @param pid Pointer to the PIDController structure.
 * @param target_speed The desired target speed.
 * @param current_speed The current measured speed.
 * @param dt The time step (in seconds) since the last update.
 * @return The PID controller output value.
 */
uint32_t calculatePID(PIDController *pid, uint32_t target_speed,
                    uint32_t current_speed, float dt) {
  // Calculate error
  uint32_t error = target_speed - current_speed;

  // Proportional term
  uint32_t proportional = pid->Kp * error;

  // Integral with windup guard
  pid->error_sum += error * dt;
  if (pid->error_sum > pid->integral_limit)
    pid->error_sum = pid->integral_limit;
  if (pid->error_sum < -pid->integral_limit)
    pid->error_sum = -pid->integral_limit;
  uint32_t integral = pid->Ki * pid->error_sum;

  // pid->last_error = error;

  // DERIVATIVE ON MEASUREMENT (no derivative kick)
  // uint32_t derivative_raw = (current_speed - pid->last_measurement) / dt;
  // pid->last_measurement = current_speed;

  // Low‑pass filter for derivative
  // uint32_t alpha = dt / (pid->tau + dt);
  // pid->derivative_filtered =
  //     (1 - alpha) * pid->derivative_filtered + alpha * derivative_raw;
  // uint32_t derivative = -pid->Kd * pid->derivative_filtered;

  // Raw PID output
  // uint32_t pid_output = proportional + integral + derivative;
  uint32_t pid_output = proportional + integral;

  // Limit the PID output
  pid_output = fmaxf(900, fminf(pid_output, pid->output_limit));

  return pid_output;
}

