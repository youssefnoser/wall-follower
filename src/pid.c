#include "pid.h"

/* ----------------------------------------------------------------------- */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float out_min, float out_max) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;

  pid->integral   = 0.0f;
  pid->prev_error = 0.0f;

  pid->out_min = out_min;
  pid->out_max = out_max;

  /* Default anti-windup limit: half the output range */
  pid->integral_max = (out_max - out_min) * 0.5f;
}

/* ----------------------------------------------------------------------- */
float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt) {
  if (dt <= 0.0f)
    return 0.0f;

  float error = setpoint - measurement;

  /* --- Proportional --------------------------------------------------- */
  float P = pid->Kp * error;

  /* --- Integral (with anti-windup clamp) ------------------------------ */
  pid->integral += error * dt;
  if (pid->integral > pid->integral_max)
    pid->integral = pid->integral_max;
  else if (pid->integral < -pid->integral_max)
    pid->integral = -pid->integral_max;
  float I = pid->Ki * pid->integral;

  /* --- Derivative ----------------------------------------------------- */
  float derivative = (error - pid->prev_error) / dt;
  float D = pid->Kd * derivative;
  pid->prev_error = error;

  /* --- Sum & clamp ---------------------------------------------------- */
  float output = P + I + D;
  if (output > pid->out_max)
    output = pid->out_max;
  else if (output < pid->out_min)
    output = pid->out_min;

  return output;
}

/* ----------------------------------------------------------------------- */
void PID_Reset(PID_t *pid) {
  pid->integral   = 0.0f;
  pid->prev_error = 0.0f;
}
