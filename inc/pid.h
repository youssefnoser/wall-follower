#ifndef PID_H
#define PID_H

/* -----------------------------------------------------------------------
 * Generic PID controller with anti-windup
 * --------------------------------------------------------------------- */

typedef struct {
  float Kp;
  float Ki;
  float Kd;

  float integral;
  float prev_error;

  float out_min; /* output clamp lower bound */
  float out_max; /* output clamp upper bound */

  float integral_max; /* anti-windup: |integral| ≤ this */
} PID_t;

/**
 * @brief  Initialise a PID controller.
 * @param  pid      PID context
 * @param  Kp       Proportional gain
 * @param  Ki       Integral gain
 * @param  Kd       Derivative gain
 * @param  out_min  Minimum output (e.g. -100)
 * @param  out_max  Maximum output (e.g. +100)
 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float out_min, float out_max);

/**
 * @brief  Compute one PID iteration.
 * @param  pid          PID context
 * @param  setpoint     Desired value
 * @param  measurement  Current measured value
 * @param  dt           Time step in seconds (> 0)
 * @return Clamped control output in [out_min, out_max]
 */
float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt);

/**
 * @brief  Reset integral and derivative state (e.g. on state transitions).
 */
void PID_Reset(PID_t *pid);

#endif /* PID_H */
