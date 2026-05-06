#include "wall_follower_fsm.h"
#include "l298n.h"
#include "stm32f1xx_hal.h"

extern L298N_t motor_driver;

/* -----------------------------------------------------------------------
 * Internal Motor Helpers
 * --------------------------------------------------------------------- */

/** Drive both wheels forward with independent speeds (0–100 each). */
static void drive_differential(L298N_t *driver,
                               uint8_t left_speed, uint8_t right_speed) {
  L298N_Move(driver, MOTOR_LEFT,  MOTOR_FORWARD, left_speed);
  L298N_Move(driver, MOTOR_RIGHT, MOTOR_FORWARD, right_speed);
}

static void L298N_TurnLeft(L298N_t *driver, uint8_t speed) {
  L298N_Move(driver, MOTOR_LEFT,  MOTOR_BACKWARD, speed);
  L298N_Move(driver, MOTOR_RIGHT, MOTOR_FORWARD,  speed);
}

static void L298N_TurnRight(L298N_t *driver, uint8_t speed) {
  L298N_Move(driver, MOTOR_LEFT,  MOTOR_FORWARD,  speed);
  L298N_Move(driver, MOTOR_RIGHT, MOTOR_BACKWARD, speed);
}

static void L298N_Stop_All(L298N_t *driver) { L298N_StopAll(driver); }

/* -----------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------- */

/** Normalise angle to (−180, +180] */
static float norm_angle(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

/** Clamp an integer to [lo, hi]. */
static int clamp_i(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/** Absolute value for float. */
static float fabsf_local(float x) { return x < 0.0f ? -x : x; }

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

void WallFollower_Init(WallFollower_t *wf) {
  wf->state       = FSM_IDLE;
  wf->retry_left  = 0;
  wf->retry_right = 0;
  wf->yaw         = 0.0f;
  wf->yaw_target  = 0.0f;
  wf->last_tick   = HAL_GetTick();

  /* Initialise PID controllers */
  PID_Init(&wf->pid_wall, WALL_KP, WALL_KI, WALL_KD, -100.0f, 100.0f);
  PID_Init(&wf->pid_yaw,  YAW_KP,  YAW_KI,  YAW_KD,    0.0f, 100.0f);

  /* Seed EMA filters (will converge after first real readings) */
  wf->filt_front = 100.0f;
  wf->filt_left  = 100.0f;
  wf->filt_right = 100.0f;
}

/* -----------------------------------------------------------------------
 * Main FSM update – call every loop iteration
 *
 * FSM map (unchanged from the original design):
 *
 *  IDLE ──IR_SAFE──────────────────────────────► FORWARD
 *  FORWARD ──(obs|!ir) & !wall_lost_L & !R────► IDLE
 *  FORWARD ──(obs|!ir) & wall_lost_L──────────► TURN_LEFT
 *  FORWARD ──(obs|!ir) & wall_lost_R──────────► TURN_RIGHT
 *  TURN_LEFT  ──rot_done(+90°) & ir_safe──────► FORWARD
 *  TURN_RIGHT ──rot_done(−90°) & ir_safe──────► FORWARD
 *  COUNTER_LEFT  ──retry ≤ MAX────────────────► TURN_LEFT
 *  COUNTER_RIGHT ──retry ≤ MAX────────────────► TURN_RIGHT
 *  COUNTER_*     ──retry > MAX────────────────► IDLE
 * --------------------------------------------------------------------- */
void WallFollower_Update(WallFollower_t *wf, float dist_f, float dist_l,
                         float dist_r, uint8_t ir_safe, float gyro_z_dps) {
  /* ---- compute dt --------------------------------------------------- */
  uint32_t now = HAL_GetTick();
  float dt = (float)(now - wf->last_tick) * 0.001f; /* ms → s */
  wf->last_tick = now;

  /* ---- gyro deadband & yaw integration ------------------------------ */
  if (gyro_z_dps > -GYRO_DEADBAND && gyro_z_dps < GYRO_DEADBAND) {
    gyro_z_dps = 0.0f;
  }
  wf->yaw += gyro_z_dps * dt;

  /* ---- EMA filter on ultrasonic readings ---------------------------- */
  wf->filt_front = EMA_ALPHA * dist_f + (1.0f - EMA_ALPHA) * wf->filt_front;
  wf->filt_left  = EMA_ALPHA * dist_l + (1.0f - EMA_ALPHA) * wf->filt_left;
  wf->filt_right = EMA_ALPHA * dist_r + (1.0f - EMA_ALPHA) * wf->filt_right;

  /* ---- convenience flags (use filtered values) ---------------------- */
  uint8_t obs_front   = (wf->filt_front < D_BLOCK);
  uint8_t wall_lost_l = (wf->filt_left  > D_FAR);
  uint8_t wall_lost_r = (wf->filt_right > D_FAR);

  /* Signed error from current yaw to target (positive = CCW = left) */
  float yaw_err = norm_angle(wf->yaw - wf->yaw_target);

  /* ---- state machine ------------------------------------------------ */
  switch (wf->state) {

  /* ================================================================== */
  case FSM_IDLE:
    L298N_Stop_All(&motor_driver);
    wf->retry_left  = 0;
    wf->retry_right = 0;

    if (ir_safe) {
      PID_Reset(&wf->pid_wall);
      wf->state = FSM_FORWARD;
    }
    break;

  /* ================================================================== */
  case FSM_FORWARD: {
    /*
     * Wall-centering PID:
     *   - Both walls visible → error = right_dist - left_dist
     *     (positive error means car is closer to left wall → steer right)
     *   - Only left wall → error = left_dist - WALL_SETPOINT
     *     (positive = too far from left wall → steer left = reduce error)
     *   - Only right wall → error = WALL_SETPOINT - right_dist
     *     (positive = too close to right wall → steer left)
     *
     * PID output is added to left wheel and subtracted from right wheel.
     */
    float wall_error;
    if (!wall_lost_l && !wall_lost_r) {
      /* Both walls visible: centre between them */
      wall_error = wf->filt_right - wf->filt_left;
    } else if (!wall_lost_l) {
      /* Only left wall visible: maintain WALL_SETPOINT from it */
      wall_error = wf->filt_left - WALL_SETPOINT;
    } else if (!wall_lost_r) {
      /* Only right wall visible: maintain WALL_SETPOINT from it */
      wall_error = WALL_SETPOINT - wf->filt_right;
    } else {
      /* No walls visible at all: drive straight */
      wall_error = 0.0f;
    }

    /* Setpoint = 0 (we want error = 0), measurement = -wall_error */
    float pid_out = PID_Compute(&wf->pid_wall, 0.0f, -wall_error, dt);

    int left_spd  = clamp_i((int)(BASE_SPEED + pid_out), 0, 100);
    int right_spd = clamp_i((int)(BASE_SPEED - pid_out), 0, 100);

    drive_differential(&motor_driver, (uint8_t)left_spd, (uint8_t)right_spd);

    /* ---- State transitions ----------------------------------------- */
    if ((obs_front || !ir_safe) && !wall_lost_l && !wall_lost_r) {
      /* Front blocked, no open side → stop */
      L298N_Stop_All(&motor_driver);
      wf->state = FSM_IDLE;

    } else if ((obs_front || !ir_safe) && wall_lost_l) {
      /* Front blocked & left wall lost → turn left */
      wf->yaw_target = wf->yaw + 90.0f;
      PID_Reset(&wf->pid_yaw);
      wf->state = FSM_TURN_LEFT;

    } else if ((obs_front || !ir_safe) && wall_lost_r) {
      /* Front blocked & right wall lost → turn right */
      wf->yaw_target = wf->yaw - 90.0f;
      PID_Reset(&wf->pid_yaw);
      wf->state = FSM_TURN_RIGHT;
    }
    break;
  }

  /* ================================================================== */
  case FSM_TURN_LEFT: {
    /*
     * Yaw PID: setpoint = yaw_target, measurement = yaw
     * Output = turn speed that decreases as we approach the target.
     */
    float turn_out = PID_Compute(&wf->pid_yaw, wf->yaw_target, wf->yaw, dt);
    uint8_t turn_spd = (uint8_t)clamp_i((int)fabsf_local(turn_out),
                                        TURN_SPEED_MIN, 100);

    L298N_TurnLeft(&motor_driver, turn_spd);

    if (!ir_safe) {
      L298N_Stop_All(&motor_driver);
      wf->state = FSM_IDLE;

    } else if (yaw_err >= (90.0f - YAW_TOL) && ir_safe) {
      /* Rotation +90° complete */
      L298N_Stop_All(&motor_driver);
      wf->retry_left = 0;
      PID_Reset(&wf->pid_wall);
      wf->state = FSM_FORWARD;

    } else if ((obs_front || !ir_safe) && wall_lost_l) {
      L298N_Stop_All(&motor_driver);
      wf->state = FSM_COUNTER_LEFT;
    }
    break;
  }

  /* ================================================================== */
  case FSM_TURN_RIGHT: {
    float turn_out = PID_Compute(&wf->pid_yaw, wf->yaw_target, wf->yaw, dt);
    uint8_t turn_spd = (uint8_t)clamp_i((int)fabsf_local(turn_out),
                                        TURN_SPEED_MIN, 100);

    L298N_TurnRight(&motor_driver, turn_spd);

    if (!ir_safe) {
      L298N_Stop_All(&motor_driver);
      wf->state = FSM_IDLE;

    } else if (yaw_err <= -(90.0f - YAW_TOL) && ir_safe) {
      /* Rotation −90° complete */
      L298N_Stop_All(&motor_driver);
      wf->retry_right = 0;
      PID_Reset(&wf->pid_wall);
      wf->state = FSM_FORWARD;

    } else if ((obs_front || !ir_safe) && wall_lost_r) {
      L298N_Stop_All(&motor_driver);
      wf->state = FSM_COUNTER_RIGHT;
    }
    break;
  }

  /* ================================================================== */
  case FSM_COUNTER_LEFT:
    wf->retry_left++;

    if (wf->retry_left > MAX_RETRIES) {
      wf->retry_left = 0;
      wf->state = FSM_IDLE;
    } else {
      wf->yaw_target = wf->yaw + 90.0f;
      PID_Reset(&wf->pid_yaw);
      wf->state = FSM_TURN_LEFT;
    }
    break;

  /* ================================================================== */
  case FSM_COUNTER_RIGHT:
    wf->retry_right++;

    if (wf->retry_right > MAX_RETRIES) {
      wf->retry_right = 0;
      wf->state = FSM_IDLE;
    } else {
      wf->yaw_target = wf->yaw - 90.0f;
      PID_Reset(&wf->pid_yaw);
      wf->state = FSM_TURN_RIGHT;
    }
    break;
  }
}

/* -----------------------------------------------------------------------
 * Debug helper
 * --------------------------------------------------------------------- */
const char *FSM_StateName(FSM_State_t s) {
  switch (s) {
  case FSM_IDLE:          return "IDLE";
  case FSM_FORWARD:       return "FORWARD";
  case FSM_TURN_LEFT:     return "TURN_LEFT";
  case FSM_TURN_RIGHT:    return "TURN_RIGHT";
  case FSM_COUNTER_LEFT:  return "COUNTER_LEFT";
  case FSM_COUNTER_RIGHT: return "COUNTER_RIGHT";
  default:                return "UNKNOWN";
  }
}
