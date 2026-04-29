#include "wall_follower_fsm.h"
#include "l298n.h"
#include "stm32f1xx_hal.h"

/* Motor speeds – tune to your chassis */
#define SPEED_FWD 65  /* 0–100 PWM duty, forward drive  */
#define SPEED_TURN 50 /* 0–100 PWM duty, in-place turn  */

extern L298N_t motor_driver;

/* -----------------------------------------------------------------------
 * Internal Motor Helpers
 * --------------------------------------------------------------------- */

static void L298N_Forward(L298N_t *driver, uint8_t speed) {
  L298N_Move(driver, MOTOR_LEFT, MOTOR_FORWARD, speed);
  L298N_Move(driver, MOTOR_RIGHT, MOTOR_FORWARD, speed);
}

static void L298N_TurnLeft(L298N_t *driver, uint8_t speed) {
  L298N_Move(driver, MOTOR_LEFT, MOTOR_BACKWARD, speed);
  L298N_Move(driver, MOTOR_RIGHT, MOTOR_FORWARD, speed);
}

static void L298N_TurnRight(L298N_t *driver, uint8_t speed) {
  L298N_Move(driver, MOTOR_LEFT, MOTOR_FORWARD, speed);
  L298N_Move(driver, MOTOR_RIGHT, MOTOR_BACKWARD, speed);
}

static void L298N_Stop_All(L298N_t *driver) { L298N_StopAll(driver); }

/* -----------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------- */

/** Normalise angle to (−180, +180] */
static float norm_angle(float a) {
  while (a > 180.0f)
    a -= 360.0f;
  while (a < -180.0f)
    a += 360.0f;
  return a;
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

void WallFollower_Init(WallFollower_t *wf) {
  wf->state = FSM_IDLE;
  wf->retry_left = 0;
  wf->retry_right = 0;
  wf->yaw = 0.0f;
  wf->yaw_target = 0.0f;
  wf->last_tick = HAL_GetTick();
}

/* -----------------------------------------------------------------------
 * Main FSM update – call every loop iteration
 *
 * FSM map (from diagram):
 *
 *  IDLE ──IR_SAFE──────────────────────────────► FORWARD
 *
 *  FORWARD ──(obs|!ir) & !wall_lost_L & !wall_lost_R──► IDLE
 *  FORWARD ──(obs|!ir) & wall_lost_L───────────► TURN_LEFT
 *  FORWARD ──(obs|!ir) & wall_lost_R───────────► TURN_RIGHT
 *
 *  TURN_LEFT ──!ir_safe────────────────────────► IDLE
 *  TURN_LEFT ──rot_done(+90°) & ir_safe────────► FORWARD
 *  TURN_LEFT ──(obs|!ir) & wall_lost_L─────────► COUNTER_LEFT
 *
 *  TURN_RIGHT ──!ir_safe───────────────────────► IDLE
 *  TURN_RIGHT ──rot_done(−90°) & ir_safe───────► FORWARD
 *  TURN_RIGHT ──(obs|!ir) & wall_lost_R────────► COUNTER_RIGHT
 *
 *  COUNTER_LEFT  ──retry ≤ MAX──────────────────► TURN_LEFT   (retry)
 *  COUNTER_LEFT  ──retry > MAX──────────────────► IDLE
 *
 *  COUNTER_RIGHT ──retry ≤ MAX──────────────────► TURN_RIGHT  (retry)
 *  COUNTER_RIGHT ──retry > MAX──────────────────► IDLE
 * --------------------------------------------------------------------- */
void WallFollower_Update(WallFollower_t *wf, float dist_f, float dist_l,
                         float dist_r, uint8_t ir_safe, float gyro_z_dps) {
  /* ---- integrate yaw ------------------------------------------------ */
  uint32_t now = HAL_GetTick();
  float dt = (float)(now - wf->last_tick) * 0.001f; /* ms → s */
  wf->last_tick = now;

  /* Apply a deadband filter to eliminate stationary gyro drift */
  if (gyro_z_dps > -GYRO_DEADBAND && gyro_z_dps < GYRO_DEADBAND) {
    gyro_z_dps = 0.0f;
  }

  wf->yaw += gyro_z_dps * dt;

  /* ---- convenience flags -------------------------------------------- */
  uint8_t obs_front = (dist_f < D_BLOCK);
  uint8_t wall_lost_l = (dist_l > D_FAR);
  uint8_t wall_lost_r = (dist_r > D_FAR);

  /* Signed error from current yaw to target (positive = CCW = left) */
  float yaw_err = norm_angle(wf->yaw - wf->yaw_target);

  /* ---- state machine ------------------------------------------------ */
  switch (wf->state) {

  /* ================================================================== */
  case FSM_IDLE:
    L298N_Stop_All(&motor_driver);
    wf->retry_left = 0;
    wf->retry_right = 0;

    if (ir_safe) {
      /* Surface confirmed on both sides → start moving */
      wf->state = FSM_FORWARD;
    }
    break;

  /* ================================================================== */
  case FSM_FORWARD:
    L298N_Forward(&motor_driver, SPEED_FWD);

    if ((obs_front || !ir_safe) && !wall_lost_l && !wall_lost_r) {
      /* Front blocked or cliff detected, and no side open – stop immediately */
      L298N_Stop_All(&motor_driver);
      wf->state = FSM_IDLE;

    } else if ((obs_front || !ir_safe) && wall_lost_l) {
      /* Front blocked & left wall lost → open space left, turn left */
      wf->yaw_target = wf->yaw + 90.0f;
      wf->state = FSM_TURN_LEFT;

    } else if ((obs_front || !ir_safe) && wall_lost_r) {
      /* Front blocked & right wall lost → open space right, turn right */
      wf->yaw_target = wf->yaw - 90.0f;
      wf->state = FSM_TURN_RIGHT;
    }
    break;

  /* ================================================================== */
  case FSM_TURN_LEFT:
    L298N_TurnLeft(&motor_driver, SPEED_TURN);

    if (!ir_safe) {
      L298N_Stop_All(&motor_driver);
      wf->state = FSM_IDLE;

    } else if (yaw_err >= (90.0f - YAW_TOL) && ir_safe) {
      /* Rotation +90° complete, surface safe → resume forward */
      L298N_Stop_All(&motor_driver);
      wf->retry_left = 0;
      wf->state = FSM_FORWARD;

    } else if ((obs_front || !ir_safe) && wall_lost_l) {
      /* Still blocked after turning → count the attempt */
      L298N_Stop_All(&motor_driver);
      wf->state = FSM_COUNTER_LEFT;
    }
    break;

  /* ================================================================== */
  case FSM_TURN_RIGHT:
    L298N_TurnRight(&motor_driver, SPEED_TURN);

    if (!ir_safe) {
      L298N_Stop_All(&motor_driver);
      wf->state = FSM_IDLE;

    } else if (yaw_err <= -(90.0f - YAW_TOL) && ir_safe) {
      /* Rotation −90° complete, surface safe → resume forward */
      L298N_Stop_All(&motor_driver);
      wf->retry_right = 0;
      wf->state = FSM_FORWARD;

    } else if ((obs_front || !ir_safe) && wall_lost_r) {
      L298N_Stop_All(&motor_driver);
      wf->state = FSM_COUNTER_RIGHT;
    }
    break;

  /* ================================================================== */
  case FSM_COUNTER_LEFT:
    /*
     * Count = 0,1,2,3 → retry TURN_LEFT each time.
     * Counter > 3      → give up, return to IDLE.
     *
     * This state is entered and immediately transitions out; the
     * count just increments on each visit (each failed turn attempt).
     */
    wf->retry_left++;

    if (wf->retry_left > MAX_RETRIES) {
      wf->retry_left = 0;
      wf->state = FSM_IDLE; /* give up */
    } else {
      /* Reset target yaw for a fresh 90° attempt */
      wf->yaw_target = wf->yaw + 90.0f;
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
  case FSM_IDLE:
    return "IDLE";
  case FSM_FORWARD:
    return "FORWARD";
  case FSM_TURN_LEFT:
    return "TURN_LEFT";
  case FSM_TURN_RIGHT:
    return "TURN_RIGHT";
  case FSM_COUNTER_LEFT:
    return "COUNTER_LEFT";
  case FSM_COUNTER_RIGHT:
    return "COUNTER_RIGHT";
  default:
    return "UNKNOWN";
  }
}
