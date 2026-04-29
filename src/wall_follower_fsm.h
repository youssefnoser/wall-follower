#ifndef WALL_FOLLOWER_FSM_H
#define WALL_FOLLOWER_FSM_H

#include <stdint.h>

/* -----------------------------------------------------------------------
 * Thresholds  (tune these in place)
 * --------------------------------------------------------------------- */
#define D_BLOCK 20.0f      /* cm  – front obstacle → trigger avoidance  */
#define D_FAR 30.0f        /* cm  – wall lost → need to turn back        */
#define MAX_RETRIES 3      /* Counter limit before returning to Idle      */
#define YAW_TOL 5.0f       /* deg – rotation-complete tolerance           */
#define GYRO_DEADBAND 2.0f /* deg/s - noise floor for gyro Z drift */

/* -----------------------------------------------------------------------
 * States
 * --------------------------------------------------------------------- */
typedef enum {
  FSM_IDLE,
  FSM_FORWARD,
  FSM_TURN_LEFT,
  FSM_TURN_RIGHT,
  FSM_COUNTER_LEFT,
  FSM_COUNTER_RIGHT
} FSM_State_t;

/* -----------------------------------------------------------------------
 * FSM context struct
 * --------------------------------------------------------------------- */
typedef struct {
  FSM_State_t state;

  /* Retry counters (one per side) */
  uint8_t retry_left;
  uint8_t retry_right;

  /* Yaw tracking (integrated from gyro_z) */
  float yaw;        /* current integrated yaw, degrees            */
  float yaw_target; /* target yaw for active rotation             */

  /* Timing for dt calculation */
  uint32_t last_tick; /* HAL_GetTick() value from last update call  */
} WallFollower_t;

/* -----------------------------------------------------------------------
 * API
 * --------------------------------------------------------------------- */

/**
 * @brief  Initialise FSM to IDLE, zero yaw, zero counters.
 *         Call once after all hardware inits.
 */
void WallFollower_Init(WallFollower_t *wf);

/**
 * @brief  Run one FSM step.  Call every main-loop iteration.
 *
 * @param  wf          FSM context
 * @param  dist_f      HC-SR04 front distance, cm
 * @param  dist_l      HC-SR04 left  distance, cm
 * @param  dist_r      HC-SR04 right distance, cm
 * @param  ir_safe     1 = front IR clear (safe), 0 = front obstacle detected
 * @param  gyro_z_dps  MPU-6050 gyro Z reading in degrees/second
 */
void WallFollower_Update(WallFollower_t *wf, float dist_f, float dist_l,
                         float dist_r, uint8_t ir_safe, float gyro_z_dps);

/** @brief  Returns a human-readable state name for printf debugging. */
const char *FSM_StateName(FSM_State_t s);

#endif /* WALL_FOLLOWER_FSM_H */
