#ifndef WALL_FOLLOWER_FSM_H
#define WALL_FOLLOWER_FSM_H

#include <stdint.h>
#include "pid.h"

/* -----------------------------------------------------------------------
 * Thresholds  (tune these in place)
 * --------------------------------------------------------------------- */
#define D_BLOCK 20.0f      /* cm  – front obstacle → trigger avoidance  */
#define D_FAR 30.0f        /* cm  – wall lost → need to turn back        */
#define MAX_RETRIES 3      /* Counter limit before returning to Idle      */
#define YAW_TOL 5.0f       /* deg – rotation-complete tolerance           */
#define GYRO_DEADBAND 2.0f /* deg/s - noise floor for gyro Z drift */
#define STATE_SETTLE_MS 2000 /* ms - blocking delay on every state transition */

/* ---- PID / filter tuning -------------------------------------------- */
#define WALL_SETPOINT 15.0f /* cm  – desired distance from nearest wall  */
#define EMA_ALPHA     0.3f  /* Exponential moving average filter coeff   */
#define BASE_SPEED    60    /* 0–100 PWM duty for forward drive          */
#define TURN_SPEED_MIN 25   /* minimum turn PWM so motors don't stall    */

/* PID gains – wall centering (output = differential speed -100..+100)   */
#define WALL_KP 2.0f
#define WALL_KI 0.05f
#define WALL_KD 0.5f

/* PID gains – yaw control   (output = turn speed 0..100)               */
#define YAW_KP  1.5f
#define YAW_KI  0.0f
#define YAW_KD  0.3f

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

  /* PID controllers */
  PID_t pid_wall;   /* wall-centering (used in FSM_FORWARD)       */
  PID_t pid_yaw;    /* yaw rotation   (used in TURN_LEFT/RIGHT)   */

  /* Ultrasonic EMA-filtered distances */
  float filt_front;
  float filt_left;
  float filt_right;
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
