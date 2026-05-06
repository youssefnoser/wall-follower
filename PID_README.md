# PID Wall Follower — Design & Usage Guide

An autonomous wall-following robot built on the **STM32F103C8** (Blue Pill). The car navigates corridors by centering itself between walls using **PID control**, and executes precise **90° turns** using IMU-guided yaw PID.

---

## System Overview

```
                  ┌──────────────┐
                  │  Ultrasonic  │
                  │   (Front)    │
                  └──────┬───────┘
                         │
  ┌──────────┐   ┌───────▼───────┐   ┌──────────┐
  │Ultrasonic│   │               │   │Ultrasonic│
  │  (Left)  ├──►│   STM32F103   │◄──┤  (Right) │
  └──────────┘   │               │   └──────────┘
                 │   FSM + PID   │
  ┌──────────┐   │               │   ┌──────────┐
  │  MPU6050 ├──►│               │   │   IR     │
  │  (IMU)   │   └───┬───────┬───┘   │ (Front) │
  └──────────┘       │       │       └─────┬────┘
                ┌────▼─┐   ┌─▼────┐        │
                │Left  │   │Right │        │
                │Motor │   │Motor │        │
                └──────┘   └──────┘        │
                     ▲                     │
                     └─────────────────────┘
                       (safety failsafe)
```

---

## What Was Added & Why

### Problem: Bang-Bang Control

The original code used **fixed motor speeds** — 65% forward, 50% turn. This caused:
- **Zigzag motion**: car bounces between walls with no centering feedback
- **90° overshoot**: fixed turn speed flies past the target angle
- **Jerky response**: raw ultrasonic noise spikes cause sudden motor changes

### Solution: PID Control + Sensor Filtering

We added **3 things** to fix this:

| Component | File | Purpose |
|-----------|------|---------|
| PID Module | `inc/pid.h`, `src/pid.c` | Reusable PID controller with anti-windup |
| Wall-Centering PID | `src/wall_follower_fsm.c` | Centers car in corridor using differential wheel speeds |
| Yaw PID | `src/wall_follower_fsm.c` | Precise 90° turns that decelerate near target |
| EMA Filter | `src/wall_follower_fsm.c` | Smooths ultrasonic noise before FSM processes it |

---

## How It Works

### Finite State Machine

| State | What Happens |
|-------|-------------|
| **IDLE** | Motors stopped. Waits for IR to confirm safe surface → FORWARD |
| **FORWARD** | Wall-centering PID adjusts left/right wheel speeds to stay centered |
| **TURN_LEFT** | Yaw PID rotates +90° — decelerates approaching target angle |
| **TURN_RIGHT** | Yaw PID rotates −90° — same deceleration logic |
| **COUNTER_LEFT/RIGHT** | Retry failed turns (max 3), then → IDLE |

### Wall-Centering PID (FORWARD State)

```
Both walls visible:   error = right_distance − left_distance
Only left wall:       error = left_distance − 15cm (setpoint)
Only right wall:      error = 15cm − right_distance

left_wheel  = 60% + PID_output
right_wheel = 60% − PID_output
```

The **P** term steers toward center, **D** dampens oscillation, **I** corrects persistent offset.

### Yaw PID (TURN States)

```
error = target_yaw − current_yaw
turn_speed = PID(error)     ← slows down as error → 0
```

Instead of spinning at a fixed 50% and overshooting, the PID decelerates smoothly. A minimum speed of 25% prevents motor stall.

### EMA Ultrasonic Filter

```
filtered = 0.3 × raw + 0.7 × previous_filtered
```

Smooths ±2 cm noise spikes that would cause jerky motor response.

---

## Files Changed

| File | Change | Description |
|------|--------|-------------|
| `inc/pid.h` | **NEW** | PID struct + API: `PID_Init`, `PID_Compute`, `PID_Reset` |
| `src/pid.c` | **NEW** | PID implementation with anti-windup & output clamping |
| `src/wall_follower_fsm.h` | **MODIFIED** | Added PID instances, EMA fields, tuning defines |
| `src/wall_follower_fsm.c` | **MODIFIED** | Integrated wall PID, yaw PID, EMA filter |
| `src/main.c` | **MODIFIED** | Enabled wall follower mode, faster loop, PID debug output |
| `CMakeLists.txt` | **MODIFIED** | Added `src/pid.c` to sources |

---

## Tuning Parameters

All in `src/wall_follower_fsm.h`:

| Parameter | Default | What It Does |
|-----------|---------|-------------|
| `WALL_KP` | 2.0 | How aggressively car steers toward center |
| `WALL_KI` | 0.05 | Corrects persistent offset to one side |
| `WALL_KD` | 0.5 | Dampens left-right oscillation |
| `YAW_KP` | 1.5 | How fast the car turns |
| `YAW_KI` | 0.0 | Not needed for quick turns |
| `YAW_KD` | 0.3 | Prevents overshooting 90° |
| `WALL_SETPOINT` | 15.0 cm | Desired distance from wall |
| `BASE_SPEED` | 60 | Forward PWM duty (0–100) |
| `EMA_ALPHA` | 0.3 | Filter strength (lower = smoother) |
| `D_BLOCK` | 20.0 cm | Front obstacle → trigger turn |
| `D_FAR` | 30.0 cm | Wall lost → side is open |

### How to Tune on Real Car

**Wall centering:**
1. Set `WALL_KI=0`, `WALL_KD=0`
2. Increase `WALL_KP` from 1.0 until car oscillates side-to-side
3. Back off Kp by 20%
4. Increase `WALL_KD` until oscillations stop
5. Add tiny `WALL_KI` (0.01–0.05) only if persistent offset exists

**90° turns:**
1. Set `YAW_KD=0`
2. Increase `YAW_KP` until turns overshoot 90°
3. Add `YAW_KD` (0.1–0.5) to stop overshoot

---

## Build & Flash

### Prerequisites
```bash
# Ubuntu/Debian
sudo apt-get install gcc-arm-none-eabi ninja-build cmake openocd

# Arch Linux
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib ninja cmake openocd
```

### Build
```bash
cd Embedded
cmake --preset arm-debug
cmake --build build/
```

### Flash
```bash
cmake --build build/ --target flash
```

Or manually:
```bash
openocd -f interface/stlink-v2.cfg \
        -c "set CPUTAPID 0x2ba01477" \
        -f target/stm32f1x.cfg \
        -c "program build/STM32_Motor_Control.elf verify reset exit"
```

### Monitor Serial Output (115200 baud)
```bash
minicom -D /dev/ttyUSB0 -b 115200
```

Example output:
```
Wall Follower FSM ready.
[FSM] FORWARD       | Raw F: 45.2 L: 14.8 R: 16.1 | Filt F: 44.0 L: 15.0 R: 15.8 cm | IR:1 | Yaw:  0.0->  0.0 | WallPID:  1.6 | Retries L:0 R:0
[FSM] TURN_LEFT     | Raw F: 18.5 L: 35.2 R: 14.0 | Filt F: 36.5 L: 21.1 R: 15.3 cm | IR:1 | Yaw:  0.5-> 90.5 | WallPID:  0.0 | Retries L:0 R:0
```

---

## Switching to WiFi Mode

To use WiFi remote control instead of autonomous wall following, uncomment in `main.c`:
```c
#define WIFI_CONTROL_MODE
```
Then send TCP commands to port 1234: `FORWARD`, `BACK`, `LEFT`, `RIGHT`, `STOP`.

---

## Main Loop Timing

| Phase | Duration |
|-------|----------|
| 3× Ultrasonic readings | 90 ms |
| IR + IMU + FSM + PID | < 1 ms |
| Rate limiter delay | 50 ms |
| **Total loop** | **~140 ms (~7 Hz)** |
