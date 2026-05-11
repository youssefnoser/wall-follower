"""
pid_tuner.py – Interactive WiFi tuner for the STM32 corridor-centering robot.

IMPORTANT: Right motor is inverted in firmware (L298N_SetDirection), so PID
steering directions are now corrected in the corridor control block.

Protocol (TCP port 1234) — all commands the MCU understands:
  PID=Kp,Ki,Kd   update gains + reset integral on MCU
  MB=<cm>         dead-band margin (suppress corrections smaller than ±N cm)
  SPEED=<n>       base PWM duty 0–100
  LFAC=<f>        left motor scale factor, e.g. 1.08  (range 0.5–2.0)
  RFAC=<f>        right motor scale factor, e.g. 0.95
  HKP=<f>         heading-hold proportional gain
  HKD=<f>         heading-hold derivative gain
  GET             query all current params + live sensor values
  DATA            just sensor snapshot: L=xx F=xx R=xx
  STOP / GO       halt / resume autonomous loop
  AUTO            switch to autonomous PID corridor-centering
  FWD / BWD       manual drive forward / backward
  LEFT / RIGHT    manual pivot left / right

Usage:
  python3 pid_tuner.py <robot-ip>           # default port 1234
  python3 pid_tuner.py <robot-ip> <port>
"""

import argparse
import math
import socket
import sys
import time
import threading

DEFAULT_PORT = 1234
RECV_TIMEOUT = 2.0

# Must match #define SENSITIVITY in firmware
SENSITIVITY = 20.0

# Factor sanity-check range (warn, not reject)
FACTOR_MIN = 0.5
FACTOR_MAX = 2.0


# ---------------------------------------------------------------------------
# Preset tuning sequences
# ---------------------------------------------------------------------------
PRESETS = {
    "1": ("Conservative start  (Kp only, no I)", "PID=0.3,0.0,0.1"),
    "2": ("Add integral bias correction", "PID=0.5,0.03,0.2"),
    "3": ("Recommended start (after motor fix)", "PID=0.5,0.03,0.2"),
    "4": ("More responsive – raise Kp", "PID=0.8,0.05,0.3"),
    "5": ("Aggressive – high Kp, higher D", "PID=1.2,0.1,0.5"),
    "6": ("Stable tuning – low gains, gentle", "PID=0.3,0.02,0.1"),
}


# ---------------------------------------------------------------------------
# Socket helpers
# ---------------------------------------------------------------------------
def send_recv(sock: socket.socket, cmd: str) -> str:
    sock.sendall((cmd.strip() + "\n").encode())
    try:
        return sock.recv(256).decode(errors="replace").strip()
    except socket.timeout:
        return ""


def send_only(sock: socket.socket, cmd: str) -> None:
    try:
        sock.sendall((cmd.strip() + "\n").encode())
    except OSError:
        pass


# ---------------------------------------------------------------------------
# GET response parser
#
# MCU sends:
#   KP=x KI=x KD=x MB=x SPD=x LFAC=x RFAC=x RUN=x L=x F=x R=x
# Returns a dict with float values for every key, or {} on parse failure.
# ---------------------------------------------------------------------------
def parse_get(response: str) -> dict:
    result = {}
    for token in response.split():
        if "=" in token:
            k, _, v = token.partition("=")
            try:
                result[k] = float(v)
            except ValueError:
                pass
                
    if "ANG" in result:
        # Clamp angle between -180 and 180
        result["ANG"] = (result["ANG"] + 180) % 360 - 180
    if "TGT" in result:
        result["TGT"] = (result["TGT"] + 180) % 360 - 180
        
    return result


# ---------------------------------------------------------------------------
# Derived debug quantities (mirrors the firmware AUTO block exactly)
# ---------------------------------------------------------------------------
def derive(state: dict) -> dict:
    """
    Compute wall_error, compressed error, and estimated PWM outputs from
    a parsed GET state dict.  Ki/integral are not tracked client-side so
    pid_out is an approximation (Kp * compressed only).
    """
    if not state:
        return {}

    kp = state.get("KP", 0.0)
    mb = state.get("MB", 2.0)
    spd = state.get("SPD", 42.0)
    lfac = state.get("LFAC", 1.0)
    rfac = state.get("RFAC", 1.0)
    L = state.get("L", 0.0)
    R = state.get("R", 0.0)

    wall_error = L - R
    if -mb < wall_error < mb:
        wall_error = 0.0

    compressed = (
        SENSITIVITY * math.tanh(wall_error / SENSITIVITY) if wall_error != 0.0 else 0.0
    )

    # Client-side estimate: Kp term only (no Ki integral history available)
    pid_est = kp * compressed

    # Right motor is inverted in firmware, so pid_out signs are swapped
    raw_l = spd - pid_est
    raw_r = spd + pid_est

    def clamp(v, lo=0, hi=100):
        return max(lo, min(hi, v))

    pwm_l = clamp(raw_l * lfac)
    pwm_r = clamp(raw_r * rfac)

    return {
        "wall_error": wall_error,
        "compressed": compressed,
        "pid_est": pid_est,
        "pwm_l": pwm_l,
        "pwm_r": pwm_r,
    }


# ---------------------------------------------------------------------------
# Monitor display
# ---------------------------------------------------------------------------
MONITOR_HEADER = (
    "\n"
    f"  {'TIME':<10} {'L':>6} {'R':>6} {'F':>6} "
    f"{'ERR':>7} {'COMP':>7} {'PID≈':>7} "
    f"{'PWM_L':>6} {'PWM_R':>6} {'TGT':>6} {'ANG':>6}  FLAGS\n" + "  " + "─" * 92
)


def format_monitor_row(ts: str, state: dict, derived: dict) -> str:
    L = state.get("L", float("nan"))
    R = state.get("R", float("nan"))
    F = state.get("F", float("nan"))
    tgt = state.get("TGT", float("nan"))
    ang = state.get("ANG", float("nan"))
    run = int(state.get("RUN", 0))

    err = derived.get("wall_error", float("nan"))
    comp = derived.get("compressed", float("nan"))
    pid_e = derived.get("pid_est", float("nan"))
    pwml = derived.get("pwm_l", float("nan"))
    pwmr = derived.get("pwm_r", float("nan"))

    flags = []
    if not run:
        flags.append("STOPPED")
    if abs(err) < state.get("MB", 2.0) and err != 0.0:
        flags.append("in-deadband")
    if F < state.get("SPD", 28.0):  # reuse FRONT_OBSTACLE proxy
        flags.append("FRONT-OBSTACLE")
    if abs(err) > 15.0:
        flags.append("NEAR-WALL")
    elif abs(err) < 3.0 and err != 0.0:
        flags.append("near-center")

    return (
        f"  {ts:<10} {L:>6.1f} {R:>6.1f} {F:>6.1f} "
        f"{err:>+7.2f} {comp:>+7.2f} {pid_e:>+7.2f} "
        f"{pwml:>6.1f} {pwmr:>6.1f} {tgt:>+6.1f} {ang:>+6.1f}  {' | '.join(flags)}"
    )


def monitor_loop(sock: socket.socket, interval: float, stop_event: threading.Event):
    print(MONITOR_HEADER)
    while not stop_event.is_set():
        reply = send_recv(sock, "GET")
        ts = time.strftime("%H:%M:%S")
        if reply:
            state = parse_get(reply)
            derived = derive(state)
            print(format_monitor_row(ts, state, derived))
        else:
            print(f"  {ts:<10}  (no reply)")
        stop_event.wait(interval)


# ---------------------------------------------------------------------------
# Help / presets
# ---------------------------------------------------------------------------
HELP_TEXT = """
Commands
────────────────────────────────────────────────────────────
  PID=Kp,Ki,Kd       e.g.  PID=3.0,0.0,1.2
  MB=<cm>             dead-band margin        e.g.  MB=2.0
  SPEED=<0-100>       base PWM duty           e.g.  SPEED=42

  Motor scale factors (multiplicative, applied per side after PID):
    LFAC=<factor>     left  motor scale       e.g.  LFAC=1.08
    RFAC=<factor>     right motor scale       e.g.  RFAC=0.95
    Typical range 0.50–2.00.  Both default to 1.0 (no correction).
    Calibration tip: send SPEED=50, drive straight in AUTO mode.
    If veering left → raise LFAC or lower RFAC.
    If veering right → raise RFAC or lower LFAC.

  Heading-hold PD (applied during sensor acquisition):
    HKP=<gain>        heading lock P gain     e.g.  HKP=2.0
    HKD=<gain>        heading lock D gain     e.g.  HKD=0.3
    Raise HKP until drift is corrected within one cycle.
    Raise HKD to dampen overshoots.

  GET                 full state dump from MCU
  DATA                live sensor snapshot  L / F / R

  AUTO                switch to autonomous PID centering
  STOP / GO           halt / resume

  Manual drive (switches to MANUAL mode automatically):
    F  or  FWD        drive forward
    B  or  BWD        drive backward
    L  or  LEFT       pivot left
    R  or  RIGHT      pivot right
    S  or  STOP       stop motors

  monitor [N]         live debug table every N s (default 0.5), Ctrl-C to stop
                      Columns: L  R  F  wall_error  compressed  pid≈  PWM_L  PWM_R  GZ  ANG
                      pid≈ is Kp*compressed only (no Ki integral tracked here)

  poll [N]            print raw DATA every N seconds (default 1.0), Ctrl-C to stop
  <1-6>               send a named preset (see 'presets')
  presets             list all presets
  h / help / ?        show this help
  q / quit            exit
────────────────────────────────────────────────────────────
"""


def print_help():
    print(HELP_TEXT)
    print_presets()


def print_presets():
    print("Presets")
    print("───────")
    for key, (label, cmd) in PRESETS.items():
        print(f"  [{key}]  {label:<45}  →  {cmd}")
    print()


# ---------------------------------------------------------------------------
# Live poll thread (raw DATA, kept for quick checks)
# ---------------------------------------------------------------------------
def poll_data(sock: socket.socket, interval: float, stop_event: threading.Event):
    while not stop_event.is_set():
        reply = send_recv(sock, "DATA")
        ts = time.strftime("%H:%M:%S")
        if reply:
            print(f"\r  [{ts}]  {reply}          ", end="", flush=True)
        else:
            print(f"\r  [{ts}]  (no reply)      ", end="", flush=True)
        stop_event.wait(interval)
    print()


# ---------------------------------------------------------------------------
# Command dispatch
# ---------------------------------------------------------------------------
MANUAL_ALIASES = {
    "F": "FWD",
    "FWD": "FWD",
    "B": "BWD",
    "BWD": "BWD",
    "L": "LEFT",
    "LEFT": "LEFT",
    "R": "RIGHT",
    "RIGHT": "RIGHT",
    "S": "STOP",
    "STOP": "STOP",
}


def handle_command(raw: str, sock: socket.socket) -> bool:
    if not raw:
        return True

    low = raw.lower()
    up = raw.upper()

    if low in ("q", "quit", "exit"):
        return False

    if low in ("h", "help", "?"):
        print_help()
        return True

    if low == "presets":
        print_presets()
        return True

    if raw in PRESETS:
        label, cmd = PRESETS[raw]
        print(f"  → {label}: {cmd}")
        reply = send_recv(sock, cmd)
        print(f"  MCU: {reply}" if reply else "  (no reply – check connection)")
        return True

    # monitor [interval]
    if low.startswith("monitor"):
        parts = low.split()
        interval = 0.5
        if len(parts) == 2:
            try:
                interval = float(parts[1])
            except ValueError:
                print("  Usage: monitor [seconds]  e.g. monitor 0.5")
                return True
        print(f"  Monitor every {interval}s — Ctrl-C to stop")
        stop_ev = threading.Event()
        t = threading.Thread(
            target=monitor_loop, args=(sock, interval, stop_ev), daemon=True
        )
        t.start()
        try:
            t.join()
        except KeyboardInterrupt:
            stop_ev.set()
            t.join()
            print("  Monitor stopped.")
        return True

    # poll [interval]
    if low.startswith("poll"):
        parts = low.split()
        interval = 1.0
        if len(parts) == 2:
            try:
                interval = float(parts[1])
            except ValueError:
                print("  Usage: poll [seconds]  e.g. poll 0.5")
                return True
        print(f"  Polling DATA every {interval}s — Ctrl-C to stop")
        stop_ev = threading.Event()
        t = threading.Thread(
            target=poll_data, args=(sock, interval, stop_ev), daemon=True
        )
        t.start()
        try:
            t.join()
        except KeyboardInterrupt:
            stop_ev.set()
            t.join()
            print("  Poll stopped.")
        return True

    if up in MANUAL_ALIASES:
        mcu_cmd = MANUAL_ALIASES[up]
        send_only(sock, mcu_cmd)
        try:
            reply = sock.recv(64).decode(errors="replace").strip()
            print(f"  MCU: {reply}")
        except socket.timeout:
            pass
        return True

    # Validated commands
    if up.startswith("PID="):
        parts = up[4:].split(",")
        if len(parts) != 3:
            print("  Usage: PID=Kp,Ki,Kd   e.g. PID=3.0,0.0,1.2")
            return True
        try:
            kp, ki, kd = float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            print("  All three values must be numbers.")
            return True
        reply = send_recv(sock, f"PID={kp},{ki},{kd}")
        print(f"  MCU: {reply}" if reply else "  (no reply)")

    elif up.startswith("MB="):
        try:
            mb = float(up[3:])
            if mb < 0:
                raise ValueError
        except ValueError:
            print("  Usage: MB=<cm>  e.g. MB=2.0  (must be ≥ 0)")
            return True
        reply = send_recv(sock, f"MB={mb}")
        print(f"  MCU: {reply}" if reply else "  (no reply)")

    elif up.startswith("SPEED="):
        try:
            spd = int(float(up[6:]))
            if not (0 <= spd <= 100):
                raise ValueError
        except ValueError:
            print("  Usage: SPEED=<0-100>  e.g. SPEED=42")
            return True
        reply = send_recv(sock, f"SPEED={spd}")
        print(f"  MCU: {reply}" if reply else "  (no reply)")

    elif up.startswith("LFAC="):
        try:
            fac = float(up[5:])
            if fac <= 0:
                raise ValueError
        except ValueError:
            print("  Usage: LFAC=<factor>  e.g. LFAC=1.08  (must be > 0)")
            return True
        if not (FACTOR_MIN <= fac <= FACTOR_MAX):
            print(
                f"  Warning: LFAC={fac:.3f} outside [{FACTOR_MIN}, {FACTOR_MAX}] — sending anyway."
            )
        reply = send_recv(sock, f"LFAC={fac}")
        print(f"  MCU: {reply}" if reply else "  (no reply)")

    elif up.startswith("RFAC="):
        try:
            fac = float(up[5:])
            if fac <= 0:
                raise ValueError
        except ValueError:
            print("  Usage: RFAC=<factor>  e.g. RFAC=0.95  (must be > 0)")
            return True
        if not (FACTOR_MIN <= fac <= FACTOR_MAX):
            print(
                f"  Warning: RFAC={fac:.3f} outside [{FACTOR_MIN}, {FACTOR_MAX}] — sending anyway."
            )
        reply = send_recv(sock, f"RFAC={fac}")
        print(f"  MCU: {reply}" if reply else "  (no reply)")

    elif up.startswith("HKP="):
        try:
            hkp = float(up[4:])
            if hkp < 0:
                raise ValueError
        except ValueError:
            print("  Usage: HKP=<gain>  e.g. HKP=2.0  (must be ≥ 0)")
            return True
        reply = send_recv(sock, f"HKP={hkp}")
        print(f"  MCU: {reply}" if reply else "  (no reply)")

    elif up.startswith("HKD="):
        try:
            hkd = float(up[4:])
            if hkd < 0:
                raise ValueError
        except ValueError:
            print("  Usage: HKD=<gain>  e.g. HKD=0.3  (must be ≥ 0)")
            return True
        reply = send_recv(sock, f"HKD={hkd}")
        print(f"  MCU: {reply}" if reply else "  (no reply)")

    elif up in ("GET", "DATA", "AUTO", "GO"):
        reply = send_recv(sock, up)
        if reply and up == "GET":
            state = parse_get(reply)
            derived = derive(state)
            hkp_val = state.get("HKP", float("nan"))
            hkd_val = state.get("HKD", float("nan"))
            tgt_val = state.get("TGT", float("nan"))
            print(f"  MCU raw  : {reply}")
            if derived:
                print(
                    f"  Derived  : wall_err={derived['wall_error']:+.2f} cm  "
                    f"compressed={derived['compressed']:+.2f} cm  "
                    f"pid≈{derived['pid_est']:+.2f}  "
                    f"PWM_L≈{derived['pwm_l']:.1f}  PWM_R≈{derived['pwm_r']:.1f}  "
                    f"HKP={hkp_val:.3f}  HKD={hkd_val:.3f}  TGT={tgt_val:.1f}"
                )
        else:
            print(f"  MCU: {reply}" if reply else "  (no reply)")

    else:
        print(f"  Unknown command '{raw}'. Type 'help' for usage.")

    return True


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="WiFi PID tuner – STM32 corridor robot"
    )
    parser.add_argument(
        "host",
        nargs="?",
        default=None,
        help="Robot IP address (printed on serial after WiFi connects)",
    )
    parser.add_argument("port", nargs="?", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()

    if args.host is None:
        args.host = input("Robot IP address: ").strip()

    print(f"\nConnecting to {args.host}:{args.port} …")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(RECV_TIMEOUT)
    try:
        sock.connect((args.host, args.port))
    except (ConnectionRefusedError, OSError) as e:
        print(f"Connection failed: {e}")
        sys.exit(1)
    print("Connected.\n")

    print_help()

    reply = send_recv(sock, "GET")
    if reply:
        state = parse_get(reply)
        derived = derive(state)
        print(f"  MCU current state → {reply}")
        if derived:
            hkp_val = state.get("HKP", float("nan"))
            hkd_val = state.get("HKD", float("nan"))
            tgt_val = state.get("TGT", float("nan"))
            print(
                f"  Derived           → wall_err={derived['wall_error']:+.2f} cm  "
                f"compressed={derived['compressed']:+.2f} cm  "
                f"pid≈{derived['pid_est']:+.2f}  "
                f"PWM_L≈{derived['pwm_l']:.1f}  PWM_R≈{derived['pwm_r']:.1f}  "
                f"HKP={hkp_val:.3f}  HKD={hkd_val:.3f}  TGT={tgt_val:.1f}\n"
            )

    try:
        while True:
            try:
                raw = input("pid_tuner> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not handle_command(raw, sock):
                break
    finally:
        sock.close()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
