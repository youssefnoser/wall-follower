"""
pid_tuner.py – Interactive WiFi PID tuner for the STM32 left-wall follower.

Protocol (TCP port 1234):
  PID=Kp,Ki,Kd   update all three gains (resets integral on the MCU)
  SP=<cm>        change wall setpoint distance
  GET            query current values  →  MCU replies with one line

Usage:
  python3 pid_tuner.py <robot-ip>           # default port 1234
  python3 pid_tuner.py <robot-ip> <port>
"""

import argparse
import socket
import sys

DEFAULT_PORT = 1234
RECV_TIMEOUT = 2.0  # seconds to wait for an ACK


# ---------------------------------------------------------------------------
# Preset tuning sequences – send these in order while watching the robot
# ---------------------------------------------------------------------------
PRESETS = {
    "1": ("Conservative start  (Kp only)",       "PID=1.5,0.0,0.3"),
    "2": ("Add derivative damping",               "PID=1.5,0.0,0.8"),
    "3": ("Raise Kp – more aggressive",           "PID=2.5,0.0,0.3"),
    "4": ("Add small integral (steady-state fix)", "PID=1.5,0.02,0.3"),
    "5": ("Balanced (Kp/Ki/Kd)",                  "PID=2.0,0.03,0.5"),
    "6": ("Oscillating fix – reduce Kp+raise Kd", "PID=1.2,0.01,0.6"),
}


# ---------------------------------------------------------------------------
def send_recv(sock: socket.socket, cmd: str) -> str:
    """Send a command string and return the MCU's reply (or '' on timeout)."""
    sock.sendall((cmd.strip() + "\n").encode())
    try:
        return sock.recv(256).decode(errors="replace").strip()
    except socket.timeout:
        return ""


def print_help():
    print("""
Commands
--------
  PID=Kp,Ki,Kd   e.g.  PID=1.5,0.0,0.3
  SP=<cm>         e.g.  SP=20.0
  GET             query current params from MCU
  <1-6>           send a named preset (see list below)
  presets         show all presets
  q / quit        exit
""")
    print_presets()


def print_presets():
    print("Presets")
    print("-------")
    for key, (label, cmd) in PRESETS.items():
        print(f"  [{key}]  {label:<40}  →  {cmd}")
    print()


# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="WiFi PID tuner for left-wall follower")
    parser.add_argument("host", nargs="?", default=None,
                        help="Robot IP address (printed on serial after WiFi connects)")
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

    # Ask MCU for its current state immediately
    reply = send_recv(sock, "GET")
    if reply:
        print(f"  MCU current params → {reply}\n")

    try:
        while True:
            try:
                raw = input("pid_tuner> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not raw:
                continue

            low = raw.lower()

            if low in ("q", "quit", "exit"):
                break

            if low in ("h", "help", "?"):
                print_help()
                continue

            if low == "presets":
                print_presets()
                continue

            # Numeric preset shortcut
            if raw in PRESETS:
                label, cmd = PRESETS[raw]
                print(f"  → {label}: {cmd}")
                reply = send_recv(sock, cmd)
                print(f"  MCU: {reply}" if reply else "  (no reply – check connection)")
                continue

            # Validate and send raw commands
            up = raw.upper()
            if up.startswith("PID="):
                parts = up[4:].split(",")
                if len(parts) != 3:
                    print("  Usage: PID=Kp,Ki,Kd   e.g. PID=1.5,0.0,0.3")
                    continue
                try:
                    kp, ki, kd = float(parts[0]), float(parts[1]), float(parts[2])
                except ValueError:
                    print("  All three values must be numbers.")
                    continue
                reply = send_recv(sock, f"PID={kp},{ki},{kd}")
                print(f"  MCU: {reply}" if reply else "  (no reply)")

            elif up.startswith("SP="):
                try:
                    sp = float(up[3:])
                except ValueError:
                    print("  Usage: SP=<cm>   e.g. SP=20.0")
                    continue
                reply = send_recv(sock, f"SP={sp}")
                print(f"  MCU: {reply}" if reply else "  (no reply)")

            elif up == "GET":
                reply = send_recv(sock, "GET")
                print(f"  MCU: {reply}" if reply else "  (no reply)")

            else:
                print(f"  Unknown command '{raw}'. Type 'help' for usage.")

    finally:
        sock.close()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
