"""
center_tuner.py – WiFi tuner for the STM32 corridor-centering PID.

The robot reads left + right ultrasonics, computes the centering error
(right_dist - left_dist), applies a dead-band margin, then runs PID to
keep both distances equal (error → 0).

TCP commands sent to the robot (port 1234):
  PID=Kp,Ki,Kd   update gains (integral is reset on the MCU)
  MB=<cm>         set dead-band margin in cm  (no correction inside window)
  GET             query current values → MCU replies with one ACK line

Usage:
  python3 center_tuner.py <robot-ip>
  python3 center_tuner.py <robot-ip> <port>
"""

import argparse
import socket
import sys

DEFAULT_PORT = 1234
RECV_TIMEOUT = 2.0

# ---------------------------------------------------------------------------
# Presets  (Kp can be negative if the car drifts the wrong way – flip it)
# ---------------------------------------------------------------------------
PRESETS = {
    "1": ("Conservative  – gentle centering",        "PID=1.0,0.0,0.5",  None),
    "2": ("Moderate      – balanced Kp/Kd",          "PID=1.5,0.0,0.8",  None),
    "3": ("Aggressive    – fast correction",          "PID=2.5,0.0,0.8",  None),
    "4": ("Add integral  – fix steady offset",        "PID=1.5,0.02,0.8", None),
    "5": ("Oscillating?  – reduce Kp, raise Kd",     "PID=1.0,0.0,1.2",  None),
    "6": ("Flip sign     – if car goes wrong way",   "PID=-1.5,0.0,-0.8", None),
    "7": ("Tight margin  – 1 cm dead-band",           None,               "MB=1.0"),
    "8": ("Loose margin  – 4 cm dead-band",           None,               "MB=4.0"),
}


# ---------------------------------------------------------------------------
def send_recv(sock: socket.socket, cmd: str) -> str:
    sock.sendall((cmd.strip() + "\n").encode())
    try:
        return sock.recv(256).decode(errors="replace").strip()
    except socket.timeout:
        return ""


def print_help():
    print("""
Commands
--------
  PID=Kp,Ki,Kd   e.g.  PID=1.5,0.0,0.8
  MB=<cm>         e.g.  MB=2.5          (dead-band margin)
  GET             query current params from MCU
  <1-8>           send a named preset
  presets         show preset list
  q / quit        exit
""")
    print_presets()


def print_presets():
    print("Presets")
    print("-------")
    for key, (label, pid_cmd, mb_cmd) in PRESETS.items():
        cmd_str = pid_cmd or mb_cmd
        print(f"  [{key}]  {label:<42}  →  {cmd_str}")
    print()


# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="WiFi corridor-centering PID tuner")
    parser.add_argument("host", nargs="?", default=None)
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
        print(f"  MCU current params → {reply}\n")

    try:
        while True:
            try:
                raw = input("center_tuner> ").strip()
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

            # Numeric preset
            if raw in PRESETS:
                label, pid_cmd, mb_cmd = PRESETS[raw]
                cmd = pid_cmd or mb_cmd
                print(f"  → {label}: {cmd}")
                reply = send_recv(sock, cmd)
                print(f"  MCU: {reply}" if reply else "  (no reply – check connection)")
                continue

            up = raw.upper()

            if up.startswith("PID="):
                parts = up[4:].split(",")
                if len(parts) != 3:
                    print("  Usage: PID=Kp,Ki,Kd")
                    continue
                try:
                    kp, ki, kd = float(parts[0]), float(parts[1]), float(parts[2])
                except ValueError:
                    print("  All three values must be numbers.")
                    continue
                reply = send_recv(sock, f"PID={kp},{ki},{kd}")
                print(f"  MCU: {reply}" if reply else "  (no reply)")

            elif up.startswith("MB="):
                try:
                    mb = float(up[3:])
                    if mb < 0:
                        raise ValueError
                except ValueError:
                    print("  Usage: MB=<cm>  (positive number)")
                    continue
                reply = send_recv(sock, f"MB={mb}")
                print(f"  MCU: {reply}" if reply else "  (no reply)")

            elif up == "GET":
                reply = send_recv(sock, "GET")
                print(f"  MCU: {reply}" if reply else "  (no reply)")

            else:
                print(f"  Unknown command '{raw}'. Type 'help'.")

    finally:
        sock.close()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
