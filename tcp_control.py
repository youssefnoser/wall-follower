"""
Interactive keyboard controller using the per-motor PWM protocol.

Pairs with src/wifi_control.c (and src/pwm_wifi_test_main.c).

Keys:
    w  forward at current speed   (B  <speed>)
    s  backward at current speed  (B -<speed>)
    a  tank-turn left             (L -<speed>, R  <speed>)
    d  tank-turn right            (L  <speed>, R -<speed>)
    space / x  stop               (STOP)
    +/=  speed +10                -/_  speed -10
    ?    query current duty
    q / Ctrl-C  quit

Usage:  python3 tcp_control.py <stm32-ip> [port]
"""

import argparse
import socket
import sys
import termios
import threading
import tty


def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def reader(sock):
    while True:
        try:
            data = sock.recv(256)
        except OSError:
            return
        if not data:
            sys.stdout.write("\r[server closed]\n")
            sys.stdout.flush()
            return
        sys.stdout.write("\r< " + data.decode("utf-8", errors="replace"))
        sys.stdout.flush()


def send(sock, cmd):
    sock.sendall((cmd + "\n").encode("utf-8"))
    sys.stdout.write("\rTX: {}\n".format(cmd))
    sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description="TCP keyboard PWM controller")
    parser.add_argument("host", nargs="?", default="127.0.0.1")
    parser.add_argument("port", nargs="?", type=int, default=1234)
    parser.add_argument("--speed", type=int, default=70,
                        help="initial PWM duty 0..100 (default 70)")
    args = parser.parse_args()

    speed = max(0, min(100, args.speed))

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.host, args.port))
    print("Connected to {}:{}".format(args.host, args.port))
    print("Keys: w/a/s/d move | space=STOP | +/- speed | ?=query | q=quit")
    print("Initial speed: {}%".format(speed))

    threading.Thread(target=reader, args=(sock,), daemon=True).start()

    try:
        while True:
            key = read_key()
            if key in ("q", "\x03"):
                break

            k = key.lower()
            if k == "w":
                send(sock, "B {}".format(speed))
            elif k == "s":
                send(sock, "B {}".format(-speed))
            elif k == "a":
                send(sock, "L {}".format(-speed))
                send(sock, "R {}".format(speed))
            elif k == "d":
                send(sock, "L {}".format(speed))
                send(sock, "R {}".format(-speed))
            elif key in (" ", "x"):
                send(sock, "STOP")
            elif key in ("+", "="):
                speed = min(100, speed + 10)
                sys.stdout.write("\rspeed = {}%\n".format(speed))
                sys.stdout.flush()
            elif key in ("-", "_"):
                speed = max(0, speed - 10)
                sys.stdout.write("\rspeed = {}%\n".format(speed))
                sys.stdout.flush()
            elif key == "?":
                send(sock, "?")
    finally:
        try:
            sock.sendall(b"STOP\n")
        except OSError:
            pass
        sock.close()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
