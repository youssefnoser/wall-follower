"""
Interactive PWM tester client.

Pairs with src/pwm_wifi_test_main.c.  Type commands at the prompt:

    L 50        left motor 50% forward
    L -30       left motor 30% backward
    R 80        right motor 80% forward
    B 60        both motors 60%
    STOP        stop all
    ?           query current values
    q / quit    exit

Usage:  python3 tcp_pwm_test.py <stm32-ip> [port]
"""

import argparse
import socket
import sys
import threading


def reader(sock):
    while True:
        try:
            data = sock.recv(256)
        except OSError:
            return
        if not data:
            print("\n[server closed]")
            return
        sys.stdout.write("< " + data.decode("utf-8", errors="replace"))
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description="PWM tester TCP client")
    parser.add_argument("host")
    parser.add_argument("port", nargs="?", type=int, default=1234)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.host, args.port))
    print("Connected to {}:{}".format(args.host, args.port))
    print("Commands: 'L <pct>', 'R <pct>', 'B <pct>', 'STOP', '?', 'q' to quit")

    t = threading.Thread(target=reader, args=(sock,), daemon=True)
    t.start()

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            if line.lower() in ("q", "quit", "exit"):
                break
            sock.sendall((line + "\n").encode("utf-8"))
    finally:
        sock.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
