import argparse
import socket
import sys
import termios
import tty

KEY_MAP = {
    "w": "FORWARD",
    "a": "LEFT",
    "d": "RIGHT",
    "s": "STOP",
}


def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main():
    parser = argparse.ArgumentParser(description="TCP keyboard controller (w/a/s/d)")
    parser.add_argument("host", nargs="?", default="127.0.0.1")
    parser.add_argument("port", nargs="?", type=int, default=5000)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.host, args.port))
    print("Connected to {}:{}".format(args.host, args.port))
    print("Controls:  w=FORWARD  d=LEFT  a=RIGHT  s=STOP   (q or Ctrl-C to quit)")

    try:
        while True:
            key = read_key().lower()
            if key in ("q", "\x03"):
                break
            cmd = KEY_MAP.get(key)
            if cmd is None:
                continue
            payload = (cmd + "\n").encode("utf-8")
            sock.sendall(payload)
            print("TX: {}".format(cmd))
    finally:
        sock.close()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
