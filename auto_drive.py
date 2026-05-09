#!/usr/bin/env python3
import socket
import sys
import time
import threading
import argparse

# Global variable to store the latest yaw value received from the car
current_yaw = 0.0

def reader_thread(sock):
    global current_yaw
    while True:
        try:
            data = sock.recv(256)
            if not data:
                break
            
            # The car sends data like: F=10.0,L=20.0,R=30.0,PITCH=0.00,ROLL=0.00,YAW=90.00\r\n
            text = data.decode("utf-8", errors="replace").strip()
            # It might send multiple lines in one TCP packet, so we split by \n
            for line in text.split('\n'):
                line = line.strip()
                if "YAW=" in line:
                    parts = line.split(',')
                    for p in parts:
                        if p.startswith("YAW="):
                            try:
                                current_yaw = float(p.split('=')[1])
                            except ValueError:
                                pass
        except OSError:
            break

def send_cmd(sock, cmd):
    print(f"TX: {cmd}")
    sock.sendall((cmd + "\n").encode("utf-8"))

def main():
    parser = argparse.ArgumentParser(description="Auto Drive Script using Yaw")
    parser.add_argument("host", help="IP address of the ESP8266")
    parser.add_argument("--port", type=int, default=1234)
    parser.add_argument("--speed", type=int, default=70, help="Motor speed (0-100)")
    parser.add_argument("--time_50cm", type=float, default=1.5, help="Time in seconds to move 50cm (adjust as needed based on your floor/battery)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to {args.host}:{args.port}...")
    try:
        sock.connect((args.host, args.port))
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    print("Connected!")
    
    # Start thread to constantly read telemetry and update current_yaw
    t = threading.Thread(target=reader_thread, args=(sock,), daemon=True)
    t.start()

    # Wait a moment for the first telemetry to arrive
    print("Waiting 1s for sensors to stabilize...")
    time.sleep(1)
    
    try:
        # --- 1. Move forward 50 cm ---
        print("\n--- 1. Moving forward ~50 cm ---")
        send_cmd(sock, f"B {args.speed}")
        time.sleep(args.time_50cm)
        send_cmd(sock, "STOP")
        time.sleep(1)

        # --- 2. Rotate Right 90 degrees ---
        print("\n--- 2. Rotating RIGHT 90 degrees ---")
        start_yaw = current_yaw
        target_yaw = start_yaw - 90.0
        print(f"Start Yaw: {start_yaw:.2f}, Target Yaw: {target_yaw:.2f}")
        
        # Turn right (Left forward, Right backward)
        send_cmd(sock, f"L {args.speed}")
        send_cmd(sock, f"R {-args.speed}")
        
        # Wait until current_yaw reaches the target (yaw becomes more negative when turning right)
        while current_yaw > target_yaw:
            time.sleep(0.01)
        
        send_cmd(sock, "STOP")
        print(f"Finished rotation. Final Yaw: {current_yaw:.2f}")
        time.sleep(1)

        # --- 3. Move forward 50 cm ---
        print("\n--- 3. Moving forward ~50 cm ---")
        send_cmd(sock, f"B {args.speed}")
        time.sleep(args.time_50cm)
        send_cmd(sock, "STOP")
        time.sleep(1)

        # --- 4. Rotate Left 90 degrees ---
        print("\n--- 4. Rotating LEFT 90 degrees ---")
        start_yaw = current_yaw
        target_yaw = start_yaw + 90.0
        print(f"Start Yaw: {start_yaw:.2f}, Target Yaw: {target_yaw:.2f}")
        
        # Turn left (Left backward, Right forward)
        send_cmd(sock, f"L {-args.speed}")
        send_cmd(sock, f"R {args.speed}")
        
        # Wait until current_yaw reaches the target (yaw becomes more positive when turning left)
        while current_yaw < target_yaw:
            time.sleep(0.01)
        
        send_cmd(sock, "STOP")
        print(f"Finished rotation. Final Yaw: {current_yaw:.2f}")
        time.sleep(1)

        # --- 5. Move forward 50 cm ---
        print("\n--- 5. Moving forward ~50 cm ---")
        send_cmd(sock, f"B {args.speed}")
        time.sleep(args.time_50cm)
        send_cmd(sock, "STOP")
        time.sleep(1)
        
        print("\nSequence Complete!")

    except KeyboardInterrupt:
        print("\nInterrupted by user, stopping car...")
        send_cmd(sock, "STOP")
    finally:
        try:
            send_cmd(sock, "STOP")
        except:
            pass
        sock.close()

if __name__ == "__main__":
    main()
