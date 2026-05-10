"""
robot_ui.py – Tkinter GUI for the STM32 corridor-centering robot.

TCP commands (port 1234):
  PID=Kp,Ki,Kd   update PID gains
  MB=<cm>         set dead-band margin
  GET             query current params + sensor readings
  DATA            query sensor readings only (fast poll)
  STOP            halt motors
  GO              resume motors

Usage:
  python3 robot_ui.py [robot-ip] [port]
"""

import queue
import socket
import sys
import threading
import tkinter as tk
from tkinter import font as tkfont
from tkinter import scrolledtext, ttk

DEFAULT_PORT = 1234
RECV_TIMEOUT = 2.0


# ---------------------------------------------------------------------------
# Network layer – runs in a background thread
# ---------------------------------------------------------------------------
class RobotConnection:
    def __init__(self, log_cb):
        self._sock = None
        self._lock = threading.Lock()
        self._log = log_cb

    def send_raw(self, cmd: str) -> str:
        """Send a command and return the raw reply (caller handles logging)."""
        with self._lock:
            sock = self._sock
        if sock is None:
            return ""
        try:
            sock.sendall((cmd.strip() + "\n").encode())
            try:
                return sock.recv(256).decode(errors="replace").strip()
            except socket.timeout:
                return ""
        except OSError:
            self.disconnect()
            return ""

    def connect(self, host: str, port: int) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(RECV_TIMEOUT)
            s.connect((host, port))
            with self._lock:
                self._sock = s
            return True
        except OSError as e:
            self._log(f"[ERROR] {e}")
            return False

    def disconnect(self):
        with self._lock:
            if self._sock:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._sock is not None

    def send(self, cmd: str):
        with self._lock:
            sock = self._sock
        if sock is None:
            self._log("[ERROR] Not connected.")
            return
        try:
            sock.sendall((cmd.strip() + "\n").encode())
            try:
                reply = sock.recv(256).decode(errors="replace").strip()
                self._log(f"MCU ← {reply}" if reply else "MCU ← (no reply)")
            except socket.timeout:
                self._log("MCU ← (timeout)")
        except OSError as e:
            self._log(f"[ERROR] Send failed: {e}")
            self.disconnect()


# ---------------------------------------------------------------------------
# Main GUI
# ---------------------------------------------------------------------------
class RobotUI(tk.Tk):
    def __init__(self, host, port: int):
        super().__init__()
        self.title("Robot Corridor-Centering Tuner")
        self.resizable(False, False)
        self._port_default = port
        self._host_default = host or ""
        self._log_queue: queue.Queue = queue.Queue()
        self._conn = RobotConnection(self._enqueue_log)
        self._live_send = tk.BooleanVar(value=False)
        self._auto_poll = tk.BooleanVar(value=False)

        self._build_ui()
        self._poll_log()
        self._schedule_sensor_poll()

    # -----------------------------------------------------------------------
    def _build_ui(self):
        pad = {"padx": 8, "pady": 4}

        # ── Connection bar ──────────────────────────────────────────────────
        conn_frame = ttk.LabelFrame(self, text="Connection")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky="ew", **pad)

        ttk.Label(conn_frame, text="IP:").grid(row=0, column=0, **pad)
        self._ip_var = tk.StringVar(value=self._host_default)
        ttk.Entry(conn_frame, textvariable=self._ip_var, width=16).grid(row=0, column=1, **pad)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=2, **pad)
        self._port_var = tk.IntVar(value=self._port_default)
        ttk.Spinbox(conn_frame, textvariable=self._port_var,
                    from_=1, to=65535, width=6).grid(row=0, column=3, **pad)

        self._conn_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connect)
        self._conn_btn.grid(row=0, column=4, **pad)

        self._status_label = ttk.Label(conn_frame, text="● Disconnected",
                                       foreground="red")
        self._status_label.grid(row=0, column=5, **pad)

        # ── PID sliders ─────────────────────────────────────────────────────
        pid_frame = ttk.LabelFrame(self, text="PID Gains")
        pid_frame.grid(row=1, column=0, sticky="nsew", **pad)

        self._kp = self._make_slider(pid_frame, "Kp", -5.0, 5.0, 1.5, row=0)
        self._ki = self._make_slider(pid_frame, "Ki", -1.0, 1.0, 0.0, row=1)
        self._kd = self._make_slider(pid_frame, "Kd", -3.0, 3.0, 0.8, row=2)

        ttk.Checkbutton(pid_frame, text="Live send", variable=self._live_send).grid(
            row=3, column=0, columnspan=3, pady=4)

        ttk.Button(pid_frame, text="Send PID", command=self._send_pid).grid(
            row=4, column=0, columnspan=3, sticky="ew", padx=8, pady=4)

        # ── Margin slider ───────────────────────────────────────────────────
        mb_frame = ttk.LabelFrame(self, text="Dead-band Margin")
        mb_frame.grid(row=2, column=0, sticky="nsew", **pad)

        self._mb = self._make_slider(mb_frame, "MB (cm)", 0.0, 10.0, 2.5, row=0)
        ttk.Button(mb_frame, text="Send MB", command=self._send_mb).grid(
            row=1, column=0, columnspan=3, sticky="ew", padx=8, pady=4)

        # ── Speed slider ────────────────────────────────────────────────────
        spd_frame = ttk.LabelFrame(self, text="Base Speed")
        spd_frame.grid(row=3, column=0, sticky="nsew", **pad)

        self._speed = self._make_slider(spd_frame, "Speed", 0, 100, 35, row=0)
        self._speed.trace_add("write", lambda *_: self._on_speed_slider())
        ttk.Button(spd_frame, text="Send Speed", command=self._send_speed).grid(
            row=1, column=0, columnspan=3, sticky="ew", padx=8, pady=4)

        # ── Left motor trim ─────────────────────────────────────────────────
        trim_frame = ttk.LabelFrame(self, text="Left Motor Trim")
        trim_frame.grid(row=4, column=0, sticky="nsew", **pad)

        self._trim = self._make_slider(trim_frame, "Trim", -50, 50, 5, row=0)
        self._trim.trace_add("write", lambda *_: self._on_trim_slider())
        ttk.Button(trim_frame, text="Send Trim", command=self._send_trim).grid(
            row=1, column=0, columnspan=3, sticky="ew", padx=8, pady=4)

        # ── Control buttons ─────────────────────────────────────────────────
        ctrl_frame = ttk.LabelFrame(self, text="Control")
        ctrl_frame.grid(row=5, column=0, sticky="nsew", **pad)

        stop_btn = tk.Button(ctrl_frame, text="⬛  STOP", bg="#c0392b", fg="white",
                             font=("", 12, "bold"), command=self._send_stop,
                             activebackground="#96281b", width=12)
        stop_btn.grid(row=0, column=0, padx=8, pady=6)

        go_btn = tk.Button(ctrl_frame, text="▶  GO", bg="#27ae60", fg="white",
                           font=("", 12, "bold"), command=self._send_go,
                           activebackground="#1e8449", width=12)
        go_btn.grid(row=0, column=1, padx=8, pady=6)

        ttk.Button(ctrl_frame, text="GET params", command=self._send_get).grid(
            row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=4)

        # ── Remote control ──────────────────────────────────────────────────
        rc_frame = ttk.LabelFrame(self, text="Remote Control  (W/A/S/D or arrow keys)")
        rc_frame.grid(row=0, column=2, rowspan=5, sticky="nsew", **pad)

        btn_cfg = dict(width=6, font=("", 13, "bold"))

        self._mode_var = tk.StringVar(value="AUTO")
        mode_row = ttk.Frame(rc_frame)
        mode_row.grid(row=0, column=0, columnspan=3, pady=(6, 2))
        ttk.Label(mode_row, text="Mode:").pack(side="left", padx=4)
        self._btn_auto   = tk.Button(mode_row, text="AUTO", width=7,
                                     bg="#2980b9", fg="white", font=("", 10, "bold"),
                                     command=self._cmd_auto)
        self._btn_manual = tk.Button(mode_row, text="MANUAL", width=7,
                                     bg="#7f8c8d", fg="white", font=("", 10, "bold"),
                                     command=self._cmd_manual)
        self._btn_auto.pack(side="left", padx=3)
        self._btn_manual.pack(side="left", padx=3)

        tk.Button(rc_frame, text="↑\nFWD",  **btn_cfg, command=self._cmd_fwd).grid(
            row=1, column=1, padx=4, pady=4)
        tk.Button(rc_frame, text="←\nLEFT", **btn_cfg, command=self._cmd_left).grid(
            row=2, column=0, padx=4, pady=4)
        tk.Button(rc_frame, text="■\nSTOP", **btn_cfg, bg="#c0392b", fg="white",
                  command=self._cmd_stop_rc).grid(row=2, column=1, padx=4, pady=4)
        tk.Button(rc_frame, text="→\nRIGHT", **btn_cfg, command=self._cmd_right).grid(
            row=2, column=2, padx=4, pady=4)
        tk.Button(rc_frame, text="↓\nBWD",  **btn_cfg, command=self._cmd_bwd).grid(
            row=3, column=1, padx=4, pady=4)

        # keyboard bindings
        for key, fn in (("<Up>",    self._cmd_fwd),
                        ("<Down>",  self._cmd_bwd),
                        ("<Left>",  self._cmd_left),
                        ("<Right>", self._cmd_right),
                        ("w",       self._cmd_fwd),
                        ("s",       self._cmd_bwd),
                        ("a",       self._cmd_left),
                        ("d",       self._cmd_right),
                        ("<space>", self._cmd_stop_rc)):
            self.bind(key, lambda e, f=fn: f())

        # ── Sensor data ─────────────────────────────────────────────────────
        sensor_frame = ttk.LabelFrame(self, text="Sensor Readings")
        sensor_frame.grid(row=6, column=0, columnspan=2, sticky="ew", **pad)

        big = ("Courier", 16, "bold")
        self._lbl_left  = tk.Label(sensor_frame, text="Left:  --- cm", font=big,
                                   fg="#2980b9", anchor="w", width=20)
        self._lbl_front = tk.Label(sensor_frame, text="Front: --- cm", font=big,
                                   fg="#e67e22", anchor="w", width=20)
        self._lbl_right = tk.Label(sensor_frame, text="Right: --- cm", font=big,
                                   fg="#27ae60", anchor="w", width=20)
        self._lbl_left.grid (row=0, column=0, padx=12, pady=2, sticky="w")
        self._lbl_front.grid(row=1, column=0, padx=12, pady=2, sticky="w")
        self._lbl_right.grid(row=2, column=0, padx=12, pady=2, sticky="w")

        poll_ctrl = ttk.Frame(sensor_frame)
        poll_ctrl.grid(row=0, column=1, rowspan=3, padx=8, sticky="ns")
        ttk.Checkbutton(poll_ctrl, text="Auto-poll (500 ms)",
                        variable=self._auto_poll).pack(pady=4)
        ttk.Button(poll_ctrl, text="Poll now",
                   command=self._poll_sensors_now).pack(pady=4, fill="x")

        # ── Log ─────────────────────────────────────────────────────────────
        log_frame = ttk.LabelFrame(self, text="Log")
        log_frame.grid(row=1, column=1, rowspan=6, sticky="nsew", **pad)

        self._log_box = scrolledtext.ScrolledText(log_frame, width=42, height=22,
                                                  state="disabled", font=("Courier", 9))
        self._log_box.pack(fill="both", expand=True, padx=4, pady=4)

        ttk.Button(log_frame, text="Clear log", command=self._clear_log).pack(pady=2)

    # -----------------------------------------------------------------------
    def _make_slider(self, parent, label: str, lo: float, hi: float,
                     init: float, row: int):
        var = tk.DoubleVar(value=init)

        ttk.Label(parent, text=label, width=7).grid(row=row, column=0, padx=4)

        slider = ttk.Scale(parent, from_=lo, to=hi, orient="horizontal",
                           variable=var, length=200,
                           command=lambda _: self._on_slider(var))
        slider.grid(row=row, column=1, padx=4)

        entry = ttk.Entry(parent, textvariable=var, width=7)
        entry.grid(row=row, column=2, padx=4)
        entry.bind("<Return>", lambda _: self._on_slider(var))

        return var

    def _on_slider(self, var: tk.DoubleVar):
        if self._live_send.get():
            if var in (self._kp, self._ki, self._kd):
                self._send_pid()
            elif var is self._mb:
                self._send_mb()

    def _on_speed_slider(self):
        if self._live_send.get():
            self._send_speed()

    def _on_trim_slider(self):
        if self._live_send.get():
            self._send_trim()

    # -----------------------------------------------------------------------
    def _toggle_connect(self):
        if self._conn.connected:
            self._conn.disconnect()
            self._conn_btn.config(text="Connect")
            self._status_label.config(text="● Disconnected", foreground="red")
            self._enqueue_log("Disconnected.")
        else:
            host = self._ip_var.get().strip()
            port = self._port_var.get()
            if not host:
                self._enqueue_log("[ERROR] Enter robot IP first.")
                return
            self._enqueue_log(f"Connecting to {host}:{port} …")
            self.update_idletasks()
            ok = self._conn.connect(host, port)
            if ok:
                self._conn_btn.config(text="Disconnect")
                self._status_label.config(text="● Connected", foreground="green")
                self._enqueue_log("Connected.")
                threading.Thread(target=self._conn.send, args=("GET",),
                                 daemon=True).start()
            else:
                self._status_label.config(text="● Failed", foreground="red")

    # -----------------------------------------------------------------------
    def _send_pid(self):
        kp = round(self._kp.get(), 3)
        ki = round(self._ki.get(), 3)
        kd = round(self._kd.get(), 3)
        cmd = f"PID={kp},{ki},{kd}"
        self._enqueue_log(f"→ {cmd}")
        threading.Thread(target=self._conn.send, args=(cmd,), daemon=True).start()

    def _send_mb(self):
        mb = round(self._mb.get(), 2)
        cmd = f"MB={mb}"
        self._enqueue_log(f"→ {cmd}")
        threading.Thread(target=self._conn.send, args=(cmd,), daemon=True).start()

    def _send_get(self):
        self._enqueue_log("→ GET")
        def _do():
            reply = self._conn.send_raw("GET")
            if reply:
                self._enqueue_log(f"MCU ← {reply}")
                self.after(0, self._parse_sensor_reply, reply)
            else:
                self._enqueue_log("MCU ← (no reply)")
        threading.Thread(target=_do, daemon=True).start()

    def _send_speed(self):
        spd = int(round(self._speed.get()))
        cmd = f"SPEED={spd}"
        self._enqueue_log(f"→ {cmd}")
        threading.Thread(target=self._conn.send, args=(cmd,), daemon=True).start()

    def _send_trim(self):
        tr = int(round(self._trim.get()))
        cmd = f"TRIM={tr}"
        self._enqueue_log(f"→ {cmd}")
        threading.Thread(target=self._conn.send, args=(cmd,), daemon=True).start()

    def _send_stop(self):
        self._enqueue_log("→ STOP")
        threading.Thread(target=self._conn.send, args=("STOP",), daemon=True).start()

    def _send_go(self):
        self._enqueue_log("→ GO")
        threading.Thread(target=self._conn.send, args=("GO",), daemon=True).start()

    # -----------------------------------------------------------------------
    def _send_rc(self, cmd: str):
        self._enqueue_log(f"→ {cmd}")
        threading.Thread(target=self._conn.send, args=(cmd,), daemon=True).start()

    def _cmd_fwd(self):    self._send_rc("FWD")
    def _cmd_bwd(self):    self._send_rc("BWD")
    def _cmd_left(self):   self._send_rc("LEFT")
    def _cmd_right(self):  self._send_rc("RIGHT")
    def _cmd_stop_rc(self):
        self._enqueue_log("→ STOP")
        threading.Thread(target=self._conn.send, args=("STOP",), daemon=True).start()

    def _cmd_auto(self):
        self._btn_auto.config(bg="#2980b9")
        self._btn_manual.config(bg="#7f8c8d")
        self._send_rc("AUTO")

    def _cmd_manual(self):
        self._btn_manual.config(bg="#8e44ad")
        self._btn_auto.config(bg="#7f8c8d")
        self._send_rc("FWD")   # enter manual + start moving immediately

    # -----------------------------------------------------------------------
    def _parse_sensor_reply(self, reply: str):
        """Extract L=, F=, R=, SPD= from any MCU reply and update widgets."""
        import re
        m = re.search(r"L=([\d.]+)", reply)
        if m:
            self._lbl_left.config(text=f"Left:  {float(m.group(1)):.1f} cm")
        m = re.search(r"F=([\d.]+)", reply)
        if m:
            self._lbl_front.config(text=f"Front: {float(m.group(1)):.1f} cm")
        m = re.search(r"R=([\d.]+)", reply)
        if m:
            self._lbl_right.config(text=f"Right: {float(m.group(1)):.1f} cm")
        m = re.search(r"SPD=(\d+)", reply)
        if m:
            self._speed.set(int(m.group(1)))
        m = re.search(r"TRIM=(-?\d+)", reply)
        if m:
            self._trim.set(int(m.group(1)))

    def _poll_sensors_now(self):
        def _do():
            reply = self._conn.send_raw("DATA")
            if reply:
                self._enqueue_log(f"MCU ← {reply}")
                self.after(0, self._parse_sensor_reply, reply)
        threading.Thread(target=_do, daemon=True).start()

    def _schedule_sensor_poll(self):
        if self._auto_poll.get() and self._conn.connected:
            self._poll_sensors_now()
        self.after(500, self._schedule_sensor_poll)

    # -----------------------------------------------------------------------
    def _enqueue_log(self, msg: str):
        self._log_queue.put(msg)

    def _poll_log(self):
        try:
            while True:
                msg = self._log_queue.get_nowait()
                self._log_box.config(state="normal")
                self._log_box.insert("end", msg + "\n")
                self._log_box.see("end")
                self._log_box.config(state="disabled")
        except queue.Empty:
            pass
        self.after(100, self._poll_log)

    def _clear_log(self):
        self._log_box.config(state="normal")
        self._log_box.delete("1.0", "end")
        self._log_box.config(state="disabled")


# ---------------------------------------------------------------------------
def main():
    host = sys.argv[1] if len(sys.argv) > 1 else None
    port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT
    app = RobotUI(host, port)
    app.mainloop()


if __name__ == "__main__":
    main()
