"""
pid_tuner.py – Premium Stable Dashboard.
Uses prompt_toolkit Application for a flicker-free, side-by-side TUI.
"""

import argparse
import math
import socket
import sys
import time
import threading
from collections import deque
from datetime import datetime

from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.columns import Columns
from rich.align import Align
from rich.live import Live

from prompt_toolkit.application import Application
from prompt_toolkit.layout.containers import HSplit, VSplit, Window, WindowAlign, FloatContainer
from prompt_toolkit.layout.controls import FormattedTextControl
from prompt_toolkit.layout.layout import Layout
from prompt_toolkit.widgets import Frame, TextArea
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.styles import Style
from prompt_toolkit.patch_stdout import patch_stdout
from prompt_toolkit.formatted_text import HTML

# ---------------------------------------------------------------------------
# Constants & Settings
# ---------------------------------------------------------------------------
DEFAULT_PORT = 1234
RECV_TIMEOUT = 2.0
SENSITIVITY = 20.0
LOG_MAX = 50

# ---------------------------------------------------------------------------
# State Management
# ---------------------------------------------------------------------------
class RobotState:
    def __init__(self, host):
        self.data = {}
        self.derived = {}
        self.logs = deque(maxlen=LOG_MAX)
        self.sensor_logs = deque(maxlen=100)
        self.connected = False
        self.last_update = 0
        self.ip = host
        self.lock = threading.Lock()
        self.app = None # To be set by main
        self.log_file = None

    def open_log(self, filename="tuner_session.log"):
        with self.lock:
            self.log_file = open(filename, "w", encoding="utf-8")
            self.log_file.write(f"--- SESSION START: {datetime.now()} ---\n")
            self.log_file.flush()

    def close(self):
        with self.lock:
            if self.log_file:
                self.log_file.write(f"--- SESSION END: {datetime.now()} ---\n")
                self.log_file.close()
                self.log_file = None

    def log(self, msg, type="info"):
        ts = datetime.now().strftime("%H:%M:%S")
        color = "white"
        if type == "mcu": color = "cyan"
        elif type == "error": color = "red"
        elif type == "cmd": color = "yellow"
        elif type == "status": color = "green"
        
        # Use rich tags for log storage
        with self.lock:
            self.logs.append(f"[{ts}] [{color}]{msg}[/{color}]")
            # Write to file (clean)
            if self.log_file:
                self.log_file.write(f"[{ts}] [CMD:{type}] {msg}\n")
                self.log_file.flush()
        
        if self.app:
            self.app.invalidate()

    def update(self, response):
        if not response:
            with self.lock:
                self.connected = False
            if self.app: self.app.invalidate()
            return
        
        if response.startswith("["):
            self.log(response, "mcu")

        parsed = {}
        # Support both k=v (GET response) and k:v ([FSM] logs)
        for token in response.split():
            if "=" in token or ":" in token:
                sep = "=" if "=" in token else ":"
                k, _, v = token.partition(sep)
                try:
                    # Strip any non-numeric noise from the value
                    v_clean = "".join(c for c in v if c.isdigit() or c in ".-+")
                    if v_clean:
                        parsed[k] = float(v_clean)
                except ValueError:
                    pass
        
        # Heading clamping [-180, 180]
        for k in ["ANG", "TGT"]:
            if k in parsed:
                parsed[k] = (parsed[k] + 180) % 360 - 180
        
        with self.lock:
            self.data.update(parsed)
            self.derive()
            
            # Log all parameters to file
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-4]
            param_str = " ".join([f"{k}:{v}" for k, v in parsed.items()])
            if self.log_file and param_str:
                self.log_file.write(f"[{ts}] {param_str}\n")
                self.log_file.flush()

            # Update TUI sensor logs specifically for the sensors panel
            if "L" in parsed and "R" in parsed and "F" in parsed:
                log_msg = f"[{ts}] [blue]L:{parsed['L']:4.1f}[/] [cyan]R:{parsed['R']:4.1f}[/] [yellow]F:{parsed['F']:4.1f}[/]"
                self.sensor_logs.append(log_msg)

            self.last_update = time.time()
            self.connected = True
        
        if self.app:
            self.app.invalidate()

    def derive(self):
        d = self.data
        L = d.get("L", 0.0)
        R = d.get("R", 0.0)
        mb = d.get("MB", 2.0)
        kp = d.get("KP", 0.0)
        spd = d.get("SPD", 42.0)
        lfac = d.get("LFAC", 1.0)
        rfac = d.get("RFAC", 1.0)

        wall_error = R - L 
        if -mb < wall_error < mb:
            wall_error = 0.0

        # In the new AUTO mode, centering effort is driven by heading bias
        # Bias = -wall_error * WG
        # Correction = HKP * Bias (approx, ignoring D term for visualization)
        wg = d.get("WG", 1.5)
        hkp = d.get("HKP", 2.0)
        
        bias = -wall_error * wg
        corr_est = hkp * bias

        raw_l = spd - corr_est
        raw_r = spd + corr_est

        self.derived = {
            "wall_error": wall_error,
            "bias": bias,
            "corr_est": corr_est,
            "pwm_l": max(0, min(100, raw_l * lfac)),
            "pwm_r": max(0, min(100, raw_r * rfac))
        }

# ---------------------------------------------------------------------------
# UI Helpers (Rich -> PromptToolkit)
# ---------------------------------------------------------------------------
from prompt_toolkit.formatted_text import ANSI

def get_rich_console_content(renderable, width=60):
    console = Console(width=width, force_terminal=True, color_system="standard")
    with console.capture() as capture:
        console.print(renderable)
    return ANSI(capture.get())

# ---------------------------------------------------------------------------
# Layout Construction
# ---------------------------------------------------------------------------
def create_ui(state):
    # Header Control
    def get_header_text():
        with state.lock:
            status = "<style fg='ansigreen' bgcolor='ansiblue'>ONLINE</style>" if state.connected else "<style fg='ansired' bgcolor='ansiblue'>OFFLINE</style>"
            ts = time.strftime('%H:%M:%S', time.localtime(state.last_update)) if state.last_update else "--:--:--"
        
        return HTML(
            f"<style bg='ansiblue' fg='white'> "
            f"<b>STM32 ROBOT TUNER</b> | IP: {state.ip} | Status: {status} | Last Update: {ts} "
            f"</style>"
        )

    # Dashboard Control (Left)
    def get_dashboard_content():
        # Get actual window width to avoid clipping
        width = dashboard_window.render_info.window_width if dashboard_window.render_info else 60
        if width < 40: width = 40 # Minimum sanity
        
        with state.lock:
            d = state.data.copy()
            drv = state.derived.copy()
            connected = state.connected
        
        # Sensors
        sensor_table = Table.grid(expand=True)
        sensor_table.add_column(width=8)
        sensor_table.add_column(justify="right")
        
        L, R, F = d.get("L", 0.0), d.get("R", 0.0), d.get("F", 0.0)
        def bar(val, max_val=80):
            bar_w = max(5, width // 6)
            filled = int(min(1.0, val / max_val) * bar_w)
            return "[" + "█" * filled + "░" * (bar_w - filled) + "]"

        sensor_table.add_row("[blue]Left[/]", f"{L:4.1f} {bar(L)}")
        sensor_table.add_row("[cyan]Right[/]", f"{R:4.1f} {bar(R)}")
        sensor_table.add_row("[yellow]Front[/]", f"{F:4.1f} {bar(F, 120)}")
        
        # PID
        pid_table = Table.grid(expand=True)
        pid_table.add_row(f"KP/I/D", f"[bold cyan]{d.get('KP',0):.2f}/{d.get('KI',0):.3f}/{d.get('KD',0):.2f}[/]")
        pid_table.add_row(f"Speed", f"{int(d.get('SPD',0))}% (M:{d.get('MB',0):.1f})")
        pid_table.add_row(f"Scale", f"L:{d.get('LFAC',1):.2f} R:{d.get('RFAC',1):.2f}")
        pid_table.add_row(f"Wall Err", f"{drv.get('wall_error',0):+5.1f} cm")
        pid_table.add_row(f"Wall Gain", f"[bold green]{d.get('WG',0):.2f}[/]")
        pid_table.add_row(f"Bias Est", f"{drv.get('bias',0):+5.1f}°")
        
        # IMU
        imu_table = Table.grid(expand=True)
        imu_table.add_row("Angle", f"[bold yellow]{d.get('ANG',0):+6.1f}°[/]")
        imu_table.add_row("Target", f"{d.get('TGT',0):+6.1f}°")
        imu_table.add_row("HK P/D", f"{d.get('HKP',0):.1f}/{d.get('HKD',0):.2f}")

        # Final Dashboard Panel
        layout = Columns([
            Panel(sensor_table, title="[blue]Sensors[/]", border_style="blue", padding=(0,1)),
            Panel(pid_table, title="[green]Control[/]", border_style="green", padding=(0,1)),
            Panel(imu_table, title="[magenta]IMU[/]", border_style="magenta", padding=(0,1))
        ], expand=True)

        run_status = "[bold green]RUNNING[/]" if d.get("RUN", 0) else "[bold red]STOPPED[/]"
        
        full_panel = Panel(
            layout, 
            title=f" [bold white]Robot Status[/] - {run_status} ", 
            border_style="bright_white",
            padding=(0,1),
            expand=True
        )
        
        return get_rich_console_content(full_panel, width=width)

    # Command Log Control
    def get_log_content():
        width = log_window.render_info.window_width if log_window.render_info else 50
        if width < 30: width = 30
        with state.lock:
            logs = list(state.logs)
        log_text = "\n".join(logs)
        panel = Panel(log_text, title="[bold cyan]Command Log[/]", border_style="cyan", padding=(0,1), expand=True)
        return get_rich_console_content(panel, width=width)

    # Sensor History Control
    def get_sensor_log_content():
        width = sensor_log_window.render_info.window_width if sensor_log_window.render_info else 50
        if width < 30: width = 30
        
        with state.lock:
            logs = list(state.sensor_logs)
        
        log_text = "\n".join(logs)
        panel = Panel(log_text, title="[bold yellow]Sensor History[/]", border_style="yellow", padding=(0,1), expand=True)
        return get_rich_console_content(panel, width=width)

    # UI Components
    header_window = Window(content=FormattedTextControl(get_header_text), height=1, align=WindowAlign.CENTER)
    
    dashboard_window = Window(content=FormattedTextControl(get_dashboard_content))
    log_window = Window(content=FormattedTextControl(get_log_content))
    sensor_log_window = Window(content=FormattedTextControl(get_sensor_log_content))
    
    input_field = TextArea(
        height=1,
        prompt="pid_tuner> ",
        multiline=False,
        wrap_lines=False,
    )

    body = VSplit([
        dashboard_window,
        Window(width=1, char="|", style="class:line"),
        HSplit([
            log_window,
            Window(height=1, char="-", style="class:line"),
            sensor_log_window
        ])
    ])

    help_line = Window(
        content=FormattedTextControl(HTML(
            "<style bg='ansigray' fg='black'> "
            "<b>F/B/L/R</b> Manual | <b>STOP/GO</b> | <b>WG=n</b> WallGain | <b>HKP=n</b> | <b>PID=k,i,d</b> | <b>q</b> Quit "
            "</style>"
        )),
        height=1
    )

    # Buttons Row
    def make_btn(label, cmd, color="gray"):
        def handler(mouse_event):
            if mouse_event.event_type == "MOUSE_UP":
                state.log(f"Clicked {label}", "cmd")
                # Trigger command (we need to pass this up or use a global-ish state)
                # For now, we'll use a trick: the app will handle it via a shared queue or direct call
                app_cmd_queue.append(cmd)
        
        control = FormattedTextControl(
            HTML(f"<style bg='ansi{color}' fg='white'>  {label}  </style>")
        )
        control.mouse_handler = handler
        return Window(
            content=control,
            height=1,
            dont_extend_width=True
        )

    buttons_row = VSplit([
        make_btn(" STOP ", "STOP", "red"),
        Window(width=1),
        make_btn(" GO (Auto) ", "GO", "green"),
        Window(width=2),
        make_btn(" FWD ", "FWD"),
        Window(width=1),
        make_btn(" BWD ", "BWD"),
        Window(width=1),
        make_btn(" LEFT ", "LEFT"),
        Window(width=1),
        make_btn(" RIGHT ", "RIGHT"),
        Window(width=2),
        make_btn(" GET ", "GET", "blue"),
    ], align=WindowAlign.CENTER)

    root_container = HSplit([
        header_window,
        body,
        buttons_row,
        help_line,
        input_field
    ])

    return root_container, input_field

# ---------------------------------------------------------------------------
# Communication logic
# ---------------------------------------------------------------------------
def send_recv(sock, cmd):
    try:
        sock.sendall((cmd.strip() + "\n").encode())
        data = b""
        while True:
            chunk = sock.recv(1024)
            data += chunk
            if b"\n" in chunk: break
        return data.decode(errors="replace").strip()
    except:
        return None

def monitor_loop(sock, state, stop_event):
    while not stop_event.is_set():
        reply = send_recv(sock, "GET")
        if reply:
            state.update(reply)
        else:
            state.update(None)
        time.sleep(0.5)

# ---------------------------------------------------------------------------
# Main Application
# ---------------------------------------------------------------------------
app_cmd_queue = deque()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("host", help="Robot IP address")
    parser.add_argument("port", type=int, default=DEFAULT_PORT, help="Port (default 1234)")
    args = parser.parse_args()

    state = RobotState(args.host)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(RECV_TIMEOUT)
    try:
        sock.connect((args.host, args.port))
        state.connected = True
        state.open_log() # Clear and start file logging only on success
    except Exception as e:
        print(f"Connection failed: {e}")
        return

    root, input_field = create_ui(state)
    kb = KeyBindings()

    def quick_cmd(cmd_text):
        up = cmd_text.upper()
        state.log(f"> {cmd_text} (hotkey)", "cmd")
        def run():
            reply = send_recv(sock, up)
            if reply:
                state.log(reply, "mcu")
                if "KP=" in reply: state.update(reply)
            else:
                state.log("No response", "error")
        threading.Thread(target=run, daemon=True).start()

    # Interactive Drive Keys
    @kb.add("up")
    @kb.add("w")
    def _(event): quick_cmd("FWD")

    @kb.add("down")
    @kb.add("s")
    def _(event): quick_cmd("BWD")

    @kb.add("left")
    @kb.add("a")
    def _(event): quick_cmd("LEFT")

    @kb.add("right")
    @kb.add("d")
    def _(event): quick_cmd("RIGHT")

    @kb.add("space")
    @kb.add("x")
    def _(event): quick_cmd("STOP")

    @kb.add("g")
    def _(event): quick_cmd("GO")

    @kb.add("c-c")
    @kb.add("q")
    def _(event):
        event.app.exit()

    app = Application(
        layout=Layout(root, focused_element=input_field),
        key_bindings=kb,
        full_screen=True,
        mouse_support=True,
        refresh_interval=0.2,
    )
    state.app = app

    def handle_input(buffer):
        cmd = buffer.text.strip()
        if not cmd: return
        
        up = cmd.upper()
        if up in ["Q", "QUIT", "EXIT"]:
            app.exit()
            return

        state.log(f"> {cmd}", "cmd")
        
        if up == "F": up = "FWD"
        elif up == "B": up = "BWD"
        elif up == "L": up = "LEFT"
        elif up == "R": up = "RIGHT"
        elif up == "S": up = "STOP"
        
        def run_cmd():
            reply = send_recv(sock, up)
            if reply:
                state.log(reply, "mcu")
                if any(x in reply for x in ["KP=", "SPD=", "WG=", "HKP=", "HKD="]):
                    state.update(reply)
            else:
                state.log("No response from MCU", "error")
        
        threading.Thread(target=run_cmd, daemon=True).start()
        buffer.reset()

    input_field.accept_handler = handle_input

    stop_event = threading.Event()
    mon_thread = threading.Thread(target=monitor_loop, args=(sock, state, stop_event), daemon=True)
    mon_thread.start()

    # Command Execution Loop (check queue)
    def check_queue():
        while not stop_event.is_set():
            if app_cmd_queue:
                cmd = app_cmd_queue.popleft()
                quick_cmd(cmd)
            time.sleep(0.05)
    
    threading.Thread(target=check_queue, daemon=True).start()

    try:
        app.run()
    finally:
        stop_event.set()
        state.close()
        sock.close()
        print("Disconnected.")

if __name__ == "__main__":
    main()
