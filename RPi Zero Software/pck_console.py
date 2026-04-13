#!/usr/bin/env python3
"""
PCK-ST1 Control Console
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
UART-based graphical control interface for the PCK-ST1 device.

  Install deps:  pip install pyserial matplotlib
  Run:           python pck_st1_console.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import queue
import time
import serial
import serial.tools.list_ports
from collections import deque
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ════════════════════════════════════════════════════════════
#  ▶  CONFIGURATION  — edit this section to add/change commands
# ════════════════════════════════════════════════════════════

# ── Theme reference for default colors ───────────────────────
TH_ACCENT2 = "#7b2fff"
TH_SUCCESS = "#00e676"
TH_WARNING = "#ffb700"

# ── Simple one-click commands ────────────────────────────────
#   Each entry: ("Button Label", "command string to send", "Button Color")
SIMPLE_COMMANDS = [
    ("HOME", "$HOME", TH_SUCCESS),
    ("CALIBRATION", "$CALIBRATE", TH_SUCCESS),
    ("ALIGNMENT", "$ALIGN", TH_SUCCESS),
    ("PARK", "$MOVE:PARK", TH_WARNING),
    ("ECHO", "$ECHO", TH_ACCENT2),
    ("RPI ECHO", "$RPI:ECHO", TH_ACCENT2),
    ("RPI SHUTDOWN", "$RPI:SHUTDOWN", TH_WARNING),
]

# ── Toggle commands ──────────────────────────────────────────
#   Each entry: ("Base Label", "Command ON", "Command OFF")
TOGGLE_COMMANDS = [
    ("RPI POWER", "$RPI:START", "$RPI:STOP"),
    ("MOTORS", "$EN", "$DIS"),
    ("TRACKING", "$TRACKING:START", "$TRACKING:STOP"),
    ("CALIBRATION", "$CALIB:ON", "$CALIB:OFF"),
]

# ── Axis-move commands ───────────────────────────────────────
#   Template uses {angle} and {speed} placeholders.
MOVE_COMMANDS = [
    ("AZ",  "$MOVE:AZ:{angle}:{speed}"),
    ("ALT", "$MOVE:ALT:{angle}:{speed}"),
    ("RA",  "$MOVE:RA:{angle}:{speed}"),
    ("DEC", "$MOVE:DEC:{angle}:{speed}"),
]

# ── Telemetry ────────────────────────────────────────────────
TELEMETRY_COMMAND = "$TEL:FULL"

# Updated to match: $TEL:AZpos:ALTpos:RApos:DECpos:Voltage:RAcoordinates:DECcoordinates...
TELEMETRY_FIELDS = [
    "AZ",
    "ALT",
    "RA",
    "DEC",
    "VOLTAGE",
    "RA COORD",
    "DEC COORD",
]

# Displayed as text cards (Row 1: AZ, ALT, RA, DEC | Row 2: VOLTAGE, RA COORD, DEC COORD)
DISPLAY_ORDER = [
    "AZ",
    "ALT",
    "RA",
    "DEC",
    "VOLTAGE",
    "RA COORD",
    "DEC COORD",
]

# Displayed as graphs (excluding coordinates)
PLOT_ORDER = [
    "AZ",
    "ALT",
    "RA",
    "DEC",
    "VOLTAGE",
]

# Updated to match: ...Homed:Calibrated:gpsOK:gpsFixed:rpiReady:polarAligned:tracking:moveEnabled:fault
FLAG_FIELDS = [
    "HOMED",
    "CALIBRATED",
    "GPS OK",
    "GPS FIXED",
    "RPI READY",
    "POLAR ALIGNED",
    "TRACKING",
    "MOVE ENABLED",
    "FAULT",
]

# Adding FAULT so it renders as a red LED
ERROR_FLAGS = {"ERROR", "LIMIT_AZ", "LIMIT_ALT", "FAULT"}

TELEMETRY_RATES = {
    "0.1 Hz": 0.1,
    "1 Hz":   1.0,
    "5 Hz":  5.0,
}

DEFAULT_BAUD = 115200
BAUD_RATES   = [9600, 19200, 38400, 57600, 115200, 230400]

PLOT_HISTORY = 300

# ════════════════════════════════════════════════════════════
#  ▶  THEME
# ════════════════════════════════════════════════════════════
TH = {
    "bg":           "#0f0f17",
    "panel":        "#1a1a2e",
    "panel2":       "#16213e",
    "border":       "#2a2a4a",
    "accent":       "#00d4ff",
    "accent2":      "#7b2fff",
    "text":         "#dde1f0",
    "dim":          "#556080",
    "success":      "#00e676",
    "danger":       "#ff3d71",
    "warning":      "#ffb700",
    "led_on":       "#00e676",
    "led_off":      "#1e2235",
    "led_err":      "#ff3d71",
    "plot_bg":      "#0b0b14",
    "tx_color":     "#40c4ff",
    "rx_color":     "#69ff47",
    "tel_color":    "#37474f",
    "err_color":    "#ff3d71",
    "stat_color":   "#ffb700",
}

FONT_MONO  = ("Consolas", 9)
FONT_LABEL = ("Segoe UI", 9)
FONT_BOLD  = ("Segoe UI", 9, "bold")
FONT_HEAD  = ("Segoe UI", 11, "bold")
FONT_TITLE = ("Segoe UI", 14, "bold")
FONT_VALUE = ("Consolas", 13, "bold")

# ════════════════════════════════════════════════════════════
#  Serial worker thread
# ════════════════════════════════════════════════════════════

class SerialWorker(threading.Thread):
    def __init__(self, port, baud, rx_q, tx_q):
        super().__init__(daemon=True)
        self.port    = port
        self.baud    = baud
        self.rx_q    = rx_q
        self.tx_q    = tx_q
        self.running = False
        self._ser    = None

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.running = True
            self.rx_q.put(("STATUS", f"Connected → {self.port}  @{self.baud} baud"))
        except Exception as exc:
            self.rx_q.put(("ERROR", f"Cannot open {self.port}: {exc}"))
            return

        buf = ""
        while self.running:
            while not self.tx_q.empty():
                try:
                    cmd = self.tx_q.get_nowait()
                    self._ser.write((cmd + "\r\n").encode())
                    self.rx_q.put(("TX", cmd))
                except Exception as exc:
                    self.rx_q.put(("ERROR", f"TX: {exc}"))

            try:
                waiting = self._ser.in_waiting
                if waiting:
                    buf += self._ser.read(waiting).decode(errors="replace")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip("\r").strip()
                        if line:
                            self.rx_q.put(("RX", line))
            except Exception as exc:
                self.rx_q.put(("ERROR", f"RX: {exc}"))
                break

            time.sleep(0.005)

        if self._ser and self._ser.is_open:
            self._ser.close()
        self.rx_q.put(("STATUS", "Disconnected."))

    def stop(self):
        self.running = False

# ════════════════════════════════════════════════════════════
#  Telemetry poller thread
# ════════════════════════════════════════════════════════════

class TelPoller(threading.Thread):
    def __init__(self, tx_q, rate_var):
        super().__init__(daemon=True)
        self.tx_q     = tx_q
        self.rate_var = rate_var
        self.running  = False
        self.enabled  = False

    def run(self):
        self.running = True
        while self.running:
            if self.enabled:
                hz   = TELEMETRY_RATES.get(self.rate_var.get(), 1.0)
                self.tx_q.put(TELEMETRY_COMMAND)
                time.sleep(1.0 / hz)
            else:
                time.sleep(0.05)

    def stop(self):
        self.running = False

# ════════════════════════════════════════════════════════════
#  Main application
# ════════════════════════════════════════════════════════════

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PCK-ST1  ·  Control Console")
        self.geometry("1440x920")
        self.minsize(1200, 800)
        self.configure(bg=TH["bg"])

        # state
        self._worker    = None
        self._poller    = None
        self._rx_q      = queue.Queue()
        self._tx_q      = queue.Queue()
        self._connected = False
        self._t0        = time.time()
        self._toggle_states = {}

        # telemetry history
        self._t_buf   = deque(maxlen=PLOT_HISTORY)
        self._tel_buf = {f: deque(maxlen=PLOT_HISTORY) for f in TELEMETRY_FIELDS}
        self._tel_var = {f: tk.StringVar(value="—") for f in TELEMETRY_FIELDS}

        self._apply_style()
        self._build_topbar()
        self._build_body()
        self._poll_rx()

    # ── ttk style ──────────────────────────────────────────
    def _apply_style(self):
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure("TCombobox",
                    fieldbackground=TH["panel2"],
                    background=TH["panel"],
                    foreground=TH["text"],
                    selectbackground=TH["panel2"],
                    selectforeground=TH["text"],
                    bordercolor=TH["border"],
                    arrowcolor=TH["accent"])
        s.map("TCombobox", fieldbackground=[("readonly", TH["panel2"])])

    # ── Top bar ────────────────────────────────────────────
    def _build_topbar(self):
        bar = tk.Frame(self, bg=TH["panel"], height=54)
        bar.pack(fill=tk.X, padx=0, pady=0)
        bar.pack_propagate(False)

        stripe = tk.Frame(bar, bg=TH["accent"], width=4)
        stripe.pack(side=tk.LEFT, fill=tk.Y)

        tk.Label(bar, text="PCK-ST1", font=FONT_TITLE,
                 bg=TH["panel"], fg=TH["accent"]).pack(side=tk.LEFT, padx=(14, 6))
        tk.Label(bar, text="Control Console", font=("Segoe UI", 10),
                 bg=TH["panel"], fg=TH["dim"]).pack(side=tk.LEFT)

        right = tk.Frame(bar, bg=TH["panel"])
        right.pack(side=tk.RIGHT, padx=14, fill=tk.Y)

        tk.Label(right, text="TELEMETRY", font=("Segoe UI", 7, "bold"),
                 bg=TH["panel"], fg=TH["dim"]).grid(row=0, column=0, columnspan=5,
                                                    sticky="w", pady=(6, 0))
        self._tel_rate_var = tk.StringVar(value="1 Hz")
        col = 0
        for label in TELEMETRY_RATES:
            rb = tk.Radiobutton(right, text=label, variable=self._tel_rate_var,
                                value=label, bg=TH["panel"], fg=TH["text"],
                                selectcolor=TH["panel2"],
                                activebackground=TH["panel"],
                                font=FONT_LABEL)
            rb.grid(row=1, column=col, padx=3)
            col += 1

        self._tel_en_var = tk.BooleanVar(value=False)
        tk.Checkbutton(right, text="Enable", variable=self._tel_en_var,
                       command=self._on_tel_toggle,
                       bg=TH["panel"], fg=TH["accent"],
                       selectcolor=TH["panel2"],
                       activebackground=TH["panel"],
                       font=FONT_BOLD).grid(row=1, column=col, padx=(8, 0))

        tk.Frame(bar, bg=TH["border"], width=1).pack(side=tk.RIGHT, fill=tk.Y, pady=8, padx=10)

        conn = tk.Frame(bar, bg=TH["panel"])
        conn.pack(side=tk.RIGHT, fill=tk.Y)

        tk.Label(conn, text="PORT", font=("Segoe UI", 7, "bold"),
                 bg=TH["panel"], fg=TH["dim"]).grid(row=0, column=0, sticky="w", pady=(6, 0))
        tk.Label(conn, text="BAUD", font=("Segoe UI", 7, "bold"),
                 bg=TH["panel"], fg=TH["dim"]).grid(row=0, column=1, sticky="w", pady=(6, 0), padx=(6, 0))

        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(conn, textvariable=self._port_var, width=9,
                                       state="readonly", font=FONT_MONO)
        self._port_cb.grid(row=1, column=0, pady=(0, 6))

        self._baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Combobox(conn, textvariable=self._baud_var,
                     values=[str(b) for b in BAUD_RATES],
                     width=8, state="readonly",
                     font=FONT_MONO).grid(row=1, column=1, padx=(6, 4), pady=(0, 6))

        tk.Button(conn, text="⟳", command=self._refresh_ports,
                  bg=TH["panel"], fg=TH["accent"],
                  font=("Segoe UI", 13), bd=0, cursor="hand2",
                  activebackground=TH["panel"],
                  activeforeground=TH["accent2"]).grid(row=1, column=2, pady=(0, 6))

        self._stat_cv = tk.Canvas(conn, width=12, height=12,
                                   bg=TH["panel"], highlightthickness=0)
        self._stat_cv.grid(row=1, column=3, padx=(6, 4), pady=(0, 6))
        self._stat_dot = self._stat_cv.create_oval(1, 1, 11, 11,
                                                    fill=TH["led_off"], outline="")

        self._conn_btn = tk.Button(conn, text="Connect",
                                    command=self._toggle_conn,
                                    bg=TH["accent2"], fg="white",
                                    font=FONT_BOLD, bd=0, padx=14,
                                    cursor="hand2",
                                    activeforeground="white",
                                    activebackground=TH["accent2"])
        self._conn_btn.grid(row=1, column=4, padx=(4, 0), pady=(0, 6))

        self._refresh_ports()

    # ── Body ───────────────────────────────────────────────
    def _build_body(self):
        body = tk.Frame(self, bg=TH["bg"])
        body.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        left_container = tk.Frame(body, bg=TH["bg"])
        left_container.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 6))

        # Column 1: Simple Commands and Toggles
        col1 = tk.Frame(left_container, bg=TH["panel"], width=225)
        col1.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 6))
        col1.pack_propagate(False)
        self._build_cmd_panel_col1(col1)

        # Column 2: Axis Moves and Instruments
        col2 = tk.Frame(left_container, bg=TH["panel"], width=225)
        col2.pack(side=tk.LEFT, fill=tk.Y)
        col2.pack_propagate(False)
        self._build_cmd_panel_col2(col2)

        ctr = tk.Frame(body, bg=TH["bg"])
        ctr.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self._build_center(ctr)

        right = tk.Frame(body, bg=TH["panel"], width=170)
        right.pack(side=tk.LEFT, fill=tk.Y, padx=(6, 0))
        right.pack_propagate(False)
        self._build_flag_panel(right)

    # ── Command Panels ─────────────────────────────────────
    def _build_cmd_panel_col1(self, parent):
        self._section_header(parent, "QUICK COMMANDS")

        for label, cmd, bg in SIMPLE_COMMANDS:
            tk.Button(parent, text=label,
                      command=lambda c=cmd: self._send(c),
                      bg=bg, fg="#0a0a14",
                      font=FONT_BOLD, bd=0, pady=6, 
                      cursor="hand2",
                      activeforeground="#0a0a14",
                      activebackground=bg).pack(fill=tk.X, padx=10, pady=2)

        if TOGGLE_COMMANDS:
            self._divider(parent)
            self._section_header(parent, "TOGGLES")

            for label, cmd_on, cmd_off in TOGGLE_COMMANDS:
                self._toggle_states[label] = False

                btn = tk.Button(parent, text=f"{label} : OFF",
                                bg=TH["danger"], fg="#0a0a14",
                                font=FONT_BOLD, bd=0, pady=6, 
                                cursor="hand2",
                                activeforeground="#0a0a14",
                                activebackground=TH["danger"])

                def make_toggle(l=label, c_on=cmd_on, c_off=cmd_off, b=btn):
                    def _toggle():
                        self._toggle_states[l] = not self._toggle_states[l]
                        is_on = self._toggle_states[l]

                        if is_on:
                            self._send(c_on)
                            b.configure(text=f"{l} : ON", bg=TH["success"], 
                                        activebackground=TH["success"])
                        else:
                            self._send(c_off)
                            b.configure(text=f"{l} : OFF", bg=TH["danger"], 
                                        activebackground=TH["danger"])
                    return _toggle

                btn.configure(command=make_toggle())
                btn.pack(fill=tk.X, padx=10, pady=2)

    def _build_cmd_panel_col2(self, parent):
        self._section_header(parent, "AXIS MOVE")

        for axis_label, template in MOVE_COMMANDS:
            self._axis_block(parent, axis_label, template)

        self._divider(parent)
        self._section_header(parent, "ASTRO")
        self._goto_block(parent)

        self._divider(parent)
        self._section_header(parent, "RPi CAMERA CONTROL")
        self._capture_block(parent)

        self._divider(parent)
        self._section_header(parent, "CAMERA CONTROL")
        self._camera_block(parent)

        self._divider(parent)
        self._section_header(parent, "INSTRUMENTS")
        self._iset_block(parent)

    def _axis_block(self, parent, label, template):
        outer = tk.Frame(parent, bg=TH["panel2"], padx=0, pady=0)
        outer.pack(fill=tk.X, padx=10, pady=2)

        head = tk.Frame(outer, bg=TH["border"])
        head.pack(fill=tk.X)
        tk.Label(head, text=f"  {label}", font=FONT_BOLD,
                 bg=TH["border"], fg=TH["accent"]).pack(side=tk.LEFT, pady=2)

        row = tk.Frame(outer, bg=TH["panel2"])
        row.pack(fill=tk.X, padx=6, pady=2)

        tk.Label(row, text="Angle", font=("Segoe UI", 7),
                 bg=TH["panel2"], fg=TH["dim"]).grid(row=0, column=0, sticky="w")
        tk.Label(row, text="Speed", font=("Segoe UI", 7),
                 bg=TH["panel2"], fg=TH["dim"]).grid(row=0, column=1, sticky="w", padx=(6, 0))

        angle_v = tk.StringVar(value="0.0")
        speed_v = tk.StringVar(value="10")

        angle_e = tk.Entry(row, textvariable=angle_v, width=6,
                           bg=TH["bg"], fg=TH["text"],
                           insertbackground=TH["text"],
                           font=FONT_MONO, bd=0, relief=tk.FLAT)
        angle_e.grid(row=1, column=0)

        speed_e = tk.Entry(row, textvariable=speed_v, width=5,
                           bg=TH["bg"], fg=TH["text"],
                           insertbackground=TH["text"],
                           font=FONT_MONO, bd=0, relief=tk.FLAT)
        speed_e.grid(row=1, column=1, padx=(6, 6))

        go_btn = tk.Button(row, text="GO ▶",
                           command=lambda t=template, av=angle_v, sv=speed_v:
                               self._send_move(t, av, sv),
                           bg=TH["accent"], fg="#0a0a14",
                           font=("Segoe UI", 8, "bold"), bd=0, padx=6, pady=2,
                           cursor="hand2",
                           activeforeground="#0a0a14",
                           activebackground=TH["accent"])
        go_btn.grid(row=1, column=2)

    def _goto_block(self, parent):
        outer = tk.Frame(parent, bg=TH["panel2"], padx=0, pady=0)
        outer.pack(fill=tk.X, padx=10, pady=2)

        head = tk.Frame(outer, bg=TH["border"])
        head.pack(fill=tk.X)
        tk.Label(head, text="  GOTO", font=FONT_BOLD,
                 bg=TH["border"], fg=TH["accent"]).pack(side=tk.LEFT, pady=2)

        row = tk.Frame(outer, bg=TH["panel2"])
        row.pack(fill=tk.X, padx=6, pady=4)

        tk.Label(row, text="RA (hh:mm:ss)", font=("Segoe UI", 7),
                 bg=TH["panel2"], fg=TH["dim"]).grid(row=0, column=0, sticky="w", columnspan=2)

        ra_v = tk.StringVar(value="00:00:00")
        ra_e = tk.Entry(row, textvariable=ra_v, width=12,
                         bg=TH["bg"], fg=TH["text"],
                         insertbackground=TH["text"],
                         font=FONT_MONO, bd=0, relief=tk.FLAT)
        ra_e.grid(row=1, column=0, pady=(0, 4), sticky="w")

        tk.Label(row, text="DEC (dd:mm:ss)", font=("Segoe UI", 7),
                 bg=TH["panel2"], fg=TH["dim"]).grid(row=2, column=0, sticky="w", columnspan=2)

        dec_v = tk.StringVar(value="+00:00:00")
        dec_e = tk.Entry(row, textvariable=dec_v, width=12,
                          bg=TH["bg"], fg=TH["text"],
                          insertbackground=TH["text"],
                          font=FONT_MONO, bd=0, relief=tk.FLAT)
        dec_e.grid(row=3, column=0, sticky="w")

        go_btn = tk.Button(row, text="GOTO ▶",
                           command=lambda: self._send(f'$GOTO:{ra_v.get()}:{dec_v.get()}'),
                           bg=TH["accent"], fg="#0a0a14",
                           font=("Segoe UI", 8, "bold"), bd=0, padx=6, pady=2,
                           cursor="hand2",
                           activeforeground="#0a0a14",
                           activebackground=TH["accent"])
        go_btn.grid(row=3, column=1, padx=(10, 0))

    def _capture_block(self, parent):
        outer = tk.Frame(parent, bg=TH["panel2"], padx=0, pady=0)
        outer.pack(fill=tk.X, padx=10, pady=2)

        head = tk.Frame(outer, bg=TH["border"])
        head.pack(fill=tk.X)
        tk.Label(head, text="  CAPTURE IMAGE", font=FONT_BOLD,
                 bg=TH["border"], fg=TH["accent"]).pack(side=tk.LEFT, pady=2)

        row = tk.Frame(outer, bg=TH["panel2"])
        row.pack(fill=tk.X, padx=6, pady=2)

        tk.Label(row, text="Exp.", font=("Segoe UI", 7),
                 bg=TH["panel2"], fg=TH["dim"]).grid(row=0, column=0, sticky="w")
        tk.Label(row, text="Name", font=("Segoe UI", 7),
                 bg=TH["panel2"], fg=TH["dim"]).grid(row=0, column=1, sticky="w", padx=(6, 0))

        exp_v = tk.StringVar(value="10")
        name_v = tk.StringVar(value="stars.jpg")

        exp_e = tk.Entry(row, textvariable=exp_v, width=4,
                         bg=TH["bg"], fg=TH["text"],
                         insertbackground=TH["text"],
                         font=FONT_MONO, bd=0, relief=tk.FLAT)
        exp_e.grid(row=1, column=0)

        name_e = tk.Entry(row, textvariable=name_v, width=10,
                          bg=TH["bg"], fg=TH["text"],
                          insertbackground=TH["text"],
                          font=FONT_MONO, bd=0, relief=tk.FLAT)
        name_e.grid(row=1, column=1, padx=(6, 6))

        go_btn = tk.Button(row, text="SEND",
                           command=lambda: self._send(f'$RPI:CAPTURE:{exp_v.get()}:{name_v.get()}'),
                           bg=TH["accent"], fg="#0a0a14",
                           font=("Segoe UI", 8, "bold"), bd=0, padx=6, pady=2,
                           cursor="hand2",
                           activeforeground="#0a0a14",
                           activebackground=TH["accent"])
        go_btn.grid(row=1, column=2)
        
    def _camera_block(self, parent):
        outer = tk.Frame(parent, bg=TH["panel2"], padx=0, pady=0)
        outer.pack(fill=tk.X, padx=10, pady=2)

        head = tk.Frame(outer, bg=TH["border"])
        head.pack(fill=tk.X)
        tk.Label(head, text="  CAMERA IMAGE", font=FONT_BOLD,
                 bg=TH["border"], fg=TH["accent"]).pack(side=tk.LEFT, pady=2)

        row = tk.Frame(outer, bg=TH["panel2"])
        row.pack(fill=tk.X, padx=6, pady=2)

        tk.Label(row, text="Exp.", font=("Segoe UI", 7),
                 bg=TH["panel2"], fg=TH["dim"]).grid(row=0, column=0, sticky="w")
        tk.Label(row, text="Images", font=("Segoe UI", 7),
                 bg=TH["panel2"], fg=TH["dim"]).grid(row=0, column=1, sticky="w", padx=(6, 0))

        exp_v = tk.StringVar(value="5")
        num_v = tk.StringVar(value="1")

        exp_e = tk.Entry(row, textvariable=exp_v, width=6,
                         bg=TH["bg"], fg=TH["text"],
                         insertbackground=TH["text"],
                         font=FONT_MONO, bd=0, relief=tk.FLAT)
        exp_e.grid(row=1, column=0)

        num_e = tk.Entry(row, textvariable=num_v, width=5,
                          bg=TH["bg"], fg=TH["text"],
                          insertbackground=TH["text"],
                          font=FONT_MONO, bd=0, relief=tk.FLAT)
        num_e.grid(row=1, column=1, padx=(6, 6))

        go_btn = tk.Button(row, text="SEND",
                           command=lambda: self._send(f'$IMAGE:{exp_v.get()}:{num_v.get()}'),
                           bg=TH["accent"], fg="#0a0a14",
                           font=("Segoe UI", 8, "bold"), bd=0, padx=6, pady=2,
                           cursor="hand2",
                           activeforeground="#0a0a14",
                           activebackground=TH["accent"])
        go_btn.grid(row=1, column=2)

    def _iset_block(self, parent):
        outer = tk.Frame(parent, bg=TH["panel2"], padx=0, pady=0)
        outer.pack(fill=tk.X, padx=10, pady=2)

        head = tk.Frame(outer, bg=TH["border"])
        head.pack(fill=tk.X)
        tk.Label(head, text="  ISET (25-100%)", font=FONT_BOLD,
                 bg=TH["border"], fg=TH["accent"]).pack(side=tk.LEFT, pady=2)

        row = tk.Frame(outer, bg=TH["panel2"])
        row.pack(fill=tk.X, padx=6, pady=6)

        val_v = tk.StringVar(value="50")

        val_e = tk.Entry(row, textvariable=val_v, width=14,
                         bg=TH["bg"], fg=TH["text"],
                         insertbackground=TH["text"],
                         font=FONT_MONO, bd=0, relief=tk.FLAT)
        val_e.grid(row=0, column=0, padx=(0, 6))

        go_btn = tk.Button(row, text="SEND",
                           command=lambda: self._send(f'$ISET:{val_v.get()}'),
                           bg=TH["accent"], fg="#0a0a14",
                           font=("Segoe UI", 8, "bold"), bd=0, padx=14, pady=2,
                           cursor="hand2",
                           activeforeground="#0a0a14",
                           activebackground=TH["accent"])
        go_btn.grid(row=0, column=1)

    # ── Center: value cards + plots + console ─────────────
    def _build_center(self, parent):
        cards = tk.Frame(parent, bg=TH["bg"])
        cards.pack(fill=tk.X, pady=(0, 5))

        self._val_labels = {}
        # Render the text cards based on DISPLAY_ORDER
        for i, f in enumerate(DISPLAY_ORDER):
            r = i // 4
            c = i % 4
            card = tk.Frame(cards, bg=TH["panel"], padx=8, pady=5)
            card.grid(row=r, column=c, padx=3, pady=3, sticky="ew")
            cards.columnconfigure(c, weight=1)
            tk.Label(card, text=f, font=("Segoe UI", 7, "bold"),
                     bg=TH["panel"], fg=TH["dim"]).pack(anchor="w")
            lbl = tk.Label(card, textvariable=self._tel_var[f],
                           font=FONT_VALUE, bg=TH["panel"], fg=TH["accent"])
            lbl.pack(anchor="w")
            self._val_labels[f] = lbl

        plot_frame = tk.Frame(parent, bg=TH["bg"])
        plot_frame.pack(fill=tk.BOTH, expand=True)

        # Build plots based on PLOT_ORDER only
        n = len(PLOT_ORDER)
        ncols = min(3, n)
        nrows = (n + ncols - 1) // ncols

        self._fig = Figure(figsize=(9, 3.2 * nrows / 2), dpi=90,
                           facecolor=TH["plot_bg"])
        self._fig.subplots_adjust(left=0.07, right=0.97,
                                  top=0.90, bottom=0.14,
                                  hspace=0.55, wspace=0.35)
        self._axes = {}
        self._lines = {}
        for idx, f in enumerate(PLOT_ORDER):
            ax = self._fig.add_subplot(nrows, ncols, idx + 1)
            ax.set_facecolor(TH["plot_bg"])
            ax.set_title(f, color=TH["text"], fontsize=8, pad=3,
                         fontfamily="Segoe UI")
            ax.tick_params(colors=TH["dim"], labelsize=6.5)
            ax.set_xlabel("t (s)", color=TH["dim"], fontsize=6)
            for sp in ax.spines.values():
                sp.set_color(TH["border"])
            line, = ax.plot([], [], color=TH["accent"], linewidth=1.3)
            self._axes[f]  = ax
            self._lines[f] = line

        self._canvas = FigureCanvasTkAgg(self._fig, master=plot_frame)
        self._canvas.get_tk_widget().configure(bg=TH["plot_bg"],
                                               highlightthickness=0)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._build_console(parent)

    def _build_console(self, parent):
        frame = tk.Frame(parent, bg=TH["panel"])
        frame.pack(fill=tk.BOTH, expand=False, pady=(5, 0))

        hdr = tk.Frame(frame, bg=TH["panel"])
        hdr.pack(fill=tk.X, padx=8, pady=(6, 2))
        tk.Label(hdr, text="UART CONSOLE", font=("Segoe UI", 8, "bold"),
                 bg=TH["panel"], fg=TH["dim"]).pack(side=tk.LEFT)
        tk.Button(hdr, text="Clear", command=self._clear_console,
                  bg=TH["panel"], fg=TH["dim"],
                  font=("Segoe UI", 8), bd=0, cursor="hand2",
                  activebackground=TH["panel"]).pack(side=tk.RIGHT)

        self._console = scrolledtext.ScrolledText(
            frame, height=8, bg="#08080f", fg=TH["text"],
            font=FONT_MONO, insertbackground=TH["text"],
            state=tk.DISABLED, bd=0, wrap=tk.WORD,
            selectbackground=TH["panel2"]
        )
        self._console.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 4))
        self._console.tag_configure("TX",     foreground=TH["tx_color"])
        self._console.tag_configure("RX",     foreground=TH["rx_color"])
        self._console.tag_configure("STATUS", foreground=TH["stat_color"])
        self._console.tag_configure("ERROR",  foreground=TH["err_color"])
        self._console.tag_configure("TEL",    foreground=TH["tel_color"])

        inp = tk.Frame(frame, bg=TH["panel"])
        inp.pack(fill=tk.X, padx=8, pady=(0, 8))

        tk.Label(inp, text="›", font=("Consolas", 14, "bold"),
                 bg=TH["panel"], fg=TH["accent"]).pack(side=tk.LEFT, padx=(0, 4))

        self._manual_var = tk.StringVar()
        entry = tk.Entry(inp, textvariable=self._manual_var,
                         bg="#08080f", fg=TH["text"],
                         insertbackground=TH["text"],
                         font=("Consolas", 10), bd=0, relief=tk.FLAT)
        entry.pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=5)
        entry.bind("<Return>", self._on_manual_send)

        tk.Button(inp, text="Send",
                  command=self._on_manual_send,
                  bg=TH["accent2"], fg="white",
                  font=FONT_BOLD, bd=0, padx=12, pady=5,
                  cursor="hand2",
                  activeforeground="white",
                  activebackground=TH["accent2"]).pack(side=tk.LEFT, padx=(6, 0))

    # ── Flag panel ─────────────────────────────────────────
    def _build_flag_panel(self, parent):
        self._section_header(parent, "FLAGS")

        self._flag_leds  = {}
        self._flag_lbls  = {}

        for flag in FLAG_FIELDS:
            row = tk.Frame(parent, bg=TH["panel"])
            row.pack(fill=tk.X, padx=10, pady=5)

            cv = tk.Canvas(row, width=16, height=16,
                           bg=TH["panel"], highlightthickness=0)
            cv.pack(side=tk.LEFT, padx=(0, 8))
            dot = cv.create_oval(1, 1, 15, 15, fill=TH["led_off"], outline="")
            self._flag_leds[flag] = (cv, dot)

            lbl = tk.Label(row, text=flag, font=FONT_LABEL,
                           bg=TH["panel"], fg=TH["dim"])
            lbl.pack(side=tk.LEFT)
            self._flag_lbls[flag] = lbl

    # ── Helpers ────────────────────────────────────────────
    def _section_header(self, parent, text):
        tk.Label(parent, text=text, font=("Segoe UI", 7, "bold"),
                 bg=TH["panel"], fg=TH["dim"]).pack(pady=(10, 4), padx=10, anchor="w")
        self._divider(parent)

    def _divider(self, parent):
        tk.Frame(parent, bg=TH["border"], height=1).pack(fill=tk.X, padx=10,
                                                          pady=(0, 6))

    # ── Coordinate Formatting ──────────────────────────────
    def _format_ra(self, val_str):
            # If it's already formatted with colons from the device
            if ":" in val_str:
                p = val_str.split(":")
                if len(p) >= 3:
                    try:
                        return f"{int(p[0]):02d}h {int(p[1]):02d}m {int(float(p[2])):02d}s"
                    except ValueError:
                        pass
            # If it's a raw decimal number (assumed degrees)
            try:
                val = float(val_str)
                
                # Convert degrees to hours
                val_hours = val / 15.0 
                
                h = int(val_hours)
                m_dec = abs(val_hours - h) * 60.0
                m = int(m_dec)
                s = (m_dec - m) * 60.0
                
                return f"{h:02d}h {m:02d}m {int(s):02d}s"
            except ValueError:
                return val_str

    def _format_dec(self, val_str):
        if ":" in val_str:
            p = val_str.split(":")
            if len(p) >= 3:
                try:
                    return f"{int(p[0]):02d}° {abs(int(p[1])):02d}'{abs(int(float(p[2]))):02d}\""
                except ValueError:
                    pass
        try:
            val = float(val_str)
            sign = "-" if val < 0 else "+"
            val = abs(val)
            d = int(val)
            m_dec = (val - d) * 60.0
            m = int(m_dec)
            s = (m_dec - m) * 60.0
            return f"{sign}{d:02d}° {m:02d}'{int(s):02d}\""
        except ValueError:
            return val_str

    # ── Connection ─────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_cb["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _toggle_conn(self):
        if self._connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self._port_var.get()
        if not port:
            messagebox.showerror("No port", "Please select a COM port.")
            return
        baud = int(self._baud_var.get())

        self._worker = SerialWorker(port, baud, self._rx_q, self._tx_q)
        self._worker.start()

        self._poller = TelPoller(self._tx_q, self._tel_rate_var)
        self._poller.start()

        self._tel_en_var.set(True)
        self._poller.enabled = True

        self._connected = True
        self._conn_btn.configure(text="Disconnect", bg=TH["danger"],
                                  activebackground=TH["danger"])
        self._stat_cv.itemconfig(self._stat_dot, fill=TH["led_on"])

    def _disconnect(self):
        if self._worker:  self._worker.stop()
        if self._poller:  self._poller.stop()
        self._connected = False
        self._tel_en_var.set(False)
        if self._poller:  self._poller.enabled = False
        self._conn_btn.configure(text="Connect", bg=TH["accent2"],
                                  activebackground=TH["accent2"])
        self._stat_cv.itemconfig(self._stat_dot, fill=TH["led_off"])

    def _on_tel_toggle(self):
        if self._poller:
            self._poller.enabled = self._tel_en_var.get()

    # ── Sending ────────────────────────────────────────────
    def _send(self, cmd):
        if not self._connected:
            self._log("Not connected", "ERROR")
            return
        self._tx_q.put(cmd)

    def _send_move(self, template, angle_v, speed_v):
        try:
            angle = float(angle_v.get())
            speed = int(float(speed_v.get()))
        except ValueError:
            self._log("Invalid angle / speed value", "ERROR")
            return
        self._send(template.format(angle=angle, speed=speed))

    def _on_manual_send(self, _event=None):
        cmd = self._manual_var.get().strip()
        if cmd:
            self._send(cmd)
            self._manual_var.set("")

    # ── RX queue polling (runs in main thread) ─────────────
    def _poll_rx(self):
        try:
            while True:
                msg_type, msg = self._rx_q.get_nowait()
                self._dispatch(msg_type, msg)
        except queue.Empty:
            pass
        self.after(40, self._poll_rx)

    def _dispatch(self, msg_type, msg):
        if msg_type == "RX":
            stripped = msg.strip()
            if stripped.startswith("#TEL:") or stripped.startswith("$TEL:"):
                self._parse_tel(stripped)
                self._log(stripped, "TEL")
            else:
                self._log(stripped, "RX")
        elif msg_type == "TX":
            if msg == TELEMETRY_COMMAND and self._tel_en_var.get():
                return
            self._log(msg, "TX")
        else:
            self._log(msg, msg_type)

    # ── Telemetry parsing ──────────────────────────────────
    def _parse_tel(self, msg):
        try:
            data  = msg.split(":", 1)[1] if ":" in msg else msg
            parts = [p.strip() for p in data.split(":")]
            t_rel = time.time() - self._t0
            self._t_buf.append(t_rel)

            for i, f in enumerate(TELEMETRY_FIELDS):
                if i < len(parts):
                    raw_str = parts[i].strip()
                    
                    if f == "RA COORD":
                        self._tel_var[f].set(self._format_ra(raw_str))
                    elif f == "DEC COORD":
                        self._tel_var[f].set(self._format_dec(raw_str))
                    else:
                        try:
                            val = float(raw_str)
                            self._tel_buf[f].append(val)
                            self._tel_var[f].set(f"{val:.3f}")
                        except ValueError:
                            # Safely handle pure string data
                            self._tel_var[f].set(raw_str)

            off = len(TELEMETRY_FIELDS)
            for j, flag in enumerate(FLAG_FIELDS):
                idx = off + j
                if idx < len(parts):
                    try:
                        state = bool(int(parts[idx]))
                        self._set_led(flag, state)
                    except ValueError:
                        pass

            self._update_plots()

        except Exception as exc:
            self._log(f"Parse error: {exc}", "ERROR")

    # ── Plot update ────────────────────────────────────────
    def _update_plots(self):
        t_list = list(self._t_buf)
        # Update plots based on PLOT_ORDER
        for f in PLOT_ORDER:
            vals = list(self._tel_buf[f])
            if len(vals) < 2:
                continue
            ts = t_list[-len(vals):]
            self._lines[f].set_data(ts, vals)
            ax = self._axes[f]
            ax.relim()
            ax.autoscale_view()
        self._canvas.draw_idle()

    # ── LED update ─────────────────────────────────────────
    def _set_led(self, flag, state):
        cv, dot = self._flag_leds[flag]
        if state:
            color = TH["led_err"] if flag in ERROR_FLAGS else TH["led_on"]
        else:
            color = TH["led_off"]
        cv.itemconfig(dot, fill=color)
        self._flag_lbls[flag].configure(fg=TH["text"] if state else TH["dim"])

    # ── Console ────────────────────────────────────────────
    def _log(self, msg, tag="RX"):
        ts  = time.strftime("%H:%M:%S")
        pfx = {"TX": ">> ", "RX": "<< ",
               "STATUS": "-- ", "ERROR": "!! ", "TEL": "~~ "}.get(tag, "   ")
        self._console.configure(state=tk.NORMAL)
        self._console.insert(tk.END, f"[{ts}] {pfx}{msg}\n", tag)
        self._console.see(tk.END)
        self._console.configure(state=tk.DISABLED)

    def _clear_console(self):
        self._console.configure(state=tk.NORMAL)
        self._console.delete("1.0", tk.END)
        self._console.configure(state=tk.DISABLED)

# ════════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = App()
    app.mainloop()