#!/usr/bin/env python3
"""Graphical User Interface (GUI) for testing Kenny's wheel odometers & 1-revolution spin accuracy.

Automatically detects which /dev/tty* port is the ESP32 (verifying the KENNY_SIDE_V3 / ODOM
handshake) and connects immediately on startup.

Launch directly:
  python3 scripts/wheel_odometer_gui.py
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Deque, Dict, List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.test_wheel_odometer import (
    DEFAULT_BAUD,
    DEFAULT_COUNTS_PER_REV,
    DEFAULT_TRACK_WIDTH_M,
    DEFAULT_WHEEL_RADIUS_M,
    ENCODER_PIN_MAP,
    DiagnosticFinding,
    DifferentialDriveOdometry,
    RevSpinResult,
    SerialESP32Board,
    SimulatedESP32Board,
    WheelMetrics,
    WheelTelemetry,
    auto_detect_esp32_port,
    compute_wheel_metrics,
    diagnose_wheel_health,
    discover_serial_ports,
    estimate_cpr_from_manual_turns,
    evaluate_one_rev_accuracy,
    flash_esp32_without_arduino_ide,
    parse_odom_line,
    parse_rev_result_line,
)


class WheelOdometerGUI:
    """Tkinter desktop dashboard for wheel odometer diagnostics and 1-revolution accuracy testing."""

    BG_MAIN = "#111827"
    BG_CARD = "#1f2937"
    BG_PANEL = "#0f172a"
    FG_TEXT = "#f9fafb"
    FG_MUTED = "#9ca3af"
    ACCENT_M1 = "#38bdf8"
    ACCENT_M2 = "#a78bfa"
    COLOR_OK = "#10b981"
    COLOR_WARN = "#f59e0b"
    COLOR_ERR = "#ef4444"

    def __init__(
        self,
        root: tk.Tk,
        port: Optional[str] = None,
        side: str = "left",
        cpr: float = 660.0,
        wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_M,
        track_width_m: float = DEFAULT_TRACK_WIDTH_M,
        simulate: bool = False,
        allow_lidar_port: bool = False,
    ) -> None:
        self.root = root
        self.root.title("KENNY — Wheel Odometer, Auto-Detect & 1-Revolution Accuracy Studio")
        self.root.geometry("1360x880")
        self.root.minsize(1120, 760)
        self.root.configure(bg=self.BG_MAIN)

        self.allow_lidar_port = allow_lidar_port
        self.odom = DifferentialDriveOdometry(
            wheel_radius_m=wheel_radius_m,
            track_width_m=track_width_m,
            counts_per_rev=cpr,
        )

        # Connection & Board State
        self.board: Optional[object] = None
        self.stop_reader = threading.Event()
        self.rx_queue: "queue.Queue[str]" = queue.Queue()
        self.latest_telemetry = WheelTelemetry(side=side.upper())
        self.m1_metrics = WheelMetrics()
        self.m2_metrics = WheelMetrics()
        self.m1_sign = 1
        self.m2_sign = 1
        self.target_revs_marker = 1.0
        self.manual_cal_Active = False
        self.manual_cal_start_ticks = (0, 0)
        self.rev_history: List[RevSpinResult] = []
        self.measured_cpr_pair: Optional[Tuple[float, float]] = None
        self.auto_cal_active = False
        self.auto_cal_baseline: Dict[str, Tuple[int, int, int, int]] = {}
        self.auto_cal_last_ticks: Dict[str, int] = {}
        self.auto_cal_last_motion: Dict[str, Optional[float]] = {}
        self.auto_cal_results: Dict[str, float] = {}
        self.auto_cal_started_at = 0.0
        self.pwm_tune_motor: Optional[str] = None
        self.pwm_tune_candidates: List[int] = []
        self.pwm_tune_results: List[Tuple[int, RevSpinResult]] = []
        self.pwm_tune_current_pwm: Optional[int] = None
        self.pwm_tune_generation = 0
        self.pwm_tune_start_edges = (0, 0)

        # Time-series history for scope (timestamp_s, m1_revs, m2_revs, target_revs)
        self.trace_history: Deque[Tuple[float, float, float, float]] = deque(maxlen=200)
        self._start_mono = time.monotonic()
        self._prev_telem_mono: Optional[float] = None

        # Tkinter Variables
        self.port_var = tk.StringVar(value=port or "")
        self.sim_var = tk.BooleanVar(value=simulate)
        self.swap_enc_var = tk.BooleanVar(value=True)
        self.side_var = tk.StringVar(value=side.upper())
        self.cpr_var = tk.StringVar(value=f"{cpr:.1f}")
        self.radius_var = tk.StringVar(value=f"{wheel_radius_m:.4f}")
        self.deadband_var = tk.StringVar(value="75")
        self.target_rev_var = tk.StringVar(value="1.000")
        self.max_pwm_var = tk.StringVar(value="135")
        self.jog_pwm_var = tk.StringVar(value="120")
        self.jog_ms_var = tk.StringVar(value="500")
        self.conn_status_var = tk.StringVar(value="Scanning for ESP32...")

        self._build_ui()

        # Auto-detect ESP32 on startup unless --simulate was explicitly requested
        if simulate:
            self.refresh_ports(probe=False)
            self.sim_var.set(True)
            self.connect_board()
        elif port:
            self.refresh_ports(probe=False)
            self.port_var.set(port)
            self.sim_var.set(False)
            self.connect_board()
        else:
            self.auto_detect_and_connect()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(40, self._periodic_ui_tick)

    # =========================================================================
    # UI Layout Construction
    # =========================================================================
    def _build_ui(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Treeview", background=self.BG_PANEL, fieldbackground=self.BG_PANEL,
                        foreground=self.FG_TEXT, rowheight=24, font=("DejaVu Sans Mono", 9))
        style.configure("Treeview.Heading", background=self.BG_CARD, foreground=self.FG_TEXT,
                        font=("DejaVu Sans", 9, "bold"))

        # 1. Top Control & Connection Bar
        top_bar = tk.Frame(self.root, bg=self.BG_CARD, padx=12, pady=8, highlightthickness=1,
                           highlightbackground="#374151")
        top_bar.pack(fill=tk.X, padx=10, pady=(10, 6))

        tk.Label(top_bar, text="ESP32 Port:", bg=self.BG_CARD, fg=self.FG_TEXT,
                 font=("DejaVu Sans", 10, "bold")).pack(side=tk.LEFT, padx=(0, 4))
        self.port_combo = ttk.Combobox(top_bar, textvariable=self.port_var, width=18)
        self.port_combo.pack(side=tk.LEFT, padx=(0, 4))

        tk.Button(
            top_bar, text="Auto-Detect ESP32", command=self.auto_detect_and_connect,
            bg="#0284c7", fg="#ffffff", font=("DejaVu Sans", 9, "bold"), relief=tk.FLAT, padx=8
        ).pack(side=tk.LEFT, padx=3)

        tk.Checkbutton(
            top_bar, text="Simulation Mode", variable=self.sim_var,
            command=self.on_toggle_sim, bg=self.BG_CARD, fg="#38bdf8", selectcolor=self.BG_PANEL,
            activebackground=self.BG_CARD, activeforeground="#38bdf8", font=("DejaVu Sans", 9, "bold")
        ).pack(side=tk.LEFT, padx=6)

        self.connect_btn = tk.Button(
            top_bar, text="Connect", command=self.toggle_connection, bg="#2563eb", fg=self.FG_TEXT,
            font=("DejaVu Sans", 9, "bold"), relief=tk.FLAT, padx=10
        )
        self.connect_btn.pack(side=tk.LEFT, padx=4)

        tk.Label(top_bar, text="Side:", bg=self.BG_CARD, fg=self.FG_TEXT,
                 font=("DejaVu Sans", 9, "bold")).pack(side=tk.LEFT, padx=(8, 4))
        side_combo = ttk.Combobox(top_bar, textvariable=self.side_var, values=["LEFT", "RIGHT"],
                                  width=7, state="readonly")
        side_combo.pack(side=tk.LEFT, padx=2)
        side_combo.bind("<<ComboboxSelected>>", lambda _e: self.apply_parameters())

        tk.Label(top_bar, text="CPR:", bg=self.BG_CARD, fg=self.FG_TEXT,
                 font=("DejaVu Sans", 9, "bold")).pack(side=tk.LEFT, padx=(8, 2))
        tk.Entry(top_bar, textvariable=self.cpr_var, width=7, bg=self.BG_PANEL, fg=self.FG_TEXT,
                 insertbackground=self.FG_TEXT).pack(side=tk.LEFT, padx=2)

        tk.Label(top_bar, text="Wheel r (m):", bg=self.BG_CARD, fg=self.FG_TEXT,
                 font=("DejaVu Sans", 9, "bold")).pack(side=tk.LEFT, padx=(8, 2))
        tk.Entry(top_bar, textvariable=self.radius_var, width=6, bg=self.BG_PANEL, fg=self.FG_TEXT,
                 insertbackground=self.FG_TEXT).pack(side=tk.LEFT, padx=2)

        tk.Button(top_bar, text="Apply", command=self.apply_parameters, bg="#059669", fg=self.FG_TEXT,
                  font=("DejaVu Sans", 9, "bold"), relief=tk.FLAT, padx=8).pack(side=tk.LEFT, padx=4)

        self.swap_enc_checkbox = tk.Checkbutton(
            top_bar, text="Swap M1<->M2 Enc", variable=self.swap_enc_var,
            command=self.on_toggle_swap_encoders, bg=self.BG_CARD, fg="#fbbf24", selectcolor=self.BG_PANEL,
            activebackground=self.BG_CARD, activeforeground="#fbbf24", font=("DejaVu Sans", 9, "bold")
        )
        self.swap_enc_checkbox.pack(side=tk.LEFT, padx=6)

        tk.Button(
            top_bar, text="EMERGENCY STOP", command=self.emergency_stop,
            bg=self.COLOR_ERR, fg="#ffffff", font=("DejaVu Sans", 10, "bold"), relief=tk.FLAT, padx=12
        ).pack(side=tk.RIGHT, padx=4)

        self.conn_label = tk.Label(top_bar, textvariable=self.conn_status_var, bg=self.BG_CARD,
                                   fg=self.COLOR_OK, font=("DejaVu Sans Mono", 9, "bold"))
        self.conn_label.pack(side=tk.RIGHT, padx=8)

        # 2. Main Split Body (Left: Wheel Dials + Telemetry + Scope | Right: 1-Rev Accuracy + Troubleshooting)
        main_frame = tk.Frame(self.root, bg=self.BG_MAIN)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)
        main_frame.columnconfigure(0, weight=5)
        main_frame.columnconfigure(1, weight=5)
        main_frame.rowconfigure(0, weight=1)

        left_col = tk.Frame(main_frame, bg=self.BG_MAIN)
        left_col.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        right_col = tk.Frame(main_frame, bg=self.BG_MAIN)
        right_col.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        self._build_wheel_cards(left_col)
        self._build_scope_panel(left_col)
        self._build_one_rev_accuracy_panel(right_col)
        self._build_troubleshooting_panel(right_col)

    def _build_wheel_cards(self, parent: tk.Widget) -> None:
        wheels_frame = tk.Frame(parent, bg=self.BG_MAIN)
        wheels_frame.pack(fill=tk.X, pady=(0, 6))
        wheels_frame.columnconfigure(0, weight=1)
        wheels_frame.columnconfigure(1, weight=1)

        self.m1_widgets = self._create_single_wheel_card(
            wheels_frame, col=0, title="MOTOR 1 (OUT1 / OUT2)", accent=self.ACCENT_M1, motor_id="M1"
        )
        self.m2_widgets = self._create_single_wheel_card(
            wheels_frame, col=1, title="MOTOR 2 (OUT3 / OUT4)", accent=self.ACCENT_M2, motor_id="M2"
        )

        # 2D Robot Pose Bar + Zero Button
        pose_bar = tk.Frame(parent, bg=self.BG_CARD, padx=10, pady=6, highlightthickness=1,
                            highlightbackground="#374151")
        pose_bar.pack(fill=tk.X, pady=(0, 6))

        self.pose_label = tk.Label(
            pose_bar,
            text="2D Odometry:  X = +0.000 m   Y = +0.000 m   Yaw = +0.0°   Path = 0.000 m   v = +0.00 m/s",
            bg=self.BG_CARD, fg="#e5e7eb", font=("DejaVu Sans Mono", 10, "bold")
        )
        self.pose_label.pack(side=tk.LEFT)

        tk.Button(
            pose_bar, text="Zero Ticks & Pose (Reset)", command=self.zero_all,
            bg="#d97706", fg="#ffffff", font=("DejaVu Sans", 9, "bold"), relief=tk.FLAT, padx=10
        ).pack(side=tk.RIGHT, padx=4)

        tk.Button(
            pose_bar, text="Save Report JSON", command=self.save_report_json,
            bg="#374151", fg=self.FG_TEXT, font=("DejaVu Sans", 9), relief=tk.FLAT, padx=8
        ).pack(side=tk.RIGHT, padx=4)

    def _create_single_wheel_card(
        self, parent: tk.Widget, col: int, title: str, accent: str, motor_id: str
    ) -> Dict[str, tk.Widget]:
        card = tk.Frame(parent, bg=self.BG_CARD, padx=10, pady=8, highlightthickness=1,
                        highlightbackground="#374151")
        card.grid(row=0, column=col, sticky="nsew", padx=(0 if col == 0 else 4, 4 if col == 0 else 0))

        header = tk.Frame(card, bg=self.BG_CARD)
        header.pack(fill=tk.X)
        tk.Label(header, text=title, bg=self.BG_CARD, fg=accent,
                 font=("DejaVu Sans", 11, "bold")).pack(side=tk.LEFT)
        pin_lbl = tk.Label(header, text="Pins: GPIO18/21", bg=self.BG_CARD, fg=self.FG_MUTED,
                           font=("DejaVu Sans Mono", 9))
        pin_lbl.pack(side=tk.RIGHT)

        body = tk.Frame(card, bg=self.BG_CARD)
        body.pack(fill=tk.X, pady=4)

        # 360-degree Wheel Dial Canvas
        dial = tk.Canvas(body, width=150, height=150, bg=self.BG_PANEL, highlightthickness=0)
        dial.pack(side=tk.LEFT, padx=(0, 10))

        # Digital Readouts
        readouts = tk.Frame(body, bg=self.BG_CARD)
        readouts.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        rev_lbl = tk.Label(readouts, text="+0.0000 rev", bg=self.BG_CARD, fg=self.FG_TEXT,
                           font=("DejaVu Sans Mono", 16, "bold"), anchor="w")
        rev_lbl.pack(fill=tk.X)

        deg_lbl = tk.Label(readouts, text="Angle: +0.0°  (Dial: 0.0°)", bg=self.BG_CARD, fg=accent,
                           font=("DejaVu Sans Mono", 10, "bold"), anchor="w")
        deg_lbl.pack(fill=tk.X, pady=(1, 3))

        ticks_lbl = tk.Label(readouts, text="Ticks: +0", bg=self.BG_CARD, fg=self.FG_TEXT,
                             font=("DejaVu Sans Mono", 10), anchor="w")
        ticks_lbl.pack(fill=tk.X)

        dist_lbl = tk.Label(readouts, text="Distance: +0.000 m (+0.0 cm)", bg=self.BG_CARD, fg=self.FG_TEXT,
                            font=("DejaVu Sans Mono", 10), anchor="w")
        dist_lbl.pack(fill=tk.X)

        speed_lbl = tk.Label(readouts, text="Speed: +0.0 RPM (+0.00 m/s)", bg=self.BG_CARD, fg=self.FG_TEXT,
                             font=("DejaVu Sans Mono", 10), anchor="w")
        speed_lbl.pack(fill=tk.X)

        ab_lbl = tk.Label(readouts, text="Edges A/B: 0 / 0 (Sym: 100%)", bg=self.BG_CARD, fg=self.FG_MUTED,
                          font=("DejaVu Sans Mono", 9), anchor="w")
        ab_lbl.pack(fill=tk.X, pady=(2, 0))

        err_lbl = tk.Label(readouts, text="Glitches: 0  |  PWM: 0  |  AB: 00", bg=self.BG_CARD, fg=self.FG_MUTED,
                           font=("DejaVu Sans Mono", 9), anchor="w")
        err_lbl.pack(fill=tk.X)

        # Quick Per-Wheel Controls (Invert Sign, Manual Jog, Sim Hand-Spin)
        btn_row = tk.Frame(card, bg=self.BG_CARD)
        btn_row.pack(fill=tk.X, pady=(6, 0))

        tk.Button(
            btn_row, text=f"Jog {motor_id} +PWM",
            command=lambda m=motor_id: self.jog_motor(m, +1),
            bg="#1d4ed8", fg=self.FG_TEXT, relief=tk.FLAT, font=("DejaVu Sans", 8, "bold"), padx=6
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            btn_row, text=f"Jog {motor_id} -PWM",
            command=lambda m=motor_id: self.jog_motor(m, -1),
            bg="#374151", fg=self.FG_TEXT, relief=tk.FLAT, font=("DejaVu Sans", 8), padx=6
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            btn_row, text=f"Invert {motor_id} Sign",
            command=lambda m=motor_id: self.invert_motor_sign(m),
            bg="#4b5563", fg=self.FG_TEXT, relief=tk.FLAT, font=("DejaVu Sans", 8), padx=6
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            btn_row, text="Sim +1 Turn",
            command=lambda m=motor_id: self.sim_hand_spin(m, 1.0),
            bg="#065f46", fg=self.FG_TEXT, relief=tk.FLAT, font=("DejaVu Sans", 8), padx=6
        ).pack(side=tk.RIGHT, padx=2)

        return {
            "dial": dial,
            "pin_lbl": pin_lbl,
            "rev_lbl": rev_lbl,
            "deg_lbl": deg_lbl,
            "ticks_lbl": ticks_lbl,
            "dist_lbl": dist_lbl,
            "speed_lbl": speed_lbl,
            "ab_lbl": ab_lbl,
            "err_lbl": err_lbl,
        }

    def _build_scope_panel(self, parent: tk.Widget) -> None:
        scope_card = tk.Frame(parent, bg=self.BG_CARD, padx=10, pady=8, highlightthickness=1,
                              highlightbackground="#374151")
        scope_card.pack(fill=tk.BOTH, expand=True)

        hdr = tk.Frame(scope_card, bg=self.BG_CARD)
        hdr.pack(fill=tk.X)
        tk.Label(
            hdr, text="LIVE REVOLUTION & TARGET TRACKING SCOPE (LAST 10s)",
            bg=self.BG_CARD, fg=self.FG_TEXT, font=("DejaVu Sans", 10, "bold")
        ).pack(side=tk.LEFT)
        tk.Label(
            hdr, text="Cyan = M1 Revs   Purple = M2 Revs   Dashed Green = Target Rev",
            bg=self.BG_CARD, fg=self.FG_MUTED, font=("DejaVu Sans", 9)
        ).pack(side=tk.RIGHT)

        self.scope_canvas = tk.Canvas(scope_card, bg=self.BG_PANEL, height=220, highlightthickness=0)
        self.scope_canvas.pack(fill=tk.BOTH, expand=True, pady=(6, 0))

    def _build_one_rev_accuracy_panel(self, parent: tk.Widget) -> None:
        acc_card = tk.Frame(parent, bg=self.BG_CARD, padx=12, pady=10, highlightthickness=1,
                            highlightbackground="#374151")
        acc_card.pack(fill=tk.BOTH, expand=True, pady=(0, 6))

        tk.Label(
            acc_card, text="1-REVOLUTION SPIN ACCURACY TEST & CALIBRATION",
            bg=self.BG_CARD, fg="#34d399", font=("DejaVu Sans", 11, "bold"), anchor="w"
        ).pack(fill=tk.X)

        # Controls Row 1: Closed-Loop Active Spin by 1.000 Revolution
        ctrl_row = tk.Frame(acc_card, bg=self.BG_CARD)
        ctrl_row.pack(fill=tk.X, pady=(6, 4))

        tk.Label(ctrl_row, text="Target Revs:", bg=self.BG_CARD, fg=self.FG_TEXT,
                 font=("DejaVu Sans", 9, "bold")).pack(side=tk.LEFT)
        tk.Entry(ctrl_row, textvariable=self.target_rev_var, width=6, bg=self.BG_PANEL,
                 fg=self.FG_TEXT, insertbackground=self.FG_TEXT).pack(side=tk.LEFT, padx=(4, 10))

        tk.Label(ctrl_row, text="Max PWM (<=191):", bg=self.BG_CARD, fg=self.FG_TEXT,
                 font=("DejaVu Sans", 9, "bold")).pack(side=tk.LEFT)
        tk.Entry(ctrl_row, textvariable=self.max_pwm_var, width=5, bg=self.BG_PANEL,
                 fg=self.FG_TEXT, insertbackground=self.FG_TEXT).pack(side=tk.LEFT, padx=(4, 10))

        tk.Label(ctrl_row, text="Deadband PWM:", bg=self.BG_CARD, fg=self.FG_TEXT,
                 font=("DejaVu Sans", 9, "bold")).pack(side=tk.LEFT)
        tk.Entry(ctrl_row, textvariable=self.deadband_var, width=4, bg=self.BG_PANEL,
                 fg=self.FG_TEXT, insertbackground=self.FG_TEXT).pack(side=tk.LEFT, padx=(4, 10))

        btn_spin_row = tk.Frame(acc_card, bg=self.BG_CARD)
        btn_spin_row.pack(fill=tk.X, pady=4)

        tk.Button(
            btn_spin_row, text="Spin M1 Exactly 1 Rev",
            command=lambda: self.start_closed_loop_rev_test("M1"),
            bg="#0284c7", fg="#ffffff", font=("DejaVu Sans", 9, "bold"), relief=tk.FLAT, padx=10, pady=4
        ).pack(side=tk.LEFT, padx=(0, 6))

        tk.Button(
            btn_spin_row, text="Spin M2 Exactly 1 Rev",
            command=lambda: self.start_closed_loop_rev_test("M2"),
            bg="#7c3aed", fg="#ffffff", font=("DejaVu Sans", 9, "bold"), relief=tk.FLAT, padx=10, pady=4
        ).pack(side=tk.LEFT, padx=6)

        tk.Button(
            btn_spin_row, text="Spin BOTH Exactly 1 Rev",
            command=lambda: self.start_closed_loop_rev_test("BOTH"),
            bg="#059669", fg="#ffffff", font=("DejaVu Sans", 9, "bold"), relief=tk.FLAT, padx=10, pady=4
        ).pack(side=tk.LEFT, padx=6)

        tune_row = tk.Frame(acc_card, bg=self.BG_PANEL, padx=8, pady=5)
        tune_row.pack(fill=tk.X, pady=4)
        for motor, color in (("M1", "#0369a1"), ("M2", "#6d28d9")):
            tk.Button(
                tune_row, text=f"Auto Tune {motor} PWM", command=lambda m=motor: self.start_pwm_tune(m),
                bg=color, fg="#ffffff", font=("DejaVu Sans", 9, "bold"), relief=tk.FLAT,
                padx=8, pady=3,
            ).pack(side=tk.LEFT, padx=(0, 6))
        self.pwm_tune_status = tk.Label(
            tune_row, text="Powered test: wheels must be lifted. Uses 660 CPR; tests several PWM limits.",
            bg=self.BG_PANEL, fg=self.FG_MUTED, font=("DejaVu Sans", 9),
            anchor="w", wraplength=420, justify=tk.LEFT,
        )
        self.pwm_tune_status.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Controls Row 2: Manual Hand-Spin 1-Turn Accuracy & CPR Calibration
        cal_row = tk.Frame(acc_card, bg=self.BG_PANEL, padx=8, pady=6)
        cal_row.pack(fill=tk.X, pady=4)

        tk.Label(cal_row, text="Manual Hand-Spin 1-Rev Check:", bg=self.BG_PANEL, fg="#fbbf24",
                 font=("DejaVu Sans", 9, "bold")).pack(side=tk.LEFT, padx=(0, 6))

        self.cal_start_btn = tk.Button(
            cal_row, text="1. Zero & Start Hand-Spin", command=self.start_manual_one_rev_check,
            bg="#d97706", fg="#ffffff", font=("DejaVu Sans", 8, "bold"), relief=tk.FLAT, padx=8
        )
        self.cal_start_btn.pack(side=tk.LEFT, padx=3)

        self.cal_finish_btn = tk.Button(
            cal_row, text="2. Finish 1-Turn & Score Accuracy", command=self.finish_manual_one_rev_check,
            bg="#059669", fg="#ffffff", font=("DejaVu Sans", 8, "bold"), relief=tk.FLAT, padx=8
        )
        self.cal_finish_btn.pack(side=tk.LEFT, padx=3)

        self.apply_cpr_btn = tk.Button(
            cal_row, text="Apply Measured CPR", command=self.apply_measured_cpr,
            bg="#374151", fg=self.FG_TEXT, font=("DejaVu Sans", 8), relief=tk.FLAT, padx=8, state=tk.DISABLED
        )
        self.apply_cpr_btn.pack(side=tk.RIGHT, padx=3)

        auto_row = tk.Frame(acc_card, bg=self.BG_PANEL, padx=8, pady=5)
        auto_row.pack(fill=tk.X, pady=4)
        tk.Button(
            auto_row, text="Auto-Calibrate Hand Spins", command=self.start_auto_calibration,
            bg="#0e7490", fg="#ffffff", font=("DejaVu Sans", 9, "bold"), relief=tk.FLAT,
            padx=8, pady=3,
        ).pack(side=tk.LEFT, padx=(0, 8))
        self.auto_cal_status = tk.Label(
            auto_row, text="Mark each tire; turn M1 and M2 exactly once, one at a time.",
            bg=self.BG_PANEL, fg=self.FG_MUTED, font=("DejaVu Sans", 9),
            anchor="w", wraplength=450, justify=tk.LEFT,
        )
        self.auto_cal_status.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Latest 1-Revolution Accuracy Scorecard Box
        score_box = tk.Frame(acc_card, bg=self.BG_PANEL, padx=10, pady=8, highlightthickness=1,
                             highlightbackground="#374151")
        score_box.pack(fill=tk.X, pady=4)

        self.score_header_lbl = tk.Label(
            score_box,
            text="Latest 1-Rev Accuracy Result: Ready — Click 'Spin M1/M2/BOTH Exactly 1 Rev' or use Hand-Spin Check",
            bg=self.BG_PANEL, fg="#38bdf8", font=("DejaVu Sans Mono", 10, "bold"), anchor="w"
        )
        self.score_header_lbl.pack(fill=tk.X)

        self.score_metrics_lbl = tk.Label(
            score_box,
            text=f"Target: 1.0000 rev ({int(self.odom.counts_per_rev)} ticks)  |  Actual: —  |  Error: — ticks (—°, — mm)  |  Coast: —",
            bg=self.BG_PANEL, fg=self.FG_TEXT, font=("DejaVu Sans Mono", 9), anchor="w"
        )
        self.score_metrics_lbl.pack(fill=tk.X, pady=(2, 2))

        self.score_diag_lbl = tk.Label(
            score_box,
            text="Tip: Mark the top of the tire (0°) with tape before running a 1-revolution accuracy test.",
            bg=self.BG_PANEL, fg=self.FG_MUTED, font=("DejaVu Sans", 9), anchor="w", wraplength=600, justify=tk.LEFT
        )
        self.score_diag_lbl.pack(fill=tk.X)

        # Trial History Table
        cols = ("motor", "target", "actual", "err_ticks", "err_deg", "err_mm", "accuracy", "coast", "grade")
        self.history_tree = ttk.Treeview(acc_card, columns=cols, show="headings", height=5)
        headings = {
            "motor": ("Motor", 55),
            "target": ("Target Rev", 80),
            "actual": ("Actual Rev", 80),
            "err_ticks": ("Err Ticks", 70),
            "err_deg": ("Err Deg", 70),
            "err_mm": ("Err mm", 70),
            "accuracy": ("Accuracy %", 85),
            "coast": ("Coast Ticks", 80),
            "grade": ("Grade", 85),
        }
        for k, (title_txt, w) in headings.items():
            self.history_tree.heading(k, text=title_txt)
            self.history_tree.column(k, width=w, anchor=tk.CENTER)
        self.history_tree.pack(fill=tk.BOTH, expand=True, pady=(4, 0))

    def _build_troubleshooting_panel(self, parent: tk.Widget) -> None:
        diag_card = tk.Frame(parent, bg=self.BG_CARD, padx=12, pady=10, highlightthickness=1,
                             highlightbackground="#374151")
        diag_card.pack(fill=tk.BOTH, expand=True)

        hdr = tk.Frame(diag_card, bg=self.BG_CARD)
        hdr.pack(fill=tk.X)
        tk.Label(
            hdr, text="INSIGHTFUL TROUBLESHOOTING & SIGNAL HEALTH ANALYZER",
            bg=self.BG_CARD, fg="#fbbf24", font=("DejaVu Sans", 10, "bold")
        ).pack(side=tk.LEFT)

        self.health_badge = tk.Label(
            hdr, text="STATUS: HEALTHY", bg=self.COLOR_OK, fg="#ffffff",
            font=("DejaVu Sans Mono", 9, "bold"), padx=8, pady=2
        )
        self.health_badge.pack(side=tk.RIGHT)

        self.diag_text = tk.Text(
            diag_card, bg=self.BG_PANEL, fg=self.FG_TEXT, height=8,
            font=("DejaVu Sans Mono", 9), relief=tk.FLAT, wrap=tk.WORD, padx=8, pady=6
        )
        self.diag_text.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        self.diag_text.tag_configure("OK", foreground=self.COLOR_OK, font=("DejaVu Sans Mono", 9, "bold"))
        self.diag_text.tag_configure("WARN", foreground=self.COLOR_WARN, font=("DejaVu Sans Mono", 9, "bold"))
        self.diag_text.tag_configure("ERROR", foreground=self.COLOR_ERR, font=("DejaVu Sans Mono", 9, "bold"))
        self.diag_text.tag_configure("BODY", foreground="#d1d5db", font=("DejaVu Sans", 9))

    # =========================================================================
    # Auto-Detection, Connection & Serial Worker
    # =========================================================================
    def refresh_ports(self, probe: bool = False) -> List[Dict[str, str]]:
        ports = discover_serial_ports(allow_lidar_port=self.allow_lidar_port, probe_esp32=probe)
        devices = [p["device"] for p in ports if p["status"] in ("ESP32_DETECTED", "CANDIDATE")]
        self.port_combo["values"] = devices
        if devices and not self.port_var.get():
            self.port_var.set(devices[0])
        return ports

    def auto_detect_and_connect(self) -> None:
        """Automatically scan all USB /dev/tty* ports, identify the ESP32 via handshake, and connect."""
        self.disconnect_board()
        self.conn_status_var.set("Auto-detecting ESP32...")
        self.conn_label.configure(fg="#fbbf24")
        self.root.update_idletasks()

        detected = auto_detect_esp32_port(allow_lidar_port=self.allow_lidar_port)
        self.refresh_ports(probe=False)
        if detected:
            dev, sig = detected
            self.port_var.set(dev)
            self.sim_var.set(False)
            self.connect_board(detected_signature=sig)
        else:
            # Fall back to simulation mode if no ESP32 USB device is plugged in
            self.sim_var.set(True)
            self.connect_board()

    def on_toggle_sim(self) -> None:
        self.connect_board()

    def toggle_connection(self) -> None:
        if self.board is not None:
            self.disconnect_board()
        else:
            self.connect_board()

    def disconnect_board(self) -> None:
        self.cancel_pwm_tune()
        self.stop_reader.set()
        if self.board is not None:
            try:
                self.board.close()  # type: ignore[attr-defined]
            except Exception:
                pass
            self.board = None
        self.conn_status_var.set("Disconnected")
        self.conn_label.configure(fg=self.FG_MUTED)
        self.connect_btn.configure(text="Connect", bg="#2563eb")

    def connect_board(self, detected_signature: str = "") -> None:
        self.disconnect_board()
        self.stop_reader.clear()
        self.swap_enc_checkbox.configure(state=tk.NORMAL)
        side = self.side_var.get().upper()
        cpr = self._safe_float(self.cpr_var.get(), DEFAULT_COUNTS_PER_REV)
        radius = self._safe_float(self.radius_var.get(), DEFAULT_WHEEL_RADIUS_M)

        try:
            if self.sim_var.get():
                self.board = SimulatedESP32Board(side=side, cpr=cpr, radius_m=radius)
                self.conn_status_var.set(f"SIMULATION ({side})")
                self.conn_label.configure(fg="#38bdf8")
            else:
                port = self.port_var.get().strip()
                if not port:
                    detected = auto_detect_esp32_port(allow_lidar_port=self.allow_lidar_port)
                    if detected:
                        port, detected_signature = detected
                        self.port_var.set(port)
                if not port:
                    messagebox.showwarning(
                        "No ESP32 Detected",
                        "No ESP32 USB port found. Plug in the ESP32 and click 'Auto-Detect ESP32', "
                        "or enable 'Simulation Mode'."
                    )
                    return
                self.board = SerialESP32Board(
                    port=port, baud=DEFAULT_BAUD, side=side, cpr=cpr,
                    radius_m=radius, allow_lidar_port=self.allow_lidar_port,
                    swap_encoders=self.swap_enc_var.get(),
                )
                fw_name = getattr(self.board, "protocol", "ESP32")
                if fw_name == "ODOM_STREAM":
                    # This firmware already assigns M1/M2 to their physical
                    # pins and does not implement the app's SWAP command.
                    self.swap_enc_var.set(False)
                    self.swap_enc_checkbox.configure(state=tk.DISABLED)
                    self.send_cmd(f"CPR {cpr:.2f}")
                    self.send_cmd(f"RADIUS {radius:.4f}")
                else:
                    self.swap_enc_checkbox.configure(state=tk.NORMAL)
                self.conn_status_var.set(f"ESP32: {port} ({fw_name})")
                self.conn_label.configure(fg=self.COLOR_OK)

            self.connect_btn.configure(text="Disconnect", bg="#4b5563")
            self._update_pin_labels(side)
            threading.Thread(target=self._reader_loop, daemon=True).start()
        except Exception as exc:
            messagebox.showerror("Connection Failed", str(exc))
            self.disconnect_board()

    def _reader_loop(self) -> None:
        board_ref = self.board
        while not self.stop_reader.is_set() and board_ref is not None:
            try:
                line = board_ref.read_line()  # type: ignore[attr-defined]
                if line:
                    self.rx_queue.put(line)
            except Exception as exc:
                self.rx_queue.put(f"# ERR Serial read error: {exc}")
                break

    def send_cmd(self, cmd: str) -> None:
        if self.board is not None:
            try:
                self.board.write_command(cmd)  # type: ignore[attr-defined]
            except Exception as exc:
                self.rx_queue.put(f"# ERR Write failed: {exc}")

    # =========================================================================
    # Commands & Test Actions
    # =========================================================================
    @staticmethod
    def _safe_float(val: str, fallback: float) -> float:
        try:
            out = float(val)
            return out if out > 0 else fallback
        except ValueError:
            return fallback

    def _update_pin_labels(self, side: str) -> None:
        pins = ENCODER_PIN_MAP.get(side.upper(), ENCODER_PIN_MAP["LEFT"])
        if self.swap_enc_var.get():
            self.m1_widgets["pin_lbl"].configure(text=f"A={pins['M2_A'].split()[0]} B={pins['M2_B']} (Swapped)")
            self.m2_widgets["pin_lbl"].configure(text=f"A={pins['M1_A'].split()[0]} B={pins['M1_B']} (Swapped)")
        else:
            self.m1_widgets["pin_lbl"].configure(text=f"A={pins['M1_A'].split()[0]} B={pins['M1_B']}")
            self.m2_widgets["pin_lbl"].configure(text=f"A={pins['M2_A'].split()[0]} B={pins['M2_B']}")

    def on_toggle_swap_encoders(self) -> None:
        if isinstance(self.board, SerialESP32Board) and self.board.protocol == "ODOM_STREAM":
            self.swap_enc_var.set(False)
            self._update_pin_labels(self.side_var.get().upper())
            return
        swapped = 1 if self.swap_enc_var.get() else 0
        self.send_cmd(f"SWAP {swapped}")
        self._update_pin_labels(self.side_var.get().upper())

    def apply_parameters(self) -> None:
        side = self.side_var.get().upper()
        cpr = self._safe_float(self.cpr_var.get(), DEFAULT_COUNTS_PER_REV)
        radius = self._safe_float(self.radius_var.get(), DEFAULT_WHEEL_RADIUS_M)
        deadband = int(self._safe_float(self.deadband_var.get(), 75.0))

        self.odom.counts_per_rev = cpr
        self.odom.wheel_radius_m = radius
        self.send_cmd(f"SIDE {side}")
        self.send_cmd(f"CPR {cpr:.2f}")
        self.send_cmd(f"RADIUS {radius:.4f}")
        self.send_cmd(f"DEADBAND {deadband}")
        self._update_pin_labels(side)

    def emergency_stop(self) -> None:
        self.cancel_pwm_tune()
        self.send_cmd("STOP")

    def cancel_pwm_tune(self) -> None:
        if self.pwm_tune_motor is not None:
            self.pwm_tune_status.configure(text="PWM tuning stopped.", fg=self.COLOR_WARN)
        self.pwm_tune_motor = None
        self.pwm_tune_current_pwm = None
        self.pwm_tune_generation += 1

    def start_pwm_tune(self, motor: str) -> None:
        if self.board is None or self.sim_var.get():
            self.pwm_tune_status.configure(text="Connect the physical ESP32 first.", fg=self.COLOR_ERR)
            return
        self.cancel_pwm_tune()
        self.auto_cal_active = False
        self.pwm_tune_motor = motor
        self.pwm_tune_candidates = [90, 120, 150, 180]
        self.pwm_tune_results = []
        self.send_cmd("STOP")
        self.cpr_var.set("660.0")
        self.odom.counts_per_rev = 660.0
        self.send_cmd("CPR 660.00")
        self.send_cmd("DEADBAND 75")
        self.pwm_tune_status.configure(
            text=f"{motor}: testing PWM 90, 120, 150, 180 for one 660-tick revolution.",
            fg=self.COLOR_WARN,
        )
        self._run_next_pwm_tune_trial(self.pwm_tune_generation)

    def _run_next_pwm_tune_trial(self, generation: int) -> None:
        if generation != self.pwm_tune_generation or self.pwm_tune_motor is None:
            return
        if not self.pwm_tune_candidates:
            best_pwm, best = min(self.pwm_tune_results, key=lambda item: abs(item[1].error_ticks))
            acceptable = abs(best.error_ticks) <= 33  # 5% of the 660-tick target
            if acceptable:
                self.max_pwm_var.set(str(best_pwm))
            self.pwm_tune_status.configure(
                text=(f"{self.pwm_tune_motor}: best max PWM {best_pwm}; "
                      f"actual {best.actual_ticks}/660 ticks, error {best.error_ticks:+d}. " +
                      ("Setting saved in Max PWM field." if acceptable else
                       "No PWM was within 5%; check ENA/ENB jumper, PWM wire, and braking.")),
                fg=self.COLOR_OK if acceptable else self.COLOR_ERR,
            )
            self.pwm_tune_motor = None
            self.pwm_tune_current_pwm = None
            return
        pwm = self.pwm_tune_candidates.pop(0)
        self.pwm_tune_current_pwm = pwm
        t = self.latest_telemetry
        self.pwm_tune_start_edges = (
            (t.m1_a_edges, t.m1_b_edges) if self.pwm_tune_motor == "M1" else
            (t.m2_a_edges, t.m2_b_edges)
        )
        self.send_cmd(f"SPIN_REV {self.pwm_tune_motor} 1.0000 {pwm}")
        self.pwm_tune_status.configure(
            text=f"{self.pwm_tune_motor}: trying max PWM {pwm}; waiting for encoder result...",
            fg=self.COLOR_WARN,
        )
        self.root.after(12000, lambda g=generation, p=pwm: self._pwm_tune_timeout(g, p))

    def _pwm_tune_timeout(self, generation: int, pwm: int) -> None:
        if generation == self.pwm_tune_generation and self.pwm_tune_current_pwm == pwm:
            self.send_cmd("STOP")
            self.cancel_pwm_tune()
            self.pwm_tune_status.configure(
                text=f"No result at PWM {pwm}; stopped. Check ESP32 connection and encoder feedback.",
                fg=self.COLOR_ERR,
            )

    def _handle_pwm_tune_result(self, res: RevSpinResult) -> None:
        if self.pwm_tune_motor is None or res.motor.upper() != self.pwm_tune_motor:
            return
        pwm = self.pwm_tune_current_pwm
        if pwm is None:
            return
        self.pwm_tune_current_pwm = None
        t = self.latest_telemetry
        end_a, end_b = (
            (t.m1_a_edges, t.m1_b_edges) if res.motor.upper() == "M1" else
            (t.m2_a_edges, t.m2_b_edges)
        )
        edge_a = end_a - self.pwm_tune_start_edges[0]
        edge_b = end_b - self.pwm_tune_start_edges[1]
        if edge_a < 20 or edge_b < 20 or min(edge_a, edge_b) / max(edge_a, edge_b) < 0.7:
            self.send_cmd("STOP")
            self.cancel_pwm_tune()
            self.pwm_tune_status.configure(
                text=f"{res.motor} A/B signal failed at PWM {pwm}: {edge_a}/{edge_b} edges. Check encoder wiring.",
                fg=self.COLOR_ERR,
            )
            return
        if res.status != "COMPLETED" or res.glitches > max(3, abs(res.actual_ticks) * 0.01):
            self.send_cmd("STOP")
            self.cancel_pwm_tune()
            self.pwm_tune_status.configure(
                text=f"{res.motor} stopped at PWM {pwm}: {res.status}, {res.glitches} glitches. Check encoder signals.",
                fg=self.COLOR_ERR,
            )
            return
        self.pwm_tune_results.append((pwm, res))
        generation = self.pwm_tune_generation
        self.root.after(700, lambda g=generation: self._run_next_pwm_tune_trial(g))

    def start_auto_calibration(self) -> None:
        if self.board is None:
            self.auto_cal_status.configure(text="Connect the ESP32 first.", fg=self.COLOR_ERR)
            return
        self.send_cmd("STOP")
        t = self.latest_telemetry
        self.auto_cal_baseline = {
            "M1": (t.m1_ticks, t.m1_a_edges, t.m1_b_edges, t.m1_err),
            "M2": (t.m2_ticks, t.m2_a_edges, t.m2_b_edges, t.m2_err),
        }
        self.auto_cal_last_ticks = {"M1": t.m1_ticks, "M2": t.m2_ticks}
        self.auto_cal_last_motion = {"M1": None, "M2": None}
        self.auto_cal_results = {}
        self.auto_cal_started_at = time.monotonic()
        self.auto_cal_active = True
        self.apply_cpr_btn.configure(state=tk.DISABLED)
        self.auto_cal_status.configure(
            text="Armed: turn M1 exactly once, let it stop, then turn M2 exactly once. No motor power.",
            fg=self.COLOR_WARN,
        )

    def _update_auto_calibration(self, t: WheelTelemetry) -> None:
        if not self.auto_cal_active:
            return
        now = time.monotonic()
        if now - self.auto_cal_started_at > 60:
            self.auto_cal_active = False
            missing = ", ".join(m for m in ("M1", "M2") if m not in self.auto_cal_results)
            self.auto_cal_status.configure(
                text=f"Timed out waiting for {missing}. Check encoder power, ground, and A/B wiring; retry.",
                fg=self.COLOR_ERR,
            )
            return
        for motor in ("M1", "M2"):
            if motor in self.auto_cal_results:
                continue
            ticks, a_edges, b_edges, errors, pwm = (
                (t.m1_ticks, t.m1_a_edges, t.m1_b_edges, t.m1_err, t.m1_pwm)
                if motor == "M1" else
                (t.m2_ticks, t.m2_a_edges, t.m2_b_edges, t.m2_err, t.m2_pwm)
            )
            if pwm != 0:
                self.auto_cal_active = False
                self.auto_cal_status.configure(text="Canceled: a motor was powered during hand-spin calibration.", fg=self.COLOR_ERR)
                return
            if ticks != self.auto_cal_last_ticks[motor]:
                self.auto_cal_last_ticks[motor] = ticks
                self.auto_cal_last_motion[motor] = now
                continue
            last_motion = self.auto_cal_last_motion[motor]
            if last_motion is None or now - last_motion < 1.5:
                continue
            base_ticks, base_a, base_b, base_errors = self.auto_cal_baseline[motor]
            delta = abs(ticks - base_ticks)
            edge_a = a_edges - base_a
            edge_b = b_edges - base_b
            glitches = errors - base_errors
            if delta < 100 or edge_a < 20 or edge_b < 20 or glitches > max(3, delta * 0.01):
                self.auto_cal_active = False
                self.auto_cal_status.configure(
                    text=f"{motor} signal failed: {delta} ticks, A/B {edge_a}/{edge_b}, {glitches} glitches. Check wiring; retry.",
                    fg=self.COLOR_ERR,
                )
                return
            if min(edge_a, edge_b) / max(edge_a, edge_b) < 0.7:
                self.auto_cal_active = False
                self.auto_cal_status.configure(
                    text=f"{motor} A/B edges are unbalanced ({edge_a}/{edge_b}); check both channels, then retry.",
                    fg=self.COLOR_ERR,
                )
                return
            self.auto_cal_results[motor] = float(delta)
            self.auto_cal_status.configure(
                text=f"{motor}: {delta} counts in one turn. "
                     f"Turn {'M2' if motor == 'M1' else 'M1'} exactly once, then stop.",
                fg=self.COLOR_WARN,
            )
        if len(self.auto_cal_results) == 2:
            m1 = self.auto_cal_results["M1"]
            m2 = self.auto_cal_results["M2"]
            self.auto_cal_active = False
            self.measured_cpr_pair = (m1, m2)
            if abs(m1 - m2) / max(m1, m2) > 0.10:
                self.auto_cal_status.configure(
                    text=f"Mismatch: M1={m1:.0f}, M2={m2:.0f} CPR. Check one-turn marks and wiring; CPR not changed.",
                    fg=self.COLOR_ERR,
                )
                return
            self.apply_cpr_btn.configure(state=tk.NORMAL)
            chosen = (m1 + m2) / 2
            self.cpr_var.set(f"{chosen:.1f}")
            self.odom.counts_per_rev = chosen
            self.send_cmd(f"CPR {chosen:.2f}")
            self.auto_cal_status.configure(
                text=f"Calibrated: M1={m1:.0f}, M2={m2:.0f}; shared CPR={chosen:.1f} applied.",
                fg=self.COLOR_OK,
            )

    def zero_all(self) -> None:
        self.cancel_pwm_tune()
        self.send_cmd("ZERO")
        self.odom.reset()
        self.trace_history.clear()

    def jog_motor(self, motor: str, direction: int) -> None:
        self.cancel_pwm_tune()
        pwm_mag = int(self._safe_float(self.jog_pwm_var.get(), 120.0))
        dur_ms = int(self._safe_float(self.jog_ms_var.get(), 500.0))
        pwm = direction * pwm_mag
        self.send_cmd(f"JOG {motor} {pwm} {dur_ms}")

    def invert_motor_sign(self, motor: str) -> None:
        if motor == "M1":
            self.m1_sign *= -1
        else:
            self.m2_sign *= -1
        self.send_cmd(f"INVERT {self.m1_sign} {self.m2_sign}")

    def sim_hand_spin(self, motor: str, turns: float) -> None:
        if not self.sim_var.get():
            messagebox.showinfo(
                "Physical ESP32 Connected",
                "Your physical ESP32 is connected! Spin the physical wheel by hand to watch live ticks & revolutions."
            )
            return
        self.send_cmd(f"SPIN {motor} {turns}")

    def start_closed_loop_rev_test(self, target_motor: str) -> None:
        self.cancel_pwm_tune()
        try:
            revs = float(self.target_rev_var.get())
        except ValueError:
            revs = 1.0
        max_pwm = int(self._safe_float(self.max_pwm_var.get(), 135.0))
        deadband = int(self._safe_float(self.deadband_var.get(), 75.0))
        self.target_revs_marker = revs
        self.send_cmd(f"DEADBAND {deadband}")
        self.send_cmd("ZERO")
        self.odom.reset()
        self.send_cmd(f"SPIN_REV {target_motor} {revs:.4f} {max_pwm}")
        self.score_header_lbl.configure(
            text=f"Running Closed-Loop {revs:.3f} Rev Test on {target_motor} (Max PWM={max_pwm})...",
            fg="#fbbf24"
        )

    def start_manual_one_rev_check(self) -> None:
        self.cancel_pwm_tune()
        self.send_cmd("STOP")
        self.send_cmd("ZERO")
        self.odom.reset()
        self.manual_cal_Active = True
        self.manual_cal_start_ticks = (0, 0)
        self.score_header_lbl.configure(
            text="MANUAL 1-TURN ACTIVE: Spin wheel(s) by hand 1.000 full revolution, then click '2. Finish 1-Turn'",
            fg="#fbbf24"
        )

    def finish_manual_one_rev_check(self) -> None:
        try:
            target_revs = float(self.target_rev_var.get())
            if abs(target_revs) < 1e-4:
                target_revs = 1.0
        except ValueError:
            target_revs = 1.0

        t = self.latest_telemetry
        cpr = self.odom.counts_per_rev
        radius = self.odom.wheel_radius_m

        moved_motors: List[Tuple[str, int]] = []
        if abs(t.m1_ticks) > 0 or abs(t.m2_ticks) == 0:
            moved_motors.append(("M1", t.m1_ticks))
        if abs(t.m2_ticks) > 0:
            moved_motors.append(("M2", t.m2_ticks))

        for m_name, ticks in moved_motors:
            res = evaluate_one_rev_accuracy(
                motor=f"{m_name}-HAND",
                target_revs=target_revs,
                actual_ticks=ticks,
                counts_per_rev=cpr,
                wheel_radius_m=radius,
                brake_ticks=ticks,
                elapsed_ms=0,
                peak_rpm=0.0,
                glitches=t.m1_err if m_name == "M1" else t.m2_err,
                status="COMPLETED",
            )
            self._record_rev_result(res)

        m1_cpr, _ = estimate_cpr_from_manual_turns(t.m1_ticks, abs(target_revs))
        m2_cpr, _ = estimate_cpr_from_manual_turns(t.m2_ticks, abs(target_revs))
        self.measured_cpr_pair = (m1_cpr, m2_cpr)
        if m1_cpr > 10 or m2_cpr > 10:
            self.apply_cpr_btn.configure(state=tk.NORMAL)
            self.score_diag_lbl.configure(
                text=(
                    f"Measured CPR from {abs(target_revs):.2f} manual turn(s):  "
                    f"M1 = {m1_cpr:.1f} counts/rev ({t.m1_ticks:+d} ticks)  |  "
                    f"M2 = {m2_cpr:.1f} counts/rev ({t.m2_ticks:+d} ticks).  "
                    "Click 'Apply Measured CPR' to calibrate!"
                )
            )
        self.manual_cal_Active = False

    def apply_measured_cpr(self) -> None:
        if not self.measured_cpr_pair:
            return
        m1_cpr, m2_cpr = self.measured_cpr_pair
        chosen = m1_cpr if m1_cpr > 10 else m2_cpr
        if m1_cpr > 10 and m2_cpr > 10:
            chosen = 0.5 * (m1_cpr + m2_cpr)
        if chosen > 10:
            self.cpr_var.set(f"{chosen:.1f}")
            self.apply_parameters()
            messagebox.showinfo("CPR Calibrated", f"Updated Counts-Per-Revolution (CPR) to {chosen:.1f}.")

    def _record_rev_result(self, res: RevSpinResult) -> None:
        self.rev_history.append(res)
        color = self.COLOR_OK if res.accuracy_pct >= 97.0 else (
            self.COLOR_WARN if res.accuracy_pct >= 93.0 else self.COLOR_ERR
        )
        self.score_header_lbl.configure(
            text=f"[{res.motor}] 1-Rev Accuracy: {res.accuracy_pct:.2f}% ({res.grade}) — Status: {res.status}",
            fg=color,
        )
        self.score_metrics_lbl.configure(
            text=(
                f"Target: {res.target_revs:+.4f} rev ({res.target_ticks} ticks)  |  "
                f"Actual: {res.actual_revs:+.4f} rev ({res.actual_ticks} ticks)  |  "
                f"Error: {res.error_ticks:+d} ticks ({res.error_deg:+.2f}°, {res.error_mm:+.2f} mm)  |  "
                f"Coast: {res.coast_ticks:+d} ticks  |  Peak: {res.peak_rpm:.1f} RPM"
            )
        )
        self.score_diag_lbl.configure(text=res.diagnosis)

        self.history_tree.insert(
            "", 0,
            values=(
                res.motor,
                f"{res.target_revs:.4f}",
                f"{res.actual_revs:.4f}",
                f"{res.error_ticks:+d}",
                f"{res.error_deg:+.2f}°",
                f"{res.error_mm:+.2f} mm",
                f"{res.accuracy_pct:.2f}%",
                f"{res.coast_ticks:+d}",
                res.grade,
            ),
        )

    def save_report_json(self) -> None:
        out_path = REPO_ROOT / "artifacts" / "pi5" / "wheel-odometer-gui-report.json"
        out_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "wheel_radius_m": self.odom.wheel_radius_m,
            "track_width_m": self.odom.track_width_m,
            "counts_per_rev": self.odom.counts_per_rev,
            "pose": {**asdict(self.odom.pose), "theta_deg": self.odom.pose.theta_deg},
            "latest_telemetry": asdict(self.latest_telemetry),
            "rev_spin_history": [asdict(r) for r in self.rev_history],
        }
        out_path.write_text(json.dumps(payload, indent=2) + "\n")
        messagebox.showinfo("Report Saved", f"Saved odometer & 1-revolution accuracy report to:\n{out_path}")

    # =========================================================================
    # Periodic UI Refresh, Wheel Dials, Scope & Troubleshooting
    # =========================================================================
    def _periodic_ui_tick(self) -> None:
        updated = False
        while not self.rx_queue.empty():
            line = self.rx_queue.get_nowait()
            if line.startswith("REV_RESULT "):
                res = parse_rev_result_line(
                    line, counts_per_rev=self.odom.counts_per_rev, wheel_radius_m=self.odom.wheel_radius_m
                )
                if res:
                    self._record_rev_result(res)
                    self._handle_pwm_tune_result(res)
                continue

            parsed = parse_odom_line(line)
            if parsed is not None:
                now_mono = parsed.host_timestamp_s
                dt_s = 0.05 if self._prev_telem_mono is None else max(1e-3, now_mono - self._prev_telem_mono)
                self._prev_telem_mono = now_mono
                self.latest_telemetry = parsed
                self._update_auto_calibration(parsed)

                self.m1_metrics = compute_wheel_metrics(
                    ticks=parsed.m1_ticks,
                    dt_ticks=parsed.m1_dt,
                    dt_s=dt_s,
                    counts_per_rev=self.odom.counts_per_rev,
                    wheel_radius_m=self.odom.wheel_radius_m,
                    pwm=parsed.m1_pwm,
                    ab_state=parsed.m1_ab,
                    glitches=parsed.m1_err,
                    a_edges=parsed.m1_a_edges,
                    b_edges=parsed.m1_b_edges,
                )
                self.m2_metrics = compute_wheel_metrics(
                    ticks=parsed.m2_ticks,
                    dt_ticks=parsed.m2_dt,
                    dt_s=dt_s,
                    counts_per_rev=self.odom.counts_per_rev,
                    wheel_radius_m=self.odom.wheel_radius_m,
                    pwm=parsed.m2_pwm,
                    ab_state=parsed.m2_ab,
                    glitches=parsed.m2_err,
                    a_edges=parsed.m2_a_edges,
                    b_edges=parsed.m2_b_edges,
                )
                self.odom.update_from_ticks(parsed.m1_ticks, parsed.m2_ticks, timestamp_s=now_mono)
                rel_t = now_mono - self._start_mono
                self.trace_history.append(
                    (rel_t, self.m1_metrics.revolutions, self.m2_metrics.revolutions, self.target_revs_marker)
                )
                updated = True

        if updated:
            self._update_wheel_card_ui(self.m1_widgets, self.m1_metrics, self.ACCENT_M1)
            self._update_wheel_card_ui(self.m2_widgets, self.m2_metrics, self.ACCENT_M2)
            p = self.odom.pose
            self.pose_label.configure(
                text=(
                    f"2D Odometry:  X = {p.x_m:+.3f} m   Y = {p.y_m:+.3f} m   "
                    f"Yaw = {p.theta_deg:+6.1f}°   Path = {p.total_distance_m:.3f} m   "
                    f"v = {p.linear_mps:+.2f} m/s"
                )
            )
            self._draw_scope()
            self._update_diagnostics_ui()

        self.root.after(40, self._periodic_ui_tick)

    def _update_wheel_card_ui(self, w: Dict[str, tk.Widget], m: WheelMetrics, accent: str) -> None:
        w["rev_lbl"].configure(text=f"{m.revolutions:+.4f} rev")
        w["deg_lbl"].configure(text=f"Angle: {m.angle_deg:+.1f}°  (Dial: {m.wrapped_deg:5.1f}°)")
        w["ticks_lbl"].configure(text=f"Ticks: {m.ticks:+d}  (CPR={self.odom.counts_per_rev:.0f})")
        w["dist_lbl"].configure(text=f"Distance: {m.distance_m:+.4f} m ({m.distance_m * 100.0:+.1f} cm)")
        w["speed_lbl"].configure(text=f"Speed: {m.rpm:+6.1f} RPM ({m.velocity_mps:+.3f} m/s)")
        w["ab_lbl"].configure(
            text=f"Edges A/B: {m.a_edges} / {m.b_edges} (Sym: {m.ab_balance_pct:.0f}%)",
            fg=self.COLOR_OK if m.ab_balance_pct >= 80.0 else self.COLOR_WARN,
        )
        w["err_lbl"].configure(
            text=f"Glitches: {m.glitches}  |  PWM: {m.pwm:+d}  |  AB: {m.ab_state}",
            fg=self.COLOR_ERR if m.glitches > 0 else self.FG_MUTED,
        )
        self._draw_wheel_dial(w["dial"], m, accent)  # type: ignore[arg-type]

    def _draw_wheel_dial(self, canvas: tk.Canvas, m: WheelMetrics, accent: str) -> None:
        canvas.delete("all")
        cx, cy, r = 75, 75, 56

        canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#4b5563", width=6)
        canvas.create_oval(cx - r + 6, cy - r + 6, cx + r - 6, cy + r - 6, outline=accent, width=2)

        for deg in range(0, 360, 45):
            rad = math.radians(deg - 90.0)
            x1 = cx + (r - 8) * math.cos(rad)
            y1 = cy + (r - 8) * math.sin(rad)
            x2 = cx + r * math.cos(rad)
            y2 = cy + r * math.sin(rad)
            canvas.create_line(x1, y1, x2, y2, fill="#9ca3af", width=2 if deg % 90 == 0 else 1)

        canvas.create_text(cx, cy - r - 9, text="0° / 1 Rev", fill="#10b981", font=("DejaVu Sans", 8, "bold"))

        needle_rad = math.radians(m.wrapped_deg - 90.0)
        nx = cx + (r - 12) * math.cos(needle_rad)
        ny = cy + (r - 12) * math.sin(needle_rad)
        canvas.create_line(cx, cy, nx, ny, fill=accent, width=4)
        canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#f9fafb", outline=accent)

        canvas.create_text(cx, cy + 20, text=f"{m.revolutions:+.2f}r", fill="#f9fafb",
                           font=("DejaVu Sans Mono", 9, "bold"))

        a_high = len(m.ab_state) >= 1 and m.ab_state[0] == "1"
        b_high = len(m.ab_state) >= 2 and m.ab_state[1] == "1"
        canvas.create_oval(10, 130, 22, 142, fill=self.COLOR_OK if a_high else "#374151", outline="")
        canvas.create_text(30, 136, text="A", fill=self.FG_TEXT, font=("DejaVu Sans Mono", 8, "bold"))
        canvas.create_oval(116, 130, 128, 142, fill=self.COLOR_OK if b_high else "#374151", outline="")
        canvas.create_text(136, 136, text="B", fill=self.FG_TEXT, font=("DejaVu Sans Mono", 8, "bold"))

    def _draw_scope(self) -> None:
        c = self.scope_canvas
        c.delete("all")
        w = max(c.winfo_width(), 300)
        h = max(c.winfo_height(), 140)
        pad_l, pad_r, pad_t, pad_b = 48, 16, 14, 22
        plot_w = max(10, w - pad_l - pad_r)
        plot_h = max(10, h - pad_t - pad_b)

        if len(self.trace_history) < 2:
            return

        t_max = self.trace_history[-1][0]
        t_min = max(0.0, t_max - 10.0)
        visible = [pt for pt in self.trace_history if pt[0] >= t_min]
        if len(visible) < 2:
            return

        vals = [0.0, self.target_revs_marker]
        for _, r1, r2, _ in visible:
            vals.extend((r1, r2))
        y_min = min(vals) - 0.15
        y_max = max(vals) + 0.15
        if abs(y_max - y_min) < 0.2:
            y_max += 0.2

        def to_xy(t_val: float, y_val: float) -> Tuple[float, float]:
            x = pad_l + ((t_val - t_min) / max(1e-3, t_max - t_min)) * plot_w
            y = pad_t + (1.0 - (y_val - y_min) / (y_max - y_min)) * plot_h
            return x, y

        _, y_zero = to_xy(t_min, 0.0)
        c.create_line(pad_l, y_zero, w - pad_r, y_zero, fill="#374151", width=1)
        c.create_text(pad_l - 6, y_zero, text="0.0r", fill=self.FG_MUTED, anchor="e", font=("DejaVu Sans Mono", 8))

        _, y_targ = to_xy(t_min, self.target_revs_marker)
        c.create_line(pad_l, y_targ, w - pad_r, y_targ, fill=self.COLOR_OK, dash=(4, 4), width=1)
        c.create_text(
            pad_l - 6, y_targ, text=f"{self.target_revs_marker:.1f}r",
            fill=self.COLOR_OK, anchor="e", font=("DejaVu Sans Mono", 8, "bold")
        )

        pts_m1: List[float] = []
        pts_m2: List[float] = []
        for t_val, r1, r2, _ in visible:
            x, y1 = to_xy(t_val, r1)
            _, y2 = to_xy(t_val, r2)
            pts_m1.extend((x, y1))
            pts_m2.extend((x, y2))

        if len(pts_m1) >= 4:
            c.create_line(*pts_m1, fill=self.ACCENT_M1, width=2)
        if len(pts_m2) >= 4:
            c.create_line(*pts_m2, fill=self.ACCENT_M2, width=2)

    def _update_diagnostics_ui(self) -> None:
        findings: List[DiagnosticFinding] = diagnose_wheel_health(
            self.latest_telemetry, counts_per_rev=self.odom.counts_per_rev
        )
        worst = "OK"
        for f in findings:
            if f.severity == "ERROR":
                worst = "ERROR"
                break
            if f.severity == "WARN":
                worst = "WARN"

        badge_color = self.COLOR_OK if worst == "OK" else (self.COLOR_WARN if worst == "WARN" else self.COLOR_ERR)
        self.health_badge.configure(text=f"SIGNAL HEALTH: {worst}", bg=badge_color)

        self.diag_text.configure(state=tk.NORMAL)
        self.diag_text.delete("1.0", tk.END)
        for f in findings:
            self.diag_text.insert(tk.END, f"[{f.severity}] {f.title}\n", f.severity)
            self.diag_text.insert(tk.END, f"  {f.detail}\n\n", "BODY")
        self.diag_text.configure(state=tk.DISABLED)

    def on_close(self) -> None:
        self.disconnect_board()
        self.root.destroy()


def launch_gui(
    port: Optional[str] = None,
    side: str = "left",
    cpr: float = 660.0,
    wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_M,
    track_width_m: float = DEFAULT_TRACK_WIDTH_M,
    simulate: bool = False,
    allow_lidar_port: bool = False,
) -> int:
    root = tk.Tk()
    WheelOdometerGUI(
        root=root,
        port=port,
        side=side,
        cpr=cpr,
        wheel_radius_m=wheel_radius_m,
        track_width_m=track_width_m,
        simulate=simulate,
        allow_lidar_port=allow_lidar_port,
    )
    root.mainloop()
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port for ESP32 (auto-detected if omitted)")
    parser.add_argument("--side", choices=["left", "right"], default="left", help="Board side (left or right)")
    parser.add_argument("--cpr", type=float, default=660.0, help="Counts per wheel revolution")
    parser.add_argument("--wheel-radius", type=float, default=DEFAULT_WHEEL_RADIUS_M, help="Wheel radius in meters")
    parser.add_argument("--track-width", type=float, default=DEFAULT_TRACK_WIDTH_M, help="Track width in meters")
    parser.add_argument("--simulate", action="store_true", help="Start in offline simulation mode")
    parser.add_argument("--allow-lidar-port", action="store_true", help="Override LiDAR CP2102 port guard")
    args = parser.parse_args(argv)
    return launch_gui(
        port=args.port,
        side=args.side,
        cpr=args.cpr,
        wheel_radius_m=args.wheel_radius,
        track_width_m=args.track_width,
        simulate=args.simulate,
        allow_lidar_port=args.allow_lidar_port,
    )


if __name__ == "__main__":
    raise SystemExit(main())
