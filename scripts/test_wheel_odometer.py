#!/usr/bin/env python3
"""Manual wheel odometer, auto-detection, troubleshooting & 1-revolution accuracy engine for Kenny's ESP32 boards.

Supports:
  - Automatic ESP32 /dev/tty* port detection (probing for KENNY_SIDE_V2/V3, ODOM, or ESP32 ROM bootloader
    while skipping phantom /dev/ttyS* ports and YDLIDAR binary streams)
  - Native support for the flashed KENNY_SIDE_V3 firmware protocol (HELLO, STATUS, SIDE L/R, ARM, P, V, STOP)
    as well as ODOM streaming firmware
  - Automated hardware troubleshooting & signal health diagnostics (diagnose_wheel_health)
  - 1-Revolution accuracy evaluator (ticks, degrees, mm, overshoot/coast, accuracy %)
  - Simulated ESP32 board for offline testing
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple


LIDAR_BY_ID_SUBSTRING = "Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001"
DEFAULT_WHEEL_RADIUS_M = 0.04      # 80 mm wheel diameter -> 0.04 m radius
DEFAULT_TRACK_WIDTH_M = 0.235      # Provisional center-to-center wheel track
DEFAULT_COUNTS_PER_REV = 1320.0    # Provisional 4x quadrature counts per wheel revolution
DEFAULT_BAUD = 115200
KENNY_V3_MAX_PWM = 191             # Max PWM accepted by KENNY_SIDE_V3 'P <seq> <pwm1> <pwm2>'

# Pin lookup table matching docs/ESP32_MOTOR_WIRING.md
ENCODER_PIN_MAP = {
    "LEFT": {"M1_A": "GPIO34 (no internal pull-up)", "M1_B": "GPIO25", "M2_A": "GPIO18", "M2_B": "GPIO21"},
    "RIGHT": {"M1_A": "GPIO34 (no internal pull-up)", "M1_B": "GPIO25", "M2_A": "GPIO18", "M2_B": "GPIO21"},
}


@dataclass
class WheelTelemetry:
    """Parsed telemetry snapshot from one ESP32 board (2 motors)."""
    side: str = "LEFT"
    ms: int = 0
    armed: int = 0
    seq: int = 0
    m1_ticks: int = 0
    m2_ticks: int = 0
    m1_dt: int = 0
    m2_dt: int = 0
    m1_revs: float = 0.0
    m2_revs: float = 0.0
    m1_m: float = 0.0
    m2_m: float = 0.0
    m1_mps: float = 0.0
    m2_mps: float = 0.0
    m1_pwm: int = 0
    m2_pwm: int = 0
    m1_ab: str = "00"
    m2_ab: str = "00"
    m1_err: int = 0
    m2_err: int = 0
    m1_a_edges: int = 0
    m1_b_edges: int = 0
    m2_a_edges: int = 0
    m2_b_edges: int = 0
    firmware: str = "KENNY_SIDE_V3"
    host_timestamp_s: float = field(default_factory=time.monotonic)


@dataclass
class WheelMetrics:
    """Computed host-side odometer metrics for a single wheel."""
    ticks: int = 0
    revolutions: float = 0.0
    angle_deg: float = 0.0
    wrapped_deg: float = 0.0
    distance_m: float = 0.0
    velocity_mps: float = 0.0
    rpm: float = 0.0
    pwm: int = 0
    ab_state: str = "00"
    glitches: int = 0
    a_edges: int = 0
    b_edges: int = 0
    ab_balance_pct: float = 100.0


@dataclass
class RevSpinResult:
    """Accuracy and dynamics report from a 1-revolution (or N-revolution) spin test."""
    motor: str = "M1"
    status: str = "COMPLETED"
    target_revs: float = 1.0
    actual_revs: float = 1.0
    target_ticks: int = 1320
    actual_ticks: int = 1320
    error_ticks: int = 0
    error_revs: float = 0.0
    error_deg: float = 0.0
    error_mm: float = 0.0
    accuracy_pct: float = 100.0
    brake_ticks: int = 1320
    coast_ticks: int = 0
    elapsed_ms: int = 0
    peak_rpm: float = 0.0
    glitches: int = 0
    grade: str = "EXCELLENT"
    diagnosis: str = "On target."


@dataclass
class DiagnosticFinding:
    """A troubleshooting insight derived from encoder/motor telemetry."""
    severity: str   # "OK", "INFO", "WARN", "ERROR"
    motor: str      # "M1", "M2", "BOARD", "BOTH"
    code: str
    title: str
    detail: str


@dataclass
class OdometryPose:
    """Integrated 2D differential-drive odometry pose."""
    x_m: float = 0.0
    y_m: float = 0.0
    theta_rad: float = 0.0
    linear_mps: float = 0.0
    angular_rps: float = 0.0
    total_distance_m: float = 0.0
    left_distance_m: float = 0.0
    right_distance_m: float = 0.0

    @property
    def theta_deg(self) -> float:
        return math.degrees(self.theta_rad)


def wrap_angle_rad(angle: float) -> float:
    """Wrap angle in radians to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compute_ab_balance_pct(a_edges: int, b_edges: int) -> float:
    """Return channel A vs B edge symmetry percentage (100% = identical edge counts)."""
    peak = max(a_edges, b_edges)
    if peak == 0:
        return 100.0
    return max(0.0, (1.0 - abs(a_edges - b_edges) / float(peak)) * 100.0)


def compute_wheel_metrics(
    ticks: int,
    dt_ticks: int,
    dt_s: float,
    counts_per_rev: float,
    wheel_radius_m: float,
    pwm: int = 0,
    ab_state: str = "00",
    glitches: int = 0,
    a_edges: int = 0,
    b_edges: int = 0,
) -> WheelMetrics:
    """Convert raw encoder ticks into revolutions, angle (deg), linear distance (m), m/s, and RPM."""
    if counts_per_rev <= 0:
        raise ValueError("counts_per_rev must be positive")
    if wheel_radius_m <= 0:
        raise ValueError("wheel_radius_m must be positive")

    circumference_m = 2.0 * math.pi * wheel_radius_m
    revolutions = ticks / counts_per_rev
    angle_deg = revolutions * 360.0
    wrapped_deg = angle_deg % 360.0
    distance_m = revolutions * circumference_m
    if dt_s > 1e-6:
        rev_per_s = (dt_ticks / counts_per_rev) / dt_s
        velocity_mps = rev_per_s * circumference_m
        rpm = rev_per_s * 60.0
    else:
        velocity_mps = 0.0
        rpm = 0.0

    return WheelMetrics(
        ticks=ticks,
        revolutions=revolutions,
        angle_deg=angle_deg,
        wrapped_deg=wrapped_deg,
        distance_m=distance_m,
        velocity_mps=velocity_mps,
        rpm=rpm,
        pwm=pwm,
        ab_state=ab_state,
        glitches=glitches,
        a_edges=a_edges,
        b_edges=b_edges,
        ab_balance_pct=compute_ab_balance_pct(a_edges, b_edges),
    )


def estimate_cpr_from_manual_turns(delta_ticks: int, turns: float) -> Tuple[float, int]:
    """Calculate counts-per-revolution (CPR) and encoder direction sign from manual wheel turns."""
    if turns <= 0:
        raise ValueError("turns must be positive")
    sign = 1 if delta_ticks >= 0 else -1
    cpr = abs(delta_ticks) / float(turns)
    return cpr, sign


def evaluate_one_rev_accuracy(
    motor: str,
    target_revs: float,
    actual_ticks: int,
    counts_per_rev: float,
    wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_M,
    brake_ticks: Optional[int] = None,
    elapsed_ms: int = 0,
    peak_rpm: float = 0.0,
    glitches: int = 0,
    status: str = "COMPLETED",
) -> RevSpinResult:
    """Compute accuracy, overshoot/undershoot (ticks, deg, mm), grade, and diagnosis for a target revolution test."""
    if counts_per_rev <= 0:
        raise ValueError("counts_per_rev must be positive")
    target_ticks = int(round(target_revs * counts_per_rev))
    actual_revs = actual_ticks / counts_per_rev
    error_ticks = actual_ticks - target_ticks
    error_revs = actual_revs - target_revs
    error_deg = error_revs * 360.0
    circumference_mm = 2.0 * math.pi * wheel_radius_m * 1000.0
    error_mm = error_revs * circumference_mm
    denom = abs(target_revs) if abs(target_revs) > 1e-6 else 1.0
    accuracy_pct = max(0.0, (1.0 - abs(error_revs) / denom) * 100.0)

    b_ticks = actual_ticks if brake_ticks is None else brake_ticks
    coast_ticks = actual_ticks - b_ticks

    if status != "COMPLETED":
        grade = "FAULT"
        if "SWAPPED" in status:
            diagnosis = (
                "Encoder swap abort: motor was driven, but the OTHER motor's encoder counted the ticks! "
                "Toggle 'Swap M1<->M2 Enc' in the GUI top bar or swap the M1/M2 encoder plugs on the ESP32."
            )
        elif "REVERSE" in status:
            diagnosis = (
                "Runaway abort: wheel encoder counted opposite to motor drive! "
                "Click 'Invert Sign' for this motor or swap motor OUT wires."
            )
        elif "STALL" in status:
            diagnosis = (
                "Stall abort: motor commanded but <10 encoder ticks received. "
                "Check 12V motor power, L298N ENA/ENB jumper removal, and encoder VCC/GND."
            )
        else:
            diagnosis = f"Test did not complete normally ({status}). Check PWM deadband or mechanical load."
    else:
        if accuracy_pct >= 99.0:
            grade = "EXCELLENT"
        elif accuracy_pct >= 97.0:
            grade = "GOOD"
        elif accuracy_pct >= 94.0:
            grade = "FAIR"
        else:
            grade = "NEEDS_TUNING"

        notes: List[str] = []
        if abs(error_deg) <= 3.0:
            notes.append(f"Stopped within {error_deg:+.1f} deg ({error_mm:+.1f} mm) of target.")
        elif error_revs * (1 if target_revs >= 0 else -1) > 0:
            notes.append(
                f"Overshot by {abs(error_deg):.1f} deg ({abs(error_ticks)} ticks, coast={coast_ticks:+d} ticks after brake). "
                "Lower Max PWM or Deadband PWM to reduce braking coast."
            )
        else:
            notes.append(
                f"Undershot by {abs(error_deg):.1f} deg ({abs(error_ticks)} ticks). "
                "Increase Deadband PWM so the motor does not stall during slow approach."
            )
        if glitches > 0:
            notes.append(f"Detected {glitches} quadrature glitches during spin; check 100nF motor capacitor.")
        diagnosis = " ".join(notes)

    return RevSpinResult(
        motor=motor,
        status=status,
        target_revs=target_revs,
        actual_revs=actual_revs,
        target_ticks=target_ticks,
        actual_ticks=actual_ticks,
        error_ticks=error_ticks,
        error_revs=error_revs,
        error_deg=error_deg,
        error_mm=error_mm,
        accuracy_pct=round(accuracy_pct, 2),
        brake_ticks=b_ticks,
        coast_ticks=coast_ticks,
        elapsed_ms=elapsed_ms,
        peak_rpm=peak_rpm,
        glitches=glitches,
        grade=grade,
        diagnosis=diagnosis,
    )


def parse_kenny_side_v3_status_line(
    line: str,
    side: str = "LEFT",
    prev_t: Optional[WheelTelemetry] = None,
    counts_per_rev: float = DEFAULT_COUNTS_PER_REV,
    wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_M,
    m1_offset: int = 0,
    m2_offset: int = 0,
    m1_err_offset: int = 0,
    m2_err_offset: int = 0,
    m1_sign: int = 1,
    m2_sign: int = 1,
    swap_encoders: bool = False,
    host_time_s: Optional[float] = None,
) -> Optional[WheelTelemetry]:
    """Parse `S <ms> <armed> <seq> <m1_ticks> <m2_ticks> <m1_err> <m2_err> <m1_pwm> <m2_pwm>` from KENNY_SIDE_V3."""
    stripped = line.strip()
    if not stripped.startswith("S "):
        return None
    parts = stripped.split()
    if len(parts) != 10:
        return None
    try:
        ms = int(parts[1])
        armed = int(parts[2])
        seq = int(parts[3])
        c0_ticks = int(parts[4]) - m1_offset
        c1_ticks = int(parts[5]) - m2_offset
        c0_err = max(0, int(parts[6]) - m1_err_offset)
        c1_err = max(0, int(parts[7]) - m2_err_offset)
        use_c1_for_m1 = (side.upper() == "RIGHT") ^ bool(swap_encoders)
        if use_c1_for_m1:
            raw_m1_ticks = c1_ticks * m1_sign
            raw_m2_ticks = c0_ticks * m2_sign
            m1_err = c1_err
            m2_err = c0_err
        else:
            raw_m1_ticks = c0_ticks * m1_sign
            raw_m2_ticks = c1_ticks * m2_sign
            m1_err = c0_err
            m2_err = c1_err
        m1_pwm = int(parts[8])
        m2_pwm = int(parts[9])
    except ValueError:
        return None

    now_s = time.monotonic() if host_time_s is None else host_time_s
    if prev_t is not None:
        m1_dt = raw_m1_ticks - prev_t.m1_ticks
        m2_dt = raw_m2_ticks - prev_t.m2_ticks
        dt_s = max(1e-3, (ms - prev_t.ms) / 1000.0) if ms > prev_t.ms else max(1e-3, now_s - prev_t.host_timestamp_s)
        m1_a_edges = max(abs(raw_m1_ticks) // 2, prev_t.m1_a_edges + (abs(m1_dt) // 2 + (abs(m1_dt) % 2)))
        m1_b_edges = max(abs(raw_m1_ticks) // 2, prev_t.m1_b_edges + (abs(m1_dt) // 2))
        m2_a_edges = max(abs(raw_m2_ticks) // 2, prev_t.m2_a_edges + (abs(m2_dt) // 2 + (abs(m2_dt) % 2)))
        m2_b_edges = max(abs(raw_m2_ticks) // 2, prev_t.m2_b_edges + (abs(m2_dt) // 2))
    else:
        m1_dt = 0
        m2_dt = 0
        dt_s = 0.05
        m1_a_edges = abs(raw_m1_ticks) // 2
        m1_b_edges = abs(raw_m1_ticks) // 2
        m2_a_edges = abs(raw_m2_ticks) // 2
        m2_b_edges = abs(raw_m2_ticks) // 2

    circ_m = 2.0 * math.pi * wheel_radius_m
    m1_revs = raw_m1_ticks / counts_per_rev
    m2_revs = raw_m2_ticks / counts_per_rev
    m1_m = m1_revs * circ_m
    m2_m = m2_revs * circ_m
    m1_mps = ((m1_dt / counts_per_rev) * circ_m) / dt_s
    m2_mps = ((m2_dt / counts_per_rev) * circ_m) / dt_s

    quad_states = ("00", "01", "11", "10")
    return WheelTelemetry(
        side=side.upper(),
        ms=ms,
        armed=armed,
        seq=seq,
        m1_ticks=raw_m1_ticks,
        m2_ticks=raw_m2_ticks,
        m1_dt=m1_dt,
        m2_dt=m2_dt,
        m1_revs=m1_revs,
        m2_revs=m2_revs,
        m1_m=m1_m,
        m2_m=m2_m,
        m1_mps=m1_mps,
        m2_mps=m2_mps,
        m1_pwm=m1_pwm,
        m2_pwm=m2_pwm,
        m1_ab=quad_states[raw_m1_ticks % 4],
        m2_ab=quad_states[raw_m2_ticks % 4],
        m1_err=m1_err,
        m2_err=m2_err,
        m1_a_edges=m1_a_edges,
        m1_b_edges=m1_b_edges,
        m2_a_edges=m2_a_edges,
        m2_b_edges=m2_b_edges,
        firmware="KENNY_SIDE_V3",
        host_timestamp_s=now_s,
    )


def parse_odom_line(line: str, host_time_s: Optional[float] = None) -> Optional[WheelTelemetry]:
    """Parse an `ODOM key=value ...` line emitted by esp32_wheel_odometer_test.ino."""
    stripped = line.strip()
    if not stripped.startswith("ODOM "):
        return None
    fields: Dict[str, str] = {}
    for token in stripped.split()[1:]:
        if "=" not in token:
            continue
        k, v = token.split("=", 1)
        fields[k] = v
    try:
        return WheelTelemetry(
            side=fields.get("side", "LEFT").upper(),
            ms=int(fields.get("ms", "0")),
            armed=int(fields.get("armed", "0")),
            seq=int(fields.get("seq", "0")),
            m1_ticks=int(fields["m1_ticks"]),
            m2_ticks=int(fields["m2_ticks"]),
            m1_dt=int(fields.get("m1_dt", "0")),
            m2_dt=int(fields.get("m2_dt", "0")),
            m1_revs=float(fields.get("m1_revs", "0.0")),
            m2_revs=float(fields.get("m2_revs", "0.0")),
            m1_m=float(fields.get("m1_m", "0.0")),
            m2_m=float(fields.get("m2_m", "0.0")),
            m1_mps=float(fields.get("m1_mps", "0.0")),
            m2_mps=float(fields.get("m2_mps", "0.0")),
            m1_pwm=int(fields.get("m1_pwm", "0")),
            m2_pwm=int(fields.get("m2_pwm", "0")),
            m1_ab=fields.get("m1_ab", "00"),
            m2_ab=fields.get("m2_ab", "00"),
            m1_err=int(fields.get("m1_err", "0")),
            m2_err=int(fields.get("m2_err", "0")),
            m1_a_edges=int(fields.get("m1_a_edges", "0")),
            m1_b_edges=int(fields.get("m1_b_edges", "0")),
            m2_a_edges=int(fields.get("m2_a_edges", "0")),
            m2_b_edges=int(fields.get("m2_b_edges", "0")),
            firmware=fields.get("fw", "ODOM_STREAM"),
            host_timestamp_s=time.monotonic() if host_time_s is None else host_time_s,
        )
    except (KeyError, ValueError):
        return None


def parse_rev_result_line(
    line: str,
    counts_per_rev: float = DEFAULT_COUNTS_PER_REV,
    wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_M,
) -> Optional[RevSpinResult]:
    """Parse a `REV_RESULT key=value ...` line."""
    stripped = line.strip()
    if not stripped.startswith("REV_RESULT "):
        return None
    fields: Dict[str, str] = {}
    for token in stripped.split()[1:]:
        if "=" not in token:
            continue
        k, v = token.split("=", 1)
        fields[k] = v
    try:
        motor = fields.get("motor", "M1")
        status = fields.get("status", "COMPLETED")
        target_revs = float(fields["target_revs"])
        actual_ticks = int(fields["actual_ticks"])
        brake_ticks = int(fields.get("brake_ticks", str(actual_ticks)))
        elapsed_ms = int(fields.get("elapsed_ms", "0"))
        peak_rpm = float(fields.get("peak_rpm", "0.0"))
        glitches = int(fields.get("glitches", "0"))
        return evaluate_one_rev_accuracy(
            motor=motor,
            target_revs=target_revs,
            actual_ticks=actual_ticks,
            counts_per_rev=counts_per_rev,
            wheel_radius_m=wheel_radius_m,
            brake_ticks=brake_ticks,
            elapsed_ms=elapsed_ms,
            peak_rpm=peak_rpm,
            glitches=glitches,
            status=status,
        )
    except (KeyError, ValueError):
        return None


def diagnose_wheel_health(
    t: WheelTelemetry,
    counts_per_rev: float = DEFAULT_COUNTS_PER_REV,
) -> List[DiagnosticFinding]:
    """Analyze encoder edge balance, glitches, direction polarity, and stalls to aid troubleshooting."""
    findings: List[DiagnosticFinding] = []
    side = t.side.upper() if t.side.upper() in ENCODER_PIN_MAP else "LEFT"
    pins = ENCODER_PIN_MAP[side]

    # 0. Cross-Motor Swapped Encoders Check
    if abs(t.m1_pwm) >= 60 and t.m2_pwm == 0 and abs(t.m2_dt) >= 8 and abs(t.m1_dt) <= 2:
        findings.append(DiagnosticFinding(
            severity="ERROR",
            motor="BOTH",
            code="SWAPPED_ENCODERS",
            title="M1 & M2 encoders are swapped! (M1 driven, but M2 encoder moved)",
            detail=(
                f"M1 PWM={t.m1_pwm} while M2 PWM=0, yet M2 counted {t.m2_dt:+d} ticks and M1 counted {t.m1_dt:+d}. "
                "Check 'Swap M1<->M2 Enc' in the GUI top bar (or swap GPIO18/21 <-> GPIO34/25 encoder plugs)."
            ),
        ))
    elif abs(t.m2_pwm) >= 60 and t.m1_pwm == 0 and abs(t.m1_dt) >= 8 and abs(t.m2_dt) <= 2:
        findings.append(DiagnosticFinding(
            severity="ERROR",
            motor="BOTH",
            code="SWAPPED_ENCODERS",
            title="M1 & M2 encoders are swapped! (M2 driven, but M1 encoder moved)",
            detail=(
                f"M2 PWM={t.m2_pwm} while M1 PWM=0, yet M1 counted {t.m1_dt:+d} ticks and M2 counted {t.m2_dt:+d}. "
                "Check 'Swap M1<->M2 Enc' in the GUI top bar (or swap GPIO18/21 <-> GPIO34/25 encoder plugs)."
            ),
        ))

    for motor, ticks, dt, pwm, err, a_edges, b_edges, pin_a, pin_b in [
        ("M1", t.m1_ticks, t.m1_dt, t.m1_pwm, t.m1_err, t.m1_a_edges, t.m1_b_edges, pins["M1_A"], pins["M1_B"]),
        ("M2", t.m2_ticks, t.m2_dt, t.m2_pwm, t.m2_err, t.m2_a_edges, t.m2_b_edges, pins["M2_A"], pins["M2_B"]),
    ]:
        total_edges = a_edges + b_edges

        # 1. Missing Channel A or Channel B
        if a_edges >= 12 and b_edges == 0:
            findings.append(DiagnosticFinding(
                severity="ERROR",
                motor=motor,
                code="DEAD_CHANNEL_B",
                title=f"{motor}: Channel B has 0 edges ({pin_b})",
                detail=(
                    f"Channel A toggled {a_edges} times, but Channel B on {pin_b} has 0 transitions. "
                    "Without both channels, quadrature cannot detect direction and only counts jitter."
                ),
            ))
        elif b_edges >= 12 and a_edges == 0:
            findings.append(DiagnosticFinding(
                severity="ERROR",
                motor=motor,
                code="DEAD_CHANNEL_A",
                title=f"{motor}: Channel A has 0 edges ({pin_a})",
                detail=(
                    f"Channel B toggled {b_edges} times, but Channel A on {pin_a} has 0 transitions. "
                    "Note: GPIO34 is input-only on ESP32 and requires an external pull-up if the encoder is open-collector."
                ),
            ))
        elif total_edges >= 40:
            balance = compute_ab_balance_pct(a_edges, b_edges)
            if balance < 75.0:
                findings.append(DiagnosticFinding(
                    severity="WARN",
                    motor=motor,
                    code="AB_IMBALANCE",
                    title=f"{motor}: Channel A/B edge imbalance ({balance:.0f}% symmetry)",
                    detail=(
                        f"A={a_edges} edges ({pin_a}) vs B={b_edges} edges ({pin_b}). "
                        "A healthy quadrature encoder produces nearly equal A and B edges (~95-100%). "
                        "Check pull-ups (especially on GPIO34) or level shifting."
                    ),
                ))

        # 2. Quadrature Invalid State Transitions (Glitches)
        if err > 0:
            glitch_pct = (err / max(1, total_edges)) * 100.0
            sev = "ERROR" if glitch_pct > 2.0 or err >= 20 else "WARN"
            findings.append(DiagnosticFinding(
                severity=sev,
                motor=motor,
                code="QUADRATURE_GLITCHES",
                title=f"{motor}: {err} quadrature double-edge glitches ({glitch_pct:.1f}%)",
                detail=(
                    "Both A and B changed simultaneously between interrupts. "
                    "Verify the 100 nF (104) ceramic capacitor is across the motor terminals, "
                    "shared GND is solid, and encoder wires are separated from L298N OUT wires."
                ),
            ))

        # 3. Direction Polarity Mismatch during Active PWM
        if pwm >= 60 and dt < -4:
            findings.append(DiagnosticFinding(
                severity="WARN",
                motor=motor,
                code="INVERTED_DIRECTION",
                title=f"{motor}: Positive PWM (+{pwm}) causes NEGATIVE ticks ({dt})",
                detail=(
                    f"Motor is driven forward (+PWM) but encoder ticks are decreasing. "
                    f"Click 'Invert {motor} Sign' in the GUI or swap {motor}'s two motor power leads on the L298N."
                ),
            ))
        elif pwm <= -60 and dt > 4:
            findings.append(DiagnosticFinding(
                severity="WARN",
                motor=motor,
                code="INVERTED_DIRECTION",
                title=f"{motor}: Negative PWM ({pwm}) causes POSITIVE ticks (+{dt})",
                detail=(
                    f"Motor is driven in reverse (-PWM) but encoder ticks are increasing. "
                    f"Click 'Invert {motor} Sign' in the GUI or swap {motor}'s two motor power leads."
                ),
            ))

        # 4. Commanded PWM with Zero Encoder Response
        if abs(pwm) >= 90 and dt == 0 and total_edges == 0:
            findings.append(DiagnosticFinding(
                severity="WARN",
                motor=motor,
                code="NO_MOTION_UNDER_PWM",
                title=f"{motor}: PWM={pwm} active, but 0 encoder edges detected",
                detail=(
                    "If the wheel is spinning, check encoder VCC (3.3V/5V), GND, and A/B pins. "
                    "If the wheel is not spinning, check the 12V fuse, L298N supply, and ENA/ENB wiring."
                ),
            ))

    if not findings:
        findings.append(DiagnosticFinding(
            severity="OK",
            motor="BOTH",
            code="HEALTHY",
            title=f"All encoder channels & quadrature states healthy ({t.firmware})",
            detail=(
                f"Side={side} ({pins['M1_A']}/{pins['M1_B']} & {pins['M2_A']}/{pins['M2_B']}) | "
                f"Armed={t.armed} | M1 Glitches={t.m1_err}, M2 Glitches={t.m2_err}."
            ),
        ))
    return findings


class DifferentialDriveOdometry:
    """Integrates left and right wheel distances into 2D robot pose (x, y, theta)."""

    def __init__(
        self,
        wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_M,
        track_width_m: float = DEFAULT_TRACK_WIDTH_M,
        counts_per_rev: float = DEFAULT_COUNTS_PER_REV,
    ) -> None:
        if wheel_radius_m <= 0:
            raise ValueError("wheel_radius_m must be positive")
        if track_width_m <= 0:
            raise ValueError("track_width_m must be positive")
        if counts_per_rev <= 0:
            raise ValueError("counts_per_rev must be positive")

        self.wheel_radius_m = wheel_radius_m
        self.track_width_m = track_width_m
        self.counts_per_rev = counts_per_rev
        self.pose = OdometryPose()
        self._prev_left_m: Optional[float] = None
        self._prev_right_m: Optional[float] = None
        self._prev_time_s: Optional[float] = None

    @property
    def circumference_m(self) -> float:
        return 2.0 * math.pi * self.wheel_radius_m

    def ticks_to_meters(self, ticks: float) -> float:
        return (ticks / self.counts_per_rev) * self.circumference_m

    def reset(self) -> None:
        self.pose = OdometryPose()
        self._prev_left_m = None
        self._prev_right_m = None
        self._prev_time_s = None

    def update_from_wheel_distances(
        self,
        left_distance_m: float,
        right_distance_m: float,
        timestamp_s: Optional[float] = None,
    ) -> OdometryPose:
        """Update (x, y, theta) from cumulative left and right wheel distances in meters."""
        if self._prev_left_m is None or self._prev_right_m is None:
            self._prev_left_m = left_distance_m
            self._prev_right_m = right_distance_m
            self._prev_time_s = timestamp_s
            self.pose.left_distance_m = left_distance_m
            self.pose.right_distance_m = right_distance_m
            return self.pose

        d_left = left_distance_m - self._prev_left_m
        d_right = right_distance_m - self._prev_right_m
        self._prev_left_m = left_distance_m
        self._prev_right_m = right_distance_m

        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.track_width_m
        mid_theta = self.pose.theta_rad + 0.5 * d_theta

        self.pose.x_m += d_center * math.cos(mid_theta)
        self.pose.y_m += d_center * math.sin(mid_theta)
        self.pose.theta_rad = wrap_angle_rad(self.pose.theta_rad + d_theta)
        self.pose.total_distance_m += abs(d_center)
        self.pose.left_distance_m = left_distance_m
        self.pose.right_distance_m = right_distance_m

        if timestamp_s is not None and self._prev_time_s is not None:
            dt_s = timestamp_s - self._prev_time_s
            if dt_s > 1e-6:
                self.pose.linear_mps = d_center / dt_s
                self.pose.angular_rps = d_theta / dt_s
        self._prev_time_s = timestamp_s
        return self.pose

    def update_from_ticks(
        self,
        left_ticks: float,
        right_ticks: float,
        timestamp_s: Optional[float] = None,
    ) -> OdometryPose:
        """Update (x, y, theta) from cumulative left and right encoder ticks."""
        return self.update_from_wheel_distances(
            self.ticks_to_meters(left_ticks),
            self.ticks_to_meters(right_ticks),
            timestamp_s=timestamp_s,
        )


def is_lidar_serial_port(port_path: str, description: str = "", hwid: str = "") -> bool:
    """Return True if the serial port path or hardware ID matches the CP2102 signature."""
    combined = f"{port_path} {description} {hwid}"
    return LIDAR_BY_ID_SUBSTRING in combined


def probe_port_for_esp32(port_path: str, baud: int = DEFAULT_BAUD, timeout_s: float = 0.75) -> Tuple[bool, str]:
    """Actively probe a serial port to determine if it is an ESP32 motor board vs. a YDLIDAR or other device.

    Returns (is_esp32, signature_or_reason).
    """
    try:
        import serial
    except ImportError:
        return False, "pyserial not installed"

    try:
        with serial.Serial(port_path, baudrate=baud, timeout=0.15) as ser:
            # Wait briefly in case opening the port triggered ESP32 DTR/RTS auto-reset (~400ms boot)
            deadline = time.monotonic() + timeout_s
            collected = bytearray()
            sent_hello = False
            while time.monotonic() < deadline:
                chunk = ser.read(256)
                if chunk:
                    collected.extend(chunk)
                    # Check for YDLIDAR binary packet header (0xAA 0x55) without ASCII text
                    if b"\xaa\x55" in collected and b"KENNY" not in collected and b"ODOM" not in collected:
                        return False, "YDLIDAR binary scan stream detected"
                    text = collected.decode("utf-8", errors="ignore")
                    for marker in ("KENNY_SIDE_V3", "KENNY_SIDE_V2", "ODOM side=", "OK STOP"):
                        if marker in text:
                            for line in text.splitlines():
                                if marker in line:
                                    return True, line.strip()
                            return True, marker
                if not sent_hello and (time.monotonic() + 0.30 >= deadline or b"READY_DISABLED" in collected):
                    ser.reset_input_buffer()
                    ser.write(b"HELLO\nSTATUS\n")
                    ser.flush()
                    sent_hello = True
                time.sleep(0.03)

            text = collected.decode("utf-8", errors="ignore")
            if "SPI_FAST_FLASH_BOOT" in text or "ets Jul 29 2019" in text:
                return True, "ESP32 ROM Bootloader"
            return False, "No ESP32 response"
    except Exception as exc:
        return False, f"Port busy or inaccessible ({exc})"


def discover_serial_ports(allow_lidar_port: bool = False, probe_esp32: bool = True) -> List[Dict[str, str]]:
    """List USB serial ports on the system and automatically probe which /dev/tty* is the ESP32."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []

    results: List[Dict[str, str]] = []
    for info in sorted(list_ports.comports(), key=lambda p: p.device):
        # Skip phantom non-USB onboard /dev/ttyS* ports
        is_usb = (info.vid is not None) or ("ttyUSB" in info.device) or ("ttyACM" in info.device)
        if not is_usb:
            continue

        status = "CANDIDATE"
        signature = ""
        if probe_esp32:
            is_esp, sig = probe_port_for_esp32(info.device)
            signature = sig
            if is_esp:
                status = "ESP32_DETECTED"
            elif "YDLIDAR" in sig:
                status = "RESERVED_LIDAR"
        elif is_lidar_serial_port(info.device, info.description or "", info.hwid or "") and not allow_lidar_port:
            status = "RESERVED_LIDAR"

        results.append({
            "device": info.device,
            "description": info.description or "",
            "hwid": info.hwid or "",
            "status": status,
            "signature": signature,
        })
    return results


def auto_detect_esp32_port(allow_lidar_port: bool = False) -> Optional[Tuple[str, str]]:
    """Automatically find and verify the ESP32's /dev/tty* port. Returns (device_path, signature) or None."""
    ports = discover_serial_ports(allow_lidar_port=allow_lidar_port, probe_esp32=True)
    for p in ports:
        if p["status"] == "ESP32_DETECTED":
            return p["device"], p.get("signature", "ESP32")
    for p in ports:
        if p["status"] == "CANDIDATE":
            return p["device"], p.get("description", "USB Serial")
    return None


def flash_esp32_without_arduino_ide(port: str, allow_lidar_port: bool = False) -> Tuple[bool, str]:
    """Compile and flash esp32_wheel_odometer_test.ino using PlatformIO or arduino-cli without Arduino IDE."""
    repo_root = Path(__file__).resolve().parents[1]
    fw_dir = repo_root / "firmware" / "esp32_wheel_odometer_test"

    pio_bin = shutil.which("pio") or shutil.which("platformio")
    if pio_bin:
        cmd = [pio_bin, "run", "-d", str(fw_dir), "-t", "upload", "--upload-port", port]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        return proc.returncode == 0, (proc.stdout + "\n" + proc.stderr).strip()

    acli_bin = shutil.which("arduino-cli")
    if acli_bin:
        cmd = [
            acli_bin, "compile", "--upload",
            "-b", "esp32:esp32:esp32",
            "-p", port,
            str(fw_dir),
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        return proc.returncode == 0, (proc.stdout + "\n" + proc.stderr).strip()

    return False, (
        "Note: Your ESP32 ALREADY has KENNY_SIDE_V3 firmware flashed and works directly with this GUI!\n\n"
        "If you ever want to compile & upload custom firmware without Arduino IDE, install PlatformIO CLI:\n"
        "  pip install --user platformio\n"
        f"Then run: pio run -d firmware/esp32_wheel_odometer_test -t upload --upload-port {port}"
    )


class SimulatedESP32Board:
    """In-memory simulated ESP32 board supporting passive hand-spin, JOG, and closed-loop SPIN_REV."""

    QUAD_STATES = ("00", "01", "11", "10")

    def __init__(self, side: str = "LEFT", cpr: float = DEFAULT_COUNTS_PER_REV, radius_m: float = DEFAULT_WHEEL_RADIUS_M):
        self.side = side.upper()
        self.cpr = cpr
        self.radius_m = radius_m
        self.m1_sign = 1
        self.m2_sign = 1
        self.deadband_pwm = 75
        self.m1_ticks = 0
        self.m2_ticks = 0
        self._prev_m1_ticks = 0
        self._prev_m2_ticks = 0
        self.m1_a_edges = 0
        self.m1_b_edges = 0
        self.m2_a_edges = 0
        self.m2_b_edges = 0
        self.m1_err = 0
        self.m2_err = 0
        self.m1_pwm = 0
        self.m2_pwm = 0
        self._jog_deadline = 0.0
        self._spin_tasks: Dict[str, Dict[str, float]] = {}
        self._start_time = time.monotonic()
        self._last_step = self._start_time
        self._rx_queue: "queue.Queue[str]" = queue.Queue()
        self._lock = threading.Lock()

    def _advance_ticks(self, motor: str, delta_ticks: int) -> None:
        if delta_ticks == 0:
            return
        abs_dt = abs(delta_ticks)
        a_inc = abs_dt // 2 + (abs_dt % 2)
        b_inc = abs_dt // 2
        if motor == "M1":
            self.m1_ticks += delta_ticks
            self.m1_a_edges += a_inc
            self.m1_b_edges += b_inc
        else:
            self.m2_ticks += delta_ticks
            self.m2_a_edges += a_inc
            self.m2_b_edges += b_inc

    def write_command(self, cmd_line: str) -> None:
        parts = cmd_line.strip().split()
        if not parts:
            return
        cmd = parts[0].upper()
        with self._lock:
            if cmd in ("ZERO", "RESET", "Z"):
                self.m1_ticks = 0
                self.m2_ticks = 0
                self._prev_m1_ticks = 0
                self._prev_m2_ticks = 0
                self.m1_a_edges = 0
                self.m1_b_edges = 0
                self.m2_a_edges = 0
                self.m2_b_edges = 0
                self.m1_err = 0
                self.m2_err = 0
                self.m1_pwm = 0
                self.m2_pwm = 0
                self._spin_tasks.clear()
                self._rx_queue.put("# OK ZEROED m1_ticks=0 m2_ticks=0")
            elif cmd in ("STOP", "S"):
                self.m1_pwm = 0
                self.m2_pwm = 0
                self._spin_tasks.clear()
                self._rx_queue.put("# OK STOPPED")
            elif cmd == "SIDE" and len(parts) >= 2:
                self.side = "LEFT" if parts[1].upper().startswith("L") else "RIGHT"
                self._rx_queue.put(f"# OK SIDE {self.side}")
            elif cmd == "CPR" and len(parts) >= 2:
                self.cpr = float(parts[1])
                self._rx_queue.put(f"# OK CPR {self.cpr:.2f}")
            elif cmd == "RADIUS" and len(parts) >= 2:
                self.radius_m = float(parts[1])
                self._rx_queue.put(f"# OK RADIUS {self.radius_m:.4f}")
            elif cmd == "DEADBAND" and len(parts) >= 2:
                self.deadband_pwm = max(30, min(191, int(parts[1])))
                self._rx_queue.put(f"# OK DEADBAND {self.deadband_pwm}")
            elif cmd == "INVERT" and len(parts) >= 3:
                self.m1_sign = -1 if int(parts[1]) < 0 else 1
                self.m2_sign = -1 if int(parts[2]) < 0 else 1
                self._rx_queue.put(f"# OK INVERT m1_sign={self.m1_sign} m2_sign={self.m2_sign}")
            elif cmd == "JOG" and len(parts) >= 3:
                self._spin_tasks.clear()
                target = parts[1].upper()
                pwm = max(-191, min(191, int(parts[2])))
                dur_ms = int(parts[3]) if len(parts) >= 4 else 500
                self.m1_pwm = pwm if target in ("M1", "1", "BOTH", "ALL") else 0
                self.m2_pwm = pwm if target in ("M2", "2", "BOTH", "ALL") else 0
                self._jog_deadline = time.monotonic() + (dur_ms / 1000.0)
                self._rx_queue.put(f"# OK JOG target={target} pwm={pwm} duration_ms={dur_ms}")
            elif cmd == "SPIN_REV" and len(parts) >= 3:
                target = parts[1].upper()
                revs = float(parts[2])
                max_pwm = max(60, min(191, int(parts[3]))) if len(parts) >= 4 else 135
                self.m1_pwm = 0
                self.m2_pwm = 0
                now = time.monotonic()
                motors = ["M1", "M2"] if target in ("BOTH", "ALL") else (["M1"] if target in ("M1", "1") else ["M2"])
                for m in motors:
                    start_t = self.m1_ticks if m == "M1" else self.m2_ticks
                    target_dt = int(round(revs * self.cpr))
                    self._spin_tasks[m] = {
                        "target_revs": revs,
                        "start_ticks": float(start_t),
                        "target_delta": float(target_dt),
                        "max_pwm": float(max_pwm),
                        "start_s": now,
                        "peak_rpm": 0.0,
                    }
                self._rx_queue.put(f"# OK SPIN_REV target={target} revs={revs:.4f} max_pwm={max_pwm}")
            elif cmd == "SPIN" and len(parts) >= 3:
                target = parts[1].upper()
                turns = float(parts[2])
                add_ticks = int(round(turns * self.cpr))
                if target in ("M1", "1", "BOTH", "ALL"):
                    self._advance_ticks("M1", add_ticks * self.m1_sign)
                if target in ("M2", "2", "BOTH", "ALL"):
                    self._advance_ticks("M2", add_ticks * self.m2_sign)
                self._rx_queue.put(f"# OK SIM_SPIN target={target} turns={turns:.3f} ticks={add_ticks}")

    def read_line(self) -> Optional[str]:
        now = time.monotonic()
        with self._lock:
            if not self._rx_queue.empty():
                return self._rx_queue.get_nowait()

            dt = now - self._last_step
            if dt < 0.05:
                time.sleep(max(0.0, 0.05 - dt))
                now = time.monotonic()
                dt = now - self._last_step
            self._last_step = now

            if self._spin_tasks:
                finished_motors: List[str] = []
                for m, task in list(self._spin_tasks.items()):
                    curr_t = self.m1_ticks if m == "M1" else self.m2_ticks
                    progressed = curr_t - int(task["start_ticks"])
                    target_dt = int(task["target_delta"])
                    direction = 1 if target_dt >= 0 else -1
                    remaining = abs(target_dt) - progressed * direction
                    slow_zone = max(60.0, 0.35 * self.cpr)
                    max_pwm = int(task["max_pwm"])

                    if remaining <= 0:
                        brake_ticks = progressed
                        coast = direction * max(1, int(round((max_pwm - 70) / 25.0)))
                        self._advance_ticks(m, coast)
                        if m == "M1":
                            self.m1_pwm = 0
                        else:
                            self.m2_pwm = 0
                        final_ticks = (self.m1_ticks if m == "M1" else self.m2_ticks) - int(task["start_ticks"])
                        elapsed_ms = int((now - task["start_s"]) * 1000.0) + 80
                        res = evaluate_one_rev_accuracy(
                            motor=m,
                            target_revs=task["target_revs"],
                            actual_ticks=final_ticks,
                            counts_per_rev=self.cpr,
                            wheel_radius_m=self.radius_m,
                            brake_ticks=brake_ticks,
                            elapsed_ms=elapsed_ms,
                            peak_rpm=task["peak_rpm"],
                            glitches=0,
                            status="COMPLETED",
                        )
                        self._rx_queue.put(
                            f"REV_RESULT motor={res.motor} status={res.status} "
                            f"target_revs={res.target_revs:.4f} actual_revs={res.actual_revs:.4f} "
                            f"target_ticks={res.target_ticks} actual_ticks={res.actual_ticks} "
                            f"error_ticks={res.error_ticks:+d} error_revs={res.error_revs:+.4f} "
                            f"error_deg={res.error_deg:+.2f} error_mm={res.error_mm:+.2f} "
                            f"accuracy_pct={res.accuracy_pct:.2f} brake_ticks={res.brake_ticks} "
                            f"coast_ticks={res.coast_ticks:+d} elapsed_ms={res.elapsed_ms} "
                            f"peak_rpm={res.peak_rpm:.1f} glitches={res.glitches}"
                        )
                        finished_motors.append(m)
                    else:
                        if remaining < slow_zone:
                            ratio = remaining / slow_zone
                            cmd_mag = int(round(self.deadband_pwm + ratio * (max_pwm - self.deadband_pwm)))
                        else:
                            cmd_mag = max_pwm
                        pwm_val = direction * cmd_mag
                        if m == "M1":
                            self.m1_pwm = pwm_val
                        else:
                            self.m2_pwm = pwm_val
                        max_ticks_per_s = 1.45 * self.cpr
                        step_ticks = int(round((pwm_val / 255.0) * max_ticks_per_s * dt))
                        if step_ticks == 0:
                            step_ticks = direction * 4
                        if abs(step_ticks) > remaining + 2:
                            step_ticks = direction * remaining
                        self._advance_ticks(m, step_ticks)
                        rpm_now = abs(step_ticks / self.cpr) / max(dt, 1e-3) * 60.0
                        task["peak_rpm"] = max(task["peak_rpm"], rpm_now)

                for fm in finished_motors:
                    self._spin_tasks.pop(fm, None)
            else:
                if (self.m1_pwm != 0 or self.m2_pwm != 0) and now >= self._jog_deadline:
                    self.m1_pwm = 0
                    self.m2_pwm = 0
                    self._rx_queue.put("# INFO JOG_FINISHED motors stopped")

                max_ticks_per_s = 1.45 * self.cpr
                if self.m1_pwm != 0:
                    step1 = int(round((self.m1_pwm / 255.0) * max_ticks_per_s * dt)) * self.m1_sign
                    self._advance_ticks("M1", step1)
                if self.m2_pwm != 0:
                    step2 = int(round((self.m2_pwm / 255.0) * max_ticks_per_s * dt)) * self.m2_sign
                    self._advance_ticks("M2", step2)

            m1_dt = self.m1_ticks - self._prev_m1_ticks
            m2_dt = self.m2_ticks - self._prev_m2_ticks
            self._prev_m1_ticks = self.m1_ticks
            self._prev_m2_ticks = self.m2_ticks

            circ = 2.0 * math.pi * self.radius_m
            m1_revs = self.m1_ticks / self.cpr
            m2_revs = self.m2_ticks / self.cpr
            m1_m = m1_revs * circ
            m2_m = m2_revs * circ
            m1_mps = ((m1_dt / self.cpr) * circ) / max(dt, 1e-3)
            m2_mps = ((m2_dt / self.cpr) * circ) / max(dt, 1e-3)
            ms = int((now - self._start_time) * 1000.0)
            m1_ab = self.QUAD_STATES[self.m1_ticks % 4]
            m2_ab = self.QUAD_STATES[self.m2_ticks % 4]

            return (
                f"ODOM side={self.side} ms={ms} m1_ticks={self.m1_ticks} m2_ticks={self.m2_ticks} "
                f"m1_dt={m1_dt} m2_dt={m2_dt} m1_revs={m1_revs:.4f} m2_revs={m2_revs:.4f} "
                f"m1_m={m1_m:.4f} m2_m={m2_m:.4f} m1_mps={m1_mps:.4f} m2_mps={m2_mps:.4f} "
                f"m1_pwm={self.m1_pwm} m2_pwm={self.m2_pwm} m1_ab={m1_ab} m2_ab={m2_ab} "
                f"m1_err={self.m1_err} m2_err={self.m2_err} "
                f"m1_a_edges={self.m1_a_edges} m1_b_edges={self.m1_b_edges} "
                f"m2_a_edges={self.m2_a_edges} m2_b_edges={self.m2_b_edges}"
            )

    def close(self) -> None:
        self.m1_pwm = 0
        self.m2_pwm = 0
        self._spin_tasks.clear()


class SerialESP32Board:
    """Hardware ESP32 serial driver supporting both KENNY_SIDE_V3 (flashed on board) and ODOM streaming firmware."""

    def __init__(
        self,
        port: str,
        baud: int = DEFAULT_BAUD,
        side: str = "LEFT",
        cpr: float = DEFAULT_COUNTS_PER_REV,
        radius_m: float = DEFAULT_WHEEL_RADIUS_M,
        allow_lidar_port: bool = False,
        swap_encoders: bool = True,
    ):
        import serial

        self.port = port
        self.side = "LEFT" if side.upper().startswith("L") else "RIGHT"
        self.cpr = cpr
        self.radius_m = radius_m
        self.m1_sign = 1
        self.m2_sign = 1
        self.swap_encoders = swap_encoders
        self.deadband_pwm = 75
        self.protocol = "KENNY_SIDE_V3"
        self.firmware_banner = "KENNY_SIDE_V3"

        self._ser = serial.Serial(port, baudrate=baud, timeout=0.08)
        self._lock = threading.Lock()
        self._out_queue: "queue.Queue[str]" = queue.Queue()
        self._seq = 1
        self._prev_telem: Optional[WheelTelemetry] = None
        self._m1_offset = 0
        self._m2_offset = 0
        self._m1_err_offset = 0
        self._m2_err_offset = 0
        self._jog_active = False
        self._jog_m1_pwm = 0
        self._jog_m2_pwm = 0
        self._jog_deadline = 0.0
        self._spin_tasks: Dict[str, Dict[str, float]] = {}

        # Allow ESP32 auto-reset on port open (~450ms) and detect firmware protocol
        self._handshake()

    def _send_raw_unlocked(self, line: str) -> Optional[str]:
        self._ser.write((line.strip() + "\n").encode("ascii", errors="ignore"))
        self._ser.flush()
        raw = self._ser.readline()
        if not raw:
            return None
        return raw.decode("utf-8", errors="replace").strip()

    def _capture_baseline_unlocked(self) -> None:
        status_raw = self._send_raw_unlocked("STATUS") or ""
        parts = status_raw.strip().split()
        if len(parts) == 10 and parts[0] == "S":
            try:
                self._m1_offset = int(parts[4])
                self._m2_offset = int(parts[5])
                self._m1_err_offset = int(parts[6])
                self._m2_err_offset = int(parts[7])
            except ValueError:
                pass
        self._prev_telem = None

    def _handshake(self) -> None:
        time.sleep(0.48)
        with self._lock:
            self._ser.reset_input_buffer()
            resp = self._send_raw_unlocked("HELLO") or ""
            if "KENNY_SIDE" in resp:
                self.protocol = "KENNY_SIDE_V3"
                self.firmware_banner = resp
                self._send_raw_unlocked("STOP")
                side_char = "L" if self.side == "LEFT" else "R"
                self._send_raw_unlocked(f"SIDE {side_char}")
                self._capture_baseline_unlocked()
                self._out_queue.put(f"# Connected to {self.port}: {resp} (SIDE {side_char})")
            else:
                # Check one more line in case boot banner was queued
                extra = self._ser.read(256).decode("utf-8", errors="ignore")
                if "KENNY_SIDE" in extra or "READY_DISABLED" in extra:
                    self.protocol = "KENNY_SIDE_V3"
                    self._send_raw_unlocked("STOP")
                    side_char = "L" if self.side == "LEFT" else "R"
                    self._send_raw_unlocked(f"SIDE {side_char}")
                    self._capture_baseline_unlocked()
                    self._out_queue.put(f"# Connected to {self.port}: KENNY_SIDE_V3 (SIDE {side_char})")
                else:
                    self.protocol = "ODOM_STREAM"
                    self._out_queue.put(f"# Connected to {self.port} (ODOM stream mode)")

    def _arm_unlocked(self) -> None:
        self._seq = 1
        self._send_raw_unlocked("ARM")

    def _send_pwm_unlocked(self, m1_pwm: int, m2_pwm: int) -> None:
        p1 = max(-KENNY_V3_MAX_PWM, min(KENNY_V3_MAX_PWM, int(m1_pwm)))
        p2 = max(-KENNY_V3_MAX_PWM, min(KENNY_V3_MAX_PWM, int(m2_pwm)))
        self._send_raw_unlocked(f"P {self._seq} {p1} {p2}")
        self._seq += 1

    def write_command(self, cmd_line: str) -> None:
        parts = cmd_line.strip().split()
        if not parts:
            return
        verb = parts[0].upper()

        with self._lock:
            if self.protocol != "KENNY_SIDE_V3":
                self._ser.write((cmd_line.strip() + "\n").encode("ascii", errors="ignore"))
                self._ser.flush()
                return

            if verb in ("STOP", "S"):
                self._jog_active = False
                self._jog_m1_pwm = 0
                self._jog_m2_pwm = 0
                self._spin_tasks.clear()
                resp = self._send_raw_unlocked("STOP")
                self._out_queue.put(f"# {resp or 'OK STOP'}")
            elif verb in ("ZERO", "RESET", "Z"):
                self._jog_active = False
                self._spin_tasks.clear()
                self._send_raw_unlocked("STOP")
                side_char = "L" if self.side == "LEFT" else "R"
                resp = self._send_raw_unlocked(f"SIDE {side_char}")
                self._capture_baseline_unlocked()
                self._out_queue.put(f"# OK ZEROED ({resp or f'SIDE {side_char}'})")
            elif verb == "SIDE" and len(parts) >= 2:
                self._jog_active = False
                self._spin_tasks.clear()
                self.side = "LEFT" if parts[1].upper().startswith("L") else "RIGHT"
                side_char = "L" if self.side == "LEFT" else "R"
                self._send_raw_unlocked("STOP")
                resp = self._send_raw_unlocked(f"SIDE {side_char}")
                self._capture_baseline_unlocked()
                self._out_queue.put(f"# {resp or f'OK SIDE {side_char}'}")
            elif verb == "CPR" and len(parts) >= 2:
                self.cpr = max(1.0, float(parts[1]))
                self._out_queue.put(f"# OK CPR {self.cpr:.2f}")
            elif verb == "RADIUS" and len(parts) >= 2:
                self.radius_m = max(0.001, float(parts[1]))
                self._out_queue.put(f"# OK RADIUS {self.radius_m:.4f}")
            elif verb == "DEADBAND" and len(parts) >= 2:
                self.deadband_pwm = max(30, min(KENNY_V3_MAX_PWM, int(parts[1])))
                self._out_queue.put(f"# OK DEADBAND {self.deadband_pwm}")
            elif verb == "INVERT" and len(parts) >= 3:
                self.m1_sign = -1 if int(parts[1]) < 0 else 1
                self.m2_sign = -1 if int(parts[2]) < 0 else 1
                self._out_queue.put(f"# OK INVERT m1_sign={self.m1_sign} m2_sign={self.m2_sign}")
            elif verb == "SWAP" and len(parts) >= 2:
                self.swap_encoders = parts[1].upper() in ("1", "ON", "TRUE", "YES")
                self._prev_telem = None
                self._out_queue.put(f"# OK SWAP swap_encoders={int(self.swap_encoders)}")
            elif verb == "JOG" and len(parts) >= 3:
                self._spin_tasks.clear()
                target = parts[1].upper()
                pwm = max(-KENNY_V3_MAX_PWM, min(KENNY_V3_MAX_PWM, int(parts[2])))
                dur_ms = int(parts[3]) if len(parts) >= 4 else 500
                self._jog_m1_pwm = pwm if target in ("M1", "1", "BOTH", "ALL") else 0
                self._jog_m2_pwm = pwm if target in ("M2", "2", "BOTH", "ALL") else 0
                self._jog_active = (self._jog_m1_pwm != 0 or self._jog_m2_pwm != 0)
                self._jog_deadline = time.monotonic() + (dur_ms / 1000.0)
                self._arm_unlocked()
                self._send_pwm_unlocked(self._jog_m1_pwm, self._jog_m2_pwm)
                self._out_queue.put(f"# OK JOG target={target} pwm={pwm} duration_ms={dur_ms}")
            elif verb == "SPIN_REV" and len(parts) >= 3:
                self._jog_active = False
                target = parts[1].upper()
                revs = float(parts[2])
                max_pwm = max(60, min(KENNY_V3_MAX_PWM, int(parts[3]))) if len(parts) >= 4 else 135
                # Read current ticks before starting closed-loop 1-rev target
                status_line = self._send_raw_unlocked("STATUS") or ""
                snap = parse_kenny_side_v3_status_line(
                    status_line, side=self.side, prev_t=self._prev_telem,
                    counts_per_rev=self.cpr, wheel_radius_m=self.radius_m,
                    m1_offset=self._m1_offset, m2_offset=self._m2_offset,
                    m1_err_offset=self._m1_err_offset, m2_err_offset=self._m2_err_offset,
                    m1_sign=self.m1_sign, m2_sign=self.m2_sign,
                    swap_encoders=self.swap_encoders,
                )
                cur_m1 = snap.m1_ticks if snap else (self._prev_telem.m1_ticks if self._prev_telem else 0)
                cur_m2 = snap.m2_ticks if snap else (self._prev_telem.m2_ticks if self._prev_telem else 0)
                err_m1 = snap.m1_err if snap else 0
                err_m2 = snap.m2_err if snap else 0

                self._arm_unlocked()
                now = time.monotonic()
                motors = ["M1", "M2"] if target in ("BOTH", "ALL") else (["M1"] if target in ("M1", "1") else ["M2"])
                self._spin_tasks.clear()
                for m in motors:
                    self._spin_tasks[m] = {
                        "phase": 1.0,  # 1 = MOVING, 2 = SETTLING
                        "target_revs": revs,
                        "start_ticks": float(cur_m1 if m == "M1" else cur_m2),
                        "other_start_ticks": float(cur_m2 if m == "M1" else cur_m1),
                        "single_motor": 1.0 if len(motors) == 1 else 0.0,
                        "target_delta": float(int(round(revs * self.cpr))),
                        "brake_ticks": 0.0,
                        "max_pwm": float(max_pwm),
                        "start_s": now,
                        "settle_s": now,
                        "start_err": float(err_m1 if m == "M1" else err_m2),
                        "peak_rpm": 0.0,
                        "status_code": 0.0,  # 0=COMPLETED, 1=REVERSE, 2=STALL, 3=TIMEOUT, 4=SWAPPED
                    }
                self._out_queue.put(f"# OK SPIN_REV target={target} revs={revs:.4f} max_pwm={max_pwm}")

    def read_line(self) -> Optional[str]:
        if not self._out_queue.empty():
            return self._out_queue.get_nowait()

        if self.protocol != "KENNY_SIDE_V3":
            raw = self._ser.readline()
            return raw.decode("utf-8", errors="replace").strip() if raw else None

        # Poll KENNY_SIDE_V3 STATUS and run watchdog keepalive / closed-loop 1-rev control
        time.sleep(0.035)
        now = time.monotonic()
        with self._lock:
            if not self._out_queue.empty():
                return self._out_queue.get_nowait()

            status_raw = self._send_raw_unlocked("STATUS")
            if not status_raw:
                return None
            telem = parse_kenny_side_v3_status_line(
                status_raw,
                side=self.side,
                prev_t=self._prev_telem,
                counts_per_rev=self.cpr,
                wheel_radius_m=self.radius_m,
                m1_offset=self._m1_offset,
                m2_offset=self._m2_offset,
                m1_err_offset=self._m1_err_offset,
                m2_err_offset=self._m2_err_offset,
                m1_sign=self.m1_sign,
                m2_sign=self.m2_sign,
                swap_encoders=self.swap_encoders,
                host_time_s=now,
            )
            if telem is None:
                return None
            self._prev_telem = telem

            # Handle open-loop timed JOG watchdog refresh
            if self._jog_active:
                if now >= self._jog_deadline:
                    self._jog_active = False
                    self._jog_m1_pwm = 0
                    self._jog_m2_pwm = 0
                    self._send_raw_unlocked("STOP")
                    self._out_queue.put("# INFO JOG_FINISHED motors stopped")
                else:
                    self._send_pwm_unlocked(self._jog_m1_pwm, self._jog_m2_pwm)

            # Handle closed-loop 1-revolution spin tasks
            if self._spin_tasks:
                cmd_m1 = 0
                cmd_m2 = 0
                need_pwm_send = False
                done_motors: List[str] = []
                circ = 2.0 * math.pi * self.radius_m
                m1_rpm = abs(telem.m1_mps / max(circ, 1e-6)) * 60.0
                m2_rpm = abs(telem.m2_mps / max(circ, 1e-6)) * 60.0

                for m, task in list(self._spin_tasks.items()):
                    curr_t = telem.m1_ticks if m == "M1" else telem.m2_ticks
                    other_t = telem.m2_ticks if m == "M1" else telem.m1_ticks
                    curr_err = telem.m1_err if m == "M1" else telem.m2_err
                    curr_rpm = m1_rpm if m == "M1" else m2_rpm
                    task["peak_rpm"] = max(task["peak_rpm"], curr_rpm)

                    progressed = curr_t - int(task["start_ticks"])
                    other_progressed = other_t - int(task.get("other_start_ticks", other_t))
                    target_dt = int(task["target_delta"])
                    direction = 1 if target_dt >= 0 else -1

                    # If single motor is driven and the OTHER encoder is moving while this one is still,
                    # auto-correct the M1<->M2 encoder swap on the fly so the 1-rev spin succeeds!
                    if (
                        task["phase"] == 1.0
                        and task.get("single_motor", 0.0) > 0.5
                        and abs(other_progressed) > 35
                        and abs(progressed) < 10
                    ):
                        self.swap_encoders = not self.swap_encoders
                        task["start_ticks"] = float(int(task.get("other_start_ticks", other_t)))
                        task["other_start_ticks"] = float(curr_t)
                        progressed = other_progressed
                        curr_t, other_t = other_t, curr_t
                        self._out_queue.put(
                            f"# AUTO-SWAPPED M1<->M2 encoders (swap_encoders={int(self.swap_encoders)})"
                        )

                    # Use magnitude of encoder progress so inverted encoder polarity (+/-) never causes
                    # a false abort or runaway spin on either M1 or M2.
                    mag_prog = abs(progressed)
                    remaining = abs(target_dt) - mag_prog

                    if task["phase"] == 1.0:
                        if remaining <= 0 or target_dt == 0:
                            task["brake_ticks"] = float(direction * mag_prog)
                            task["settle_s"] = now
                            task["phase"] = 2.0
                            task["status_code"] = 0.0
                        elif (now - task["start_s"]) >= 8.0:
                            task["brake_ticks"] = float(direction * mag_prog)
                            task["settle_s"] = now
                            task["phase"] = 2.0
                            task["status_code"] = 2.0 if mag_prog < 10 else 3.0
                        else:
                            slow_zone = max(60.0, 0.35 * self.cpr)
                            max_pwm = int(task["max_pwm"])
                            if remaining < slow_zone:
                                ratio = remaining / slow_zone
                                mag = int(round(self.deadband_pwm + ratio * (max_pwm - self.deadband_pwm)))
                            else:
                                mag = max_pwm
                            mag = max(self.deadband_pwm, min(KENNY_V3_MAX_PWM, mag))
                            if m == "M1":
                                cmd_m1 = direction * mag
                            else:
                                cmd_m2 = direction * mag
                            need_pwm_send = True

                    if task["phase"] == 2.0:
                        if not need_pwm_send:
                            self._send_raw_unlocked("STOP")
                        if (now - task["settle_s"]) >= 0.35:
                            code_map = {
                                0.0: "COMPLETED",
                                1.0: "ABORT_REVERSE_ENCODER",
                                2.0: "ABORT_STALLED_NO_TICKS",
                                3.0: "ABORT_TIMEOUT",
                                4.0: "ABORT_SWAPPED_ENCODERS",
                            }
                            signed_actual = direction * mag_prog if mag_prog > 0 else progressed
                            res = evaluate_one_rev_accuracy(
                                motor=m,
                                target_revs=task["target_revs"],
                                actual_ticks=signed_actual,
                                counts_per_rev=self.cpr,
                                wheel_radius_m=self.radius_m,
                                brake_ticks=int(task["brake_ticks"]),
                                elapsed_ms=int((task["settle_s"] - task["start_s"]) * 1000.0),
                                peak_rpm=task["peak_rpm"],
                                glitches=max(0, curr_err - int(task["start_err"])),
                                status=code_map.get(task["status_code"], "COMPLETED"),
                            )
                            self._out_queue.put(
                                f"REV_RESULT motor={res.motor} status={res.status} "
                                f"target_revs={res.target_revs:.4f} actual_revs={res.actual_revs:.4f} "
                                f"target_ticks={res.target_ticks} actual_ticks={res.actual_ticks} "
                                f"error_ticks={res.error_ticks:+d} error_revs={res.error_revs:+.4f} "
                                f"error_deg={res.error_deg:+.2f} error_mm={res.error_mm:+.2f} "
                                f"accuracy_pct={res.accuracy_pct:.2f} brake_ticks={res.brake_ticks} "
                                f"coast_ticks={res.coast_ticks:+d} elapsed_ms={res.elapsed_ms} "
                                f"peak_rpm={res.peak_rpm:.1f} glitches={res.glitches}"
                            )
                            done_motors.append(m)

                if need_pwm_send:
                    self._send_pwm_unlocked(cmd_m1, cmd_m2)
                for dm in done_motors:
                    self._spin_tasks.pop(dm, None)

            return (
                f"ODOM side={telem.side} ms={telem.ms} armed={telem.armed} seq={telem.seq} "
                f"m1_ticks={telem.m1_ticks} m2_ticks={telem.m2_ticks} "
                f"m1_dt={telem.m1_dt} m2_dt={telem.m2_dt} "
                f"m1_revs={telem.m1_revs:.4f} m2_revs={telem.m2_revs:.4f} "
                f"m1_m={telem.m1_m:.4f} m2_m={telem.m2_m:.4f} "
                f"m1_mps={telem.m1_mps:.4f} m2_mps={telem.m2_mps:.4f} "
                f"m1_pwm={telem.m1_pwm} m2_pwm={telem.m2_pwm} "
                f"m1_ab={telem.m1_ab} m2_ab={telem.m2_ab} "
                f"m1_err={telem.m1_err} m2_err={telem.m2_err} "
                f"m1_a_edges={telem.m1_a_edges} m1_b_edges={telem.m1_b_edges} "
                f"m2_a_edges={telem.m2_a_edges} m2_b_edges={telem.m2_b_edges} "
                f"fw={telem.firmware}"
            )

    def close(self) -> None:
        try:
            with self._lock:
                self._jog_active = False
                self._spin_tasks.clear()
                self._send_raw_unlocked("STOP")
        except Exception:
            pass
        self._ser.close()


def format_status_banner(
    boards: Dict[str, WheelTelemetry],
    odom: DifferentialDriveOdometry,
) -> str:
    """Format a compact multi-line status summary for terminal display."""
    parts: List[str] = []
    for label in sorted(boards.keys()):
        t = boards[label]
        m1_rpm = (t.m1_mps / max(odom.circumference_m, 1e-6)) * 60.0
        m2_rpm = (t.m2_mps / max(odom.circumference_m, 1e-6)) * 60.0
        parts.append(
            f"[{label}:{t.side}] "
            f"M1: {t.m1_ticks:+7d} ticks ({t.m1_revs:+6.3f} rev, {t.m1_m:+6.3f} m, {m1_rpm:+5.1f} RPM, A/B={t.m1_a_edges}/{t.m1_b_edges}, err={t.m1_err}) | "
            f"M2: {t.m2_ticks:+7d} ticks ({t.m2_revs:+6.3f} rev, {t.m2_m:+6.3f} m, {m2_rpm:+5.1f} RPM, A/B={t.m2_a_edges}/{t.m2_b_edges}, err={t.m2_err})"
        )
    p = odom.pose
    parts.append(
        f"[ODOM 2D] x={p.x_m:+.3f} m, y={p.y_m:+.3f} m, yaw={p.theta_deg:+6.1f} deg | "
        f"path={p.total_distance_m:.3f} m | v={p.linear_mps:+.2f} m/s, w={p.angular_rps:+.2f} rad/s"
    )
    return "\n".join(parts)


def print_interactive_help() -> None:
    print(
        "\n--- Manual Wheel Odometer & 1-Rev Accuracy Commands ---\n"
        "  gui                              : Launch the graphical desktop dashboard\n"
        "  rev <m1|m2|both> [revs] [pwm]    : Closed-loop spin by exact <revs> (default 1.000 rev) & measure accuracy\n"
        "  z  (or zero)                     : Zero encoder counts and reset 2D pose (x, y, yaw)\n"
        "  cal [turns]                      : Start guided manual wheel-turn CPR calibration (default 1.0 turn)\n"
        "  done                             : Finish active manual CPR calibration and compute CPR\n"
        "  jog <m1|m2|both> <pwm> [ms]      : Jog motor(s) at PWM (-191..191) for ms (default 500 ms)\n"
        "  stop (or s)                      : Immediately stop all motors (PWM=0)\n"
        "  side <left|right>                : Switch ESP32 encoder pin table\n"
        "  cpr <counts_per_rev>             : Set quadrature counts per revolution (e.g., cpr 1320)\n"
        "  radius <wheel_radius_m>          : Set wheel radius in meters (default 0.04)\n"
        "  invert <m1_sign> <m2_sign>       : Set encoder direction signs (+1 or -1)\n"
        "  spin <m1|m2|both> <turns>        : (Simulation mode only) Simulate hand-spinning wheel(s) by <turns>\n"
        "  save [path.json]                 : Save odometer & 1-rev accuracy report JSON\n"
        "  p                                : Toggle live continuous telemetry printing on/off\n"
        "  q  (or quit)                     : Stop motors and exit\n"
    )


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gui", action="store_true", help="Launch the graphical desktop interface")
    parser.add_argument("--port", help="Serial port for a single ESP32 board (auto-detected if omitted)")
    parser.add_argument("--side", choices=["left", "right"], default="left",
                        help="Which ESP32 side pin table to use when --port is specified (default: left)")
    parser.add_argument("--left-port", help="Serial port for the LEFT ESP32 board (dual-board mode)")
    parser.add_argument("--right-port", help="Serial port for the RIGHT ESP32 board (dual-board mode)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate (default: 115200)")
    parser.add_argument("--cpr", type=float, default=DEFAULT_COUNTS_PER_REV,
                        help=f"Quadrature counts per wheel revolution (default: {DEFAULT_COUNTS_PER_REV})")
    parser.add_argument("--wheel-radius", type=float, default=DEFAULT_WHEEL_RADIUS_M,
                        help=f"Wheel radius in meters (default: {DEFAULT_WHEEL_RADIUS_M} for 80mm wheel)")
    parser.add_argument("--track-width", type=float, default=DEFAULT_TRACK_WIDTH_M,
                        help=f"Wheel track width in meters (default: {DEFAULT_TRACK_WIDTH_M})")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="Optional duration in seconds for non-interactive timed test (0 = interactive)")
    parser.add_argument("--simulate", action="store_true",
                        help="Run in offline simulation mode without a physical ESP32 connected")
    parser.add_argument("--list-ports", action="store_true",
                        help="Auto-probe and list detected USB serial ports and exit")
    parser.add_argument("--flash", action="store_true",
                        help="Compile and upload firmware to --port without Arduino IDE (via PlatformIO or arduino-cli)")
    parser.add_argument("--allow-lidar-port", action="store_true",
                        help="Override safety guard that prevents opening the YDLIDAR CP2102 port")
    parser.add_argument("--output", type=Path, default=Path("artifacts/pi5/wheel-odometer-test.json"),
                        help="Path to write JSON summary report")
    args = parser.parse_args(argv)

    if args.gui:
        from scripts.wheel_odometer_gui import launch_gui
        return launch_gui(
            port=args.port,
            side=args.side,
            cpr=args.cpr,
            wheel_radius_m=args.wheel_radius,
            track_width_m=args.track_width,
            simulate=args.simulate,
            allow_lidar_port=args.allow_lidar_port,
        )

    if args.list_ports:
        ports = discover_serial_ports(allow_lidar_port=args.allow_lidar_port, probe_esp32=True)
        if not ports:
            print("No USB serial ports detected.")
        else:
            for p in ports:
                print(
                    f"{p['device']:16s} [{p['status']:14s}] {p['description']} "
                    f"| sig={p.get('signature', '')}"
                )
        return 0

    if args.flash:
        port = args.port
        if not port:
            detected = auto_detect_esp32_port(allow_lidar_port=args.allow_lidar_port)
            if detected:
                port = detected[0]
        if not port:
            parser.error("--flash requires an ESP32 port (none auto-detected)")
        ok, msg = flash_esp32_without_arduino_ide(port, allow_lidar_port=args.allow_lidar_port)
        print(msg)
        return 0 if ok else 1

    boards: Dict[str, object] = {}
    if args.simulate:
        if args.left_port or args.right_port:
            boards["LEFT"] = SimulatedESP32Board(side="LEFT", cpr=args.cpr, radius_m=args.wheel_radius)
            boards["RIGHT"] = SimulatedESP32Board(side="RIGHT", cpr=args.cpr, radius_m=args.wheel_radius)
        else:
            label = args.side.upper()
            boards[label] = SimulatedESP32Board(side=label, cpr=args.cpr, radius_m=args.wheel_radius)
    elif args.left_port or args.right_port:
        if args.left_port:
            boards["LEFT"] = SerialESP32Board(
                args.left_port, baud=args.baud, side="LEFT", cpr=args.cpr,
                radius_m=args.wheel_radius, allow_lidar_port=args.allow_lidar_port
            )
        if args.right_port:
            boards["RIGHT"] = SerialESP32Board(
                args.right_port, baud=args.baud, side="RIGHT", cpr=args.cpr,
                radius_m=args.wheel_radius, allow_lidar_port=args.allow_lidar_port
            )
    elif args.port:
        label = args.side.upper()
        boards[label] = SerialESP32Board(
            args.port, baud=args.baud, side=label, cpr=args.cpr,
            radius_m=args.wheel_radius, allow_lidar_port=args.allow_lidar_port
        )
    else:
        detected = auto_detect_esp32_port(allow_lidar_port=args.allow_lidar_port)
        if detected:
            port, sig = detected
            label = args.side.upper()
            print(f"Auto-detected ESP32 on {port}: {sig}")
            boards[label] = SerialESP32Board(
                port, baud=args.baud, side=label, cpr=args.cpr,
                radius_m=args.wheel_radius, allow_lidar_port=args.allow_lidar_port
            )
        else:
            parser.error(
                "No ESP32 USB serial port detected. Plug in the ESP32 or run with --simulate."
            )

    odom = DifferentialDriveOdometry(
        wheel_radius_m=args.wheel_radius,
        track_width_m=args.track_width,
        counts_per_rev=args.cpr,
    )
    latest_telemetry: Dict[str, WheelTelemetry] = {}
    calibrated_cpr: Dict[str, Dict[str, float]] = {}
    rev_results: List[Dict[str, object]] = []
    calibration_state: Optional[Dict[str, object]] = None
    live_print = True
    stop_event = threading.Event()
    line_queue: "queue.Queue[Tuple[str, str]]" = queue.Queue()

    def reader_worker(board_label: str, board_obj: object) -> None:
        while not stop_event.is_set():
            try:
                line = board_obj.read_line()  # type: ignore[attr-defined]
                if line:
                    line_queue.put((board_label, line))
            except Exception as exc:
                line_queue.put((board_label, f"# ERR Serial read error: {exc}"))
                break

    for label, b_obj in boards.items():
        b_obj.write_command(f"SIDE {label}")  # type: ignore[attr-defined]
        b_obj.write_command(f"CPR {args.cpr}")  # type: ignore[attr-defined]
        b_obj.write_command(f"RADIUS {args.wheel_radius}")  # type: ignore[attr-defined]
        t = threading.Thread(target=reader_worker, args=(label, b_obj), daemon=True)
        t.start()

    cmd_queue: "queue.Queue[str]" = queue.Queue()
    interactive = args.seconds <= 0 and sys.stdin.isatty()
    if interactive:
        print_interactive_help()

        def stdin_worker() -> None:
            while not stop_event.is_set():
                try:
                    user_line = sys.stdin.readline()
                    if not user_line:
                        cmd_queue.put("quit")
                        break
                    cmd_queue.put(user_line.strip())
                except Exception:
                    break

        threading.Thread(target=stdin_worker, daemon=True).start()

    def broadcast(command_str: str) -> None:
        for b_obj in boards.values():
            b_obj.write_command(command_str)  # type: ignore[attr-defined]

    def save_report(target_path: Path) -> None:
        target_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "wheel_radius_m": odom.wheel_radius_m,
            "track_width_m": odom.track_width_m,
            "counts_per_rev": odom.counts_per_rev,
            "pose": {
                **asdict(odom.pose),
                "theta_deg": odom.pose.theta_deg,
            },
            "boards": {k: asdict(v) for k, v in latest_telemetry.items()},
            "calibrated_cpr": calibrated_cpr,
            "rev_spin_results": rev_results,
        }
        target_path.write_text(json.dumps(payload, indent=2) + "\n")
        print(f"\n[SAVED] Odometer test report written to {target_path}")

    last_display_s = 0.0
    deadline_s = (time.monotonic() + args.seconds) if args.seconds > 0 else math.inf

    try:
        while time.monotonic() < deadline_s and not stop_event.is_set():
            while not line_queue.empty():
                label, line = line_queue.get_nowait()
                if line.startswith("#"):
                    print(f"\n[{label}] {line}")
                    continue
                if line.startswith("REV_RESULT "):
                    res = parse_rev_result_line(line, counts_per_rev=odom.counts_per_rev, wheel_radius_m=odom.wheel_radius_m)
                    if res:
                        rev_results.append(asdict(res))
                        print(
                            f"\n=== [{label} {res.motor}] 1-Rev Spin Accuracy Report ({res.grade}) ===\n"
                            f"  Target : {res.target_revs:.4f} rev ({res.target_ticks} ticks)\n"
                            f"  Actual : {res.actual_revs:.4f} rev ({res.actual_ticks} ticks)\n"
                            f"  Error  : {res.error_ticks:+d} ticks | {res.error_revs:+.4f} rev | {res.error_deg:+.2f} deg | {res.error_mm:+.2f} mm\n"
                            f"  Score  : {res.accuracy_pct:.2f}% accuracy | Coast after brake: {res.coast_ticks:+d} ticks | Peak: {res.peak_rpm:.1f} RPM | Time: {res.elapsed_ms} ms\n"
                            f"  Note   : {res.diagnosis}"
                        )
                    continue

                parsed = parse_odom_line(line)
                if parsed is None:
                    continue
                latest_telemetry[label] = parsed

                if "LEFT" in latest_telemetry and "RIGHT" in latest_telemetry:
                    lt = latest_telemetry["LEFT"]
                    rt = latest_telemetry["RIGHT"]
                    left_avg_ticks = 0.5 * (lt.m1_ticks + lt.m2_ticks)
                    right_avg_ticks = 0.5 * (rt.m1_ticks + rt.m2_ticks)
                    odom.update_from_ticks(left_avg_ticks, right_avg_ticks, timestamp_s=parsed.host_timestamp_s)
                else:
                    odom.update_from_ticks(parsed.m1_ticks, parsed.m2_ticks, timestamp_s=parsed.host_timestamp_s)

            now_s = time.monotonic()
            if live_print and latest_telemetry and (now_s - last_display_s) >= 0.25:
                last_display_s = now_s
                banner = format_status_banner(latest_telemetry, odom)
                if interactive:
                    print(f"\r{banner}\n" + "-" * 100)
                else:
                    print(banner)

            while not cmd_queue.empty():
                raw_cmd = cmd_queue.get_nowait()
                if not raw_cmd:
                    continue
                tokens = raw_cmd.split()
                verb = tokens[0].lower()

                if verb in ("q", "quit", "exit"):
                    broadcast("STOP")
                    stop_event.set()
                    break
                elif verb in ("?", "help", "h"):
                    print_interactive_help()
                elif verb == "p":
                    live_print = not live_print
                    print(f"\n[INFO] Live telemetry printing: {'ON' if live_print else 'OFF'}")
                elif verb in ("z", "zero", "reset"):
                    broadcast("ZERO")
                    odom.reset()
                elif verb in ("s", "stop"):
                    broadcast("STOP")
                elif verb == "side" and len(tokens) >= 2:
                    side_val = tokens[1].upper()
                    broadcast(f"SIDE {side_val}")
                    odom.reset()
                elif verb == "cpr" and len(tokens) >= 2:
                    new_cpr = float(tokens[1])
                    odom.counts_per_rev = new_cpr
                    broadcast(f"CPR {new_cpr}")
                elif verb == "radius" and len(tokens) >= 2:
                    new_r = float(tokens[1])
                    odom.wheel_radius_m = new_r
                    broadcast(f"RADIUS {new_r}")
                elif verb == "invert" and len(tokens) >= 3:
                    broadcast(f"INVERT {tokens[1]} {tokens[2]}")
                elif verb == "jog" and len(tokens) >= 3:
                    target = tokens[1]
                    pwm = tokens[2]
                    dur = tokens[3] if len(tokens) >= 4 else "500"
                    broadcast(f"JOG {target} {pwm} {dur}")
                elif verb == "rev" and len(tokens) >= 2:
                    target = tokens[1]
                    revs = tokens[2] if len(tokens) >= 3 else "1.0"
                    max_pwm = tokens[3] if len(tokens) >= 4 else "135"
                    broadcast(f"SPIN_REV {target} {revs} {max_pwm}")
                elif verb == "spin" and len(tokens) >= 3:
                    broadcast(f"SPIN {tokens[1]} {tokens[2]}")
                elif verb in ("cal", "calibrate"):
                    turns = float(tokens[1]) if len(tokens) >= 2 else 1.0
                    broadcast("STOP")
                    broadcast("ZERO")
                    time.sleep(0.1)
                    calibration_state = {
                        "turns": turns,
                        "start": {k: (v.m1_ticks, v.m2_ticks) for k, v in latest_telemetry.items()},
                    }
                    print(
                        f"\n[CALIBRATION STARTED] Counters zeroed.\n"
                        f"  1. Spin each wheel forward by hand {turns:.2f} full revolution(s).\n"
                        f"  2. Type 'done' and press Enter to compute exact counts_per_rev (CPR)."
                    )
                elif verb == "done":
                    if not calibration_state:
                        print("\n[WARN] No active calibration. Type 'cal 1' first.")
                    else:
                        turns = float(calibration_state["turns"])  # type: ignore[arg-type]
                        print(f"\n=== Manual {turns:.2f}-Turn Calibration Results ===")
                        for label, t in latest_telemetry.items():
                            m1_cpr, m1_sgn = estimate_cpr_from_manual_turns(t.m1_ticks, turns)
                            m2_cpr, m2_sgn = estimate_cpr_from_manual_turns(t.m2_ticks, turns)
                            calibrated_cpr[label] = {
                                "turns": turns,
                                "m1_ticks": t.m1_ticks,
                                "m2_ticks": t.m2_ticks,
                                "m1_cpr": round(m1_cpr, 2),
                                "m2_cpr": round(m2_cpr, 2),
                                "m1_sign": m1_sgn,
                                "m2_sign": m2_sgn,
                            }
                            print(
                                f"  Board {label} ({t.side}):\n"
                                f"    Motor 1: {t.m1_ticks:+d} ticks -> CPR = {m1_cpr:.2f} (sign={m1_sgn:+d})\n"
                                f"    Motor 2: {t.m2_ticks:+d} ticks -> CPR = {m2_cpr:.2f} (sign={m2_sgn:+d})"
                            )
                        calibration_state = None
                elif verb == "save":
                    out_path = Path(tokens[1]) if len(tokens) >= 2 else args.output
                    save_report(out_path)
                else:
                    print(f"\n[WARN] Unknown command: '{raw_cmd}'. Type 'help' for available commands.")

            time.sleep(0.02)

        save_report(args.output)
        return 0
    finally:
        stop_event.set()
        for b_obj in boards.values():
            b_obj.close()  # type: ignore[attr-defined]


if __name__ == "__main__":
    raise SystemExit(main())
