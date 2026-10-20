"""Unit and integration tests for the wheel odometer, diagnostics, and 1-revolution accuracy suite."""
from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from scripts.test_wheel_odometer import (
    DEFAULT_COUNTS_PER_REV,
    DEFAULT_TRACK_WIDTH_M,
    DEFAULT_WHEEL_RADIUS_M,
    DifferentialDriveOdometry,
    RevSpinResult,
    SimulatedESP32Board,
    WheelTelemetry,
    compute_wheel_metrics,
    diagnose_wheel_health,
    estimate_cpr_from_manual_turns,
    evaluate_one_rev_accuracy,
    is_lidar_serial_port,
    main,
    parse_odom_line,
    parse_rev_result_line,
)


def test_hand_spin_auto_calibration_applies_only_matching_clean_wheels():
    from scripts.wheel_odometer_gui import WheelOdometerGUI

    gui = WheelOdometerGUI.__new__(WheelOdometerGUI)
    gui.board = object()
    gui.latest_telemetry = WheelTelemetry(side="RIGHT")
    gui.auto_cal_status = Mock()
    gui.apply_cpr_btn = Mock()
    gui.cpr_var = Mock()
    gui.odom = SimpleNamespace(counts_per_rev=1320.0)
    gui.send_cmd = Mock()

    with patch("scripts.wheel_odometer_gui.time.monotonic", return_value=100.0):
        gui.start_auto_calibration()
    assert gui.auto_cal_active
    assert gui.send_cmd.call_args_list[0].args == ("STOP",)

    with patch("scripts.wheel_odometer_gui.time.monotonic", return_value=101.0):
        gui._update_auto_calibration(WheelTelemetry(side="RIGHT", m1_ticks=1318, m1_a_edges=659, m1_b_edges=659))
    with patch("scripts.wheel_odometer_gui.time.monotonic", return_value=103.0):
        gui._update_auto_calibration(WheelTelemetry(side="RIGHT", m1_ticks=1318, m1_a_edges=659, m1_b_edges=659))
    assert gui.auto_cal_results == {"M1": 1318.0}
    assert gui.odom.counts_per_rev == 1320.0

    with patch("scripts.wheel_odometer_gui.time.monotonic", return_value=104.0):
        gui._update_auto_calibration(WheelTelemetry(side="RIGHT", m1_ticks=1318, m1_a_edges=659, m1_b_edges=659,
                                                    m2_ticks=1322, m2_a_edges=661, m2_b_edges=661))
    with patch("scripts.wheel_odometer_gui.time.monotonic", return_value=106.0):
        gui._update_auto_calibration(WheelTelemetry(side="RIGHT", m1_ticks=1318, m1_a_edges=659, m1_b_edges=659,
                                                    m2_ticks=1322, m2_a_edges=661, m2_b_edges=661))
    assert not gui.auto_cal_active
    assert gui.odom.counts_per_rev == 1320.0
    gui.send_cmd.assert_any_call("CPR 1320.00")


def test_hand_spin_auto_calibration_rejects_glitches():
    from scripts.wheel_odometer_gui import WheelOdometerGUI

    gui = WheelOdometerGUI.__new__(WheelOdometerGUI)
    gui.board = object()
    gui.latest_telemetry = WheelTelemetry(side="RIGHT")
    gui.auto_cal_status = Mock()
    gui.apply_cpr_btn = Mock()
    gui.cpr_var = Mock()
    gui.odom = SimpleNamespace(counts_per_rev=1320.0)
    gui.send_cmd = Mock()
    with patch("scripts.wheel_odometer_gui.time.monotonic", return_value=100.0):
        gui.start_auto_calibration()
    with patch("scripts.wheel_odometer_gui.time.monotonic", return_value=101.0):
        gui._update_auto_calibration(WheelTelemetry(side="RIGHT", m1_ticks=1300, m1_a_edges=650, m1_b_edges=650, m1_err=40))
    with patch("scripts.wheel_odometer_gui.time.monotonic", return_value=103.0):
        gui._update_auto_calibration(WheelTelemetry(side="RIGHT", m1_ticks=1300, m1_a_edges=650, m1_b_edges=650, m1_err=40))
    assert not gui.auto_cal_active
    assert gui.odom.counts_per_rev == 1320.0
    assert all(not call.args[0].startswith("CPR ") for call in gui.send_cmd.call_args_list)


def test_powered_pwm_tune_selects_smallest_one_turn_error():
    from scripts.wheel_odometer_gui import WheelOdometerGUI

    gui = WheelOdometerGUI.__new__(WheelOdometerGUI)
    gui.board = object()
    gui.sim_var = Mock()
    gui.sim_var.get.return_value = False
    gui.pwm_tune_motor = None
    gui.pwm_tune_current_pwm = None
    gui.pwm_tune_generation = 0
    gui.pwm_tune_start_edges = (0, 0)
    gui.pwm_tune_status = Mock()
    gui.cpr_var = Mock()
    gui.max_pwm_var = Mock()
    gui.odom = SimpleNamespace(counts_per_rev=1320.0)
    gui.latest_telemetry = WheelTelemetry(side="RIGHT")
    gui.root = Mock()
    gui.send_cmd = Mock()
    gui.auto_cal_active = False
    gui.start_pwm_tune("M2")
    assert gui.odom.counts_per_rev == 660.0
    gui.send_cmd.assert_any_call("SPIN_REV M2 1.0000 90")

    for pwm, error in ((90, 30), (120, 12), (150, 4), (180, 22)):
        assert gui.pwm_tune_current_pwm == pwm
        gui.latest_telemetry.m2_a_edges += 330
        gui.latest_telemetry.m2_b_edges += 330
        gui._handle_pwm_tune_result(RevSpinResult(motor="M2", status="COMPLETED",
                                                   actual_ticks=660 + error, error_ticks=error, glitches=0))
        gui._run_next_pwm_tune_trial(gui.pwm_tune_generation)
    gui.max_pwm_var.set.assert_called_with("150")
    assert gui.pwm_tune_motor is None


def test_powered_pwm_tune_stops_on_missing_encoder_feedback():
    from scripts.wheel_odometer_gui import WheelOdometerGUI

    gui = WheelOdometerGUI.__new__(WheelOdometerGUI)
    gui.board = object()
    gui.sim_var = Mock()
    gui.sim_var.get.return_value = False
    gui.pwm_tune_motor = None
    gui.pwm_tune_current_pwm = None
    gui.pwm_tune_generation = 0
    gui.pwm_tune_start_edges = (0, 0)
    gui.pwm_tune_status = Mock()
    gui.cpr_var = Mock()
    gui.max_pwm_var = Mock()
    gui.odom = SimpleNamespace(counts_per_rev=1320.0)
    gui.latest_telemetry = WheelTelemetry(side="RIGHT")
    gui.root = Mock()
    gui.send_cmd = Mock()
    gui.auto_cal_active = False
    gui.start_pwm_tune("M1")
    gui._handle_pwm_tune_result(RevSpinResult(motor="M1", status="ABORT_STALLED_NO_TICKS",
                                               actual_ticks=0, error_ticks=-660, glitches=0))
    assert gui.pwm_tune_motor is None
    gui.send_cmd.assert_any_call("STOP")
    gui.max_pwm_var.set.assert_not_called()


def test_powered_pwm_tune_rejects_unbalanced_encoder_edges():
    from scripts.wheel_odometer_gui import WheelOdometerGUI

    gui = WheelOdometerGUI.__new__(WheelOdometerGUI)
    gui.pwm_tune_motor = "M1"
    gui.pwm_tune_current_pwm = 90
    gui.pwm_tune_generation = 1
    gui.pwm_tune_start_edges = (0, 0)
    gui.pwm_tune_status = Mock()
    gui.latest_telemetry = WheelTelemetry(side="RIGHT", m1_a_edges=900, m1_b_edges=30)
    gui.send_cmd = Mock()
    gui._handle_pwm_tune_result(RevSpinResult(motor="M1", status="COMPLETED",
                                               actual_ticks=660, error_ticks=0, glitches=0))
    assert gui.pwm_tune_motor is None
    gui.send_cmd.assert_called_with("STOP")


def test_parse_odom_line_valid_and_invalid():
    line = (
        "ODOM side=LEFT ms=2500 m1_ticks=1320 m2_ticks=-660 m1_dt=66 m2_dt=-33 "
        "m1_revs=1.0000 m2_revs=-0.5000 m1_m=0.2513 m2_m=-0.1257 m1_mps=0.2513 m2_mps=-0.1257 "
        "m1_pwm=120 m2_pwm=0 m1_ab=10 m2_ab=01 m1_err=0 m2_err=2 "
        "m1_a_edges=660 m1_b_edges=660 m2_a_edges=330 m2_b_edges=330"
    )
    parsed = parse_odom_line(line, host_time_s=10.5)
    assert parsed is not None
    assert parsed.side == "LEFT"
    assert parsed.ms == 2500
    assert parsed.m1_ticks == 1320
    assert parsed.m2_ticks == -660
    assert parsed.m1_dt == 66
    assert parsed.m2_dt == -33
    assert math.isclose(parsed.m1_revs, 1.0)
    assert math.isclose(parsed.m2_revs, -0.5)
    assert math.isclose(parsed.m1_m, 0.2513)
    assert parsed.m1_pwm == 120
    assert parsed.m1_ab == "10"
    assert parsed.m2_ab == "01"
    assert parsed.m2_err == 2
    assert parsed.m1_a_edges == 660
    assert parsed.m1_b_edges == 660
    assert parsed.host_timestamp_s == 10.5

    assert parse_odom_line("# OK ZEROED") is None
    assert parse_odom_line("ODOM malformed_without_ticks=1") is None


def test_compute_wheel_metrics_one_full_revolution():
    cpr = 1320.0
    radius_m = 0.04
    expected_circumference = 2.0 * math.pi * radius_m  # ~0.2513274 m

    metrics = compute_wheel_metrics(
        ticks=1320,
        dt_ticks=132,
        dt_s=0.1,
        counts_per_rev=cpr,
        wheel_radius_m=radius_m,
        pwm=0,
        ab_state="11",
        glitches=0,
        a_edges=660,
        b_edges=660,
    )
    assert math.isclose(metrics.revolutions, 1.0)
    assert math.isclose(metrics.angle_deg, 360.0)
    assert math.isclose(metrics.wrapped_deg, 0.0)
    assert math.isclose(metrics.distance_m, expected_circumference, rel_tol=1e-6)
    assert math.isclose(metrics.velocity_mps, expected_circumference, rel_tol=1e-6)
    assert math.isclose(metrics.rpm, 60.0, rel_tol=1e-6)
    assert math.isclose(metrics.ab_balance_pct, 100.0)


def test_one_revolution_accuracy_evaluation_and_parser():
    # Overshoot by +6 ticks out of 1320 (1.004545 rev -> +1.636 deg, 99.55% accuracy)
    res = evaluate_one_rev_accuracy(
        motor="M1",
        target_revs=1.0,
        actual_ticks=1326,
        counts_per_rev=1320.0,
        wheel_radius_m=0.04,
        brake_ticks=1320,
        elapsed_ms=920,
        peak_rpm=58.0,
        glitches=0,
        status="COMPLETED",
    )
    assert res.target_ticks == 1320
    assert res.actual_ticks == 1326
    assert res.error_ticks == 6
    assert res.coast_ticks == 6
    assert math.isclose(res.error_deg, (6.0 / 1320.0) * 360.0, rel_tol=1e-4)
    assert res.accuracy_pct >= 99.5
    assert res.grade == "EXCELLENT"

    line = (
        "REV_RESULT motor=M2 status=COMPLETED target_revs=1.0000 actual_revs=0.9848 "
        "target_ticks=1320 actual_ticks=1300 error_ticks=-20 error_revs=-0.0152 "
        "error_deg=-5.45 error_mm=-3.81 accuracy_pct=98.48 brake_ticks=1300 "
        "coast_ticks=+0 elapsed_ms=1050 peak_rpm=51.2 glitches=1"
    )
    parsed = parse_rev_result_line(line, counts_per_rev=1320.0, wheel_radius_m=0.04)
    assert parsed is not None
    assert parsed.motor == "M2"
    assert parsed.actual_ticks == 1300
    assert parsed.error_ticks == -20
    assert parsed.grade == "GOOD"
    assert "Undershot" in parsed.diagnosis
    assert "glitches" in parsed.diagnosis


def test_troubleshooting_diagnostics_detects_faults():
    # 1. Healthy telemetry
    healthy = WheelTelemetry(
        side="LEFT",
        m1_ticks=1320,
        m2_ticks=1320,
        m1_a_edges=660,
        m1_b_edges=660,
        m2_a_edges=660,
        m2_b_edges=660,
    )
    findings_ok = diagnose_wheel_health(healthy)
    assert len(findings_ok) == 1
    assert findings_ok[0].code == "HEALTHY"

    # 2. Dead Channel A on M2 (GPIO34 missing pull-up), glitches on M1, and inverted M1 direction
    faulty = WheelTelemetry(
        side="LEFT",
        m1_ticks=-100,
        m1_dt=-25,
        m1_pwm=140,
        m1_err=5,
        m1_a_edges=50,
        m1_b_edges=50,
        m2_a_edges=0,
        m2_b_edges=45,
    )
    findings = diagnose_wheel_health(faulty)
    codes = {f.code for f in findings}
    assert "DEAD_CHANNEL_A" in codes
    assert "QUADRATURE_GLITCHES" in codes
    assert "INVERTED_DIRECTION" in codes


def test_estimate_cpr_from_manual_turns():
    cpr, sign = estimate_cpr_from_manual_turns(delta_ticks=2640, turns=2.0)
    assert math.isclose(cpr, 1320.0)
    assert sign == 1

    cpr_rev, sign_rev = estimate_cpr_from_manual_turns(delta_ticks=-1320, turns=1.0)
    assert math.isclose(cpr_rev, 1320.0)
    assert sign_rev == -1


def test_differential_drive_odometry_straight_and_in_place_turn():
    odom = DifferentialDriveOdometry(
        wheel_radius_m=DEFAULT_WHEEL_RADIUS_M,
        track_width_m=DEFAULT_TRACK_WIDTH_M,
        counts_per_rev=DEFAULT_COUNTS_PER_REV,
    )
    odom.update_from_ticks(0, 0, timestamp_s=0.0)

    # 1 full forward wheel revolution on both sides
    pose = odom.update_from_ticks(1320, 1320, timestamp_s=1.0)
    expected_dist = 2.0 * math.pi * DEFAULT_WHEEL_RADIUS_M
    assert math.isclose(pose.x_m, expected_dist, abs_tol=1e-6)
    assert math.isclose(pose.y_m, 0.0, abs_tol=1e-6)
    assert math.isclose(pose.theta_rad, 0.0, abs_tol=1e-6)
    assert math.isclose(pose.linear_mps, expected_dist, abs_tol=1e-6)

    # Reset and perform an in-place 90-degree (pi/2) counter-clockwise turn
    odom.reset()
    odom.update_from_wheel_distances(0.0, 0.0, timestamp_s=0.0)
    arc = 0.5 * DEFAULT_TRACK_WIDTH_M * (math.pi / 2.0)
    pose = odom.update_from_wheel_distances(-arc, +arc, timestamp_s=1.0)
    assert math.isclose(pose.x_m, 0.0, abs_tol=1e-6)
    assert math.isclose(pose.y_m, 0.0, abs_tol=1e-6)
    assert math.isclose(pose.theta_deg, 90.0, abs_tol=1e-4)


def test_lidar_serial_port_guard():
    lidar_path = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
    assert is_lidar_serial_port(lidar_path) is True
    assert is_lidar_serial_port("/dev/ttyACM0", description="USB JTAG/serial debug unit") is False


def test_simulated_board_spin_rev_and_cli_execution():
    sim = SimulatedESP32Board(side="LEFT", cpr=1320.0, radius_m=0.04)
    sim.write_command("SPIN_REV M1 1.0 135")
    rev_result = None
    for _ in range(60):
        line = sim.read_line()
        if line and line.startswith("REV_RESULT "):
            rev_result = parse_rev_result_line(line, counts_per_rev=1320.0, wheel_radius_m=0.04)
            break
    assert rev_result is not None
    assert rev_result.motor == "M1"
    assert rev_result.target_ticks == 1320
    assert rev_result.accuracy_pct >= 99.0

    with tempfile.TemporaryDirectory() as tmp_dir:
        report_path = Path(tmp_dir) / "odom_report.json"
        rc = main(["--simulate", "--seconds", "0.2", "--output", str(report_path)])
        assert rc == 0
        assert report_path.exists()
        data = json.loads(report_path.read_text())
        assert "LEFT" in data["boards"]
        assert math.isclose(data["wheel_radius_m"], 0.04)


class WheelOdometerTestSuite(unittest.TestCase):
    def test_all(self):
        test_parse_odom_line_valid_and_invalid()
        test_compute_wheel_metrics_one_full_revolution()
        test_one_revolution_accuracy_evaluation_and_parser()
        test_troubleshooting_diagnostics_detects_faults()
        test_estimate_cpr_from_manual_turns()
        test_differential_drive_odometry_straight_and_in_place_turn()
        test_lidar_serial_port_guard()
        test_simulated_board_spin_rev_and_cli_execution()


if __name__ == "__main__":
    unittest.main()
