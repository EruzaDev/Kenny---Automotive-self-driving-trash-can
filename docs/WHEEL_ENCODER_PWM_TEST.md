# ESP32 wheel encoder and powered one-revolution test

Use `python3 scripts/wheel_odometer_gui.py --port /dev/ttyUSB1 --side right`
for the right board; choose the actual port shown on your computer. The
wheel-test firmware is in `firmware/esp32_wheel_odometer_test/`. The GUI uses
the measured **660 quadrature counts per wheel revolution** for this test.
The firmware may start with a different CPR after reset; the GUI sends CPR 660
when it connects and again when powered PWM tuning starts.

## Wiring used by the test firmware

| Function | ESP32 GPIO | L298N or encoder |
| --- | --- | --- |
| M1 PWM | 32 | ENA; remove ENA jumper |
| M1 direction | 23, 13 | IN1, IN2 |
| M2 PWM | 27 | ENB; remove ENB jumper |
| M2 direction | 19, 4 | IN3, IN4 |
| M1 encoder A, B | 34, 25 | Encoder A, B |
| M2 encoder A, B | 18, 21 | Encoder A, B |

Connect the ESP32, encoder, and L298N grounds. M1 motor power goes to
OUT1/OUT2; M2 goes to OUT3/OUT4. Encoder signals do not go to L298N outputs.
Confirm the encoder supply voltage and that its outputs are safe for 3.3 V
ESP32 inputs. GPIO34 has no internal pull-up. An open-collector encoder output
on that pin may need an external pull-up to 3.3 V; do not add one without
checking the encoder output circuit. The left board's M2 drive direction is
reversed in firmware; the right board retains the normal drive direction.

## What the 2026-10-21 right-board test showed

The displayed M1 A/B edge totals were **24,618 / 868** (about 4% balance).
This is a serious channel imbalance, so M1's tick count cannot be trusted for
position or PWM tuning. Inspect the GPIO34 A connection, GPIO25 B connection,
encoder ground, connector seating, and whether A needs a 3.3 V pull-up. Reset
the counters and retest; A and B should increase at roughly equal rates.

M2 showed balanced A/B edges (795 / 795) and zero displayed glitches, but one
660-tick target ended at **1,433 ticks**. The reported **+773 coast ticks**
account for the overshoot: the wheel kept turning after the firmware commanded
braking. This observation does not by itself prove whether the cause is the
PWM enable connection, ineffective braking, supply/driver behavior, or wheel
inertia. Check that the ENB jumper is removed, GPIO27 reaches ENB, and the
common ground and motor supply are solid. A fitted ENB jumper can prevent the
ESP32 from varying the channel's speed. Compare results at different PWM
values; if coast remains large and similar, inspect the driver and braking
path before changing calibration constants.

## Powered PWM tuning

Lift the wheels clear of the floor and keep hands away from them. In the GUI,
click **Auto Tune M1 PWM** or **Auto Tune M2 PWM**. Test only one wheel at a
time. The tuner sends CPR 660 and tries max PWM values **90, 120, 150, 180**,
commanding one closed-loop revolution for each trial. It compares the final
encoder error and fills the **Max PWM** field with the best tested value only
if its final error is within **33 ticks (5%)**. It stops on missing feedback,
bad A/B balance, excessive glitches, a firmware abort, or a missing result.
The **EMERGENCY STOP** button cancels a running tune.

PWM is a speed limit, not a fixed number of ticks. Reaching exactly one
revolution requires encoder feedback and effective braking; no fixed PWM can
guarantee an exact turn as battery voltage and wheel load change. Do not tune
M1 while its A/B edge counts are imbalanced. If M2 still coasts by hundreds of
ticks at all tested PWM values, the app will report that no acceptable setting
was found; repair the enable/braking path before repeating the test.
