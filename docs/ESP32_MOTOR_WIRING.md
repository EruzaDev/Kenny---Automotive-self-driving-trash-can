# ESP32 and L298N motor wiring

## Hardware and scope

This records the wiring agreed during assembly: two classic 38-pin
ESP32-WROOM-32 development boards, two common L298N driver modules, four
JGB37-520 encoder gearmotors, and a nominal 12 V motor supply. The later
reference to JGB37-529 needs confirmation against the actual motor label.

One ESP32 and one L298N control the two left motors; another ESP32 and
L298N control the two right motors. Motor 1 means the motor on OUT1/OUT2;
Motor 2 means the motor on OUT3/OUT4. Front/rear positions are not yet assigned.
Follow printed GPIO labels, not physical header position numbers.

This is a wiring record, not tested motor firmware. Encoder supply voltage,
output type, wire colors, motor stall current, and the exact module revisions
remain unverified. Confirm these before powering the complete assembly.

## Control wiring: repeat on both sides

| ESP32 pin | Its L298N pin |
| --- | --- |
| GPIO32 | ENA |
| GPIO23 | IN1 |
| GPIO13 | IN2 |
| GPIO27 | ENB |
| GPIO19 | IN3 |
| GPIO4 | IN4 |
| GND | GND |

Remove both ENA and ENB jumpers before connecting the GPIOs. IN1/IN2 control
Motor 1 direction; IN3/IN4 control Motor 2 direction. ENA/ENB receive PWM.
The L298 accepts 3.3 V control signals; no level shifter is needed on these
ESP32-to-driver connections.

| Motor power wires | L298N terminals |
| --- | --- |
| Motor 1, two power wires | OUT1 and OUT2 |
| Motor 2, two power wires | OUT3 and OUT4 |

Encoder wires do not connect to IN1–IN4 or OUT1–OUT4.

## Encoder wiring

The final requested swap exchanges Motor 1 and Motor 2 encoder assignments
on the **left ESP32**. It does not exchange A and B within either encoder.
The right side retains its preceding assignments.

| Encoder signal | Left ESP32, after swap | Right ESP32 |
| --- | --- | --- |
| Motor 1 A | GPIO18 | GPIO34 |
| Motor 1 B | GPIO21 | GPIO25 |
| Motor 2 A | GPIO34 | GPIO18 |
| Motor 2 B | GPIO25 | GPIO21 |
| Both encoder GND wires | Shared GND | Shared GND |
| Both encoder VCC wires | Confirm encoder-rated supply first | Confirm encoder-rated supply first |

GPIO22 is not used. Configure firmware with the corresponding side's table.
Do not infer encoder A/B or VCC/GND from yellow/green or other wire colors;
manufacturer wiring information is required.

Connect A/B directly only after confirming 3.3 V compatible outputs. A 5 V
push-pull output requires suitable voltage conversion. Open-collector outputs
may need external pull-ups to 3.3 V, subject to the encoder circuit. GPIO34
has no internal pull-up. The encoder supply is separate from motor power:
do not connect it to 12 V without an explicit encoder specification.

## Resistors and capacitors

| Component, per side | First connection | Second connection |
| --- | --- | --- |
| 10 kΩ, 1/4 W resistor 1 | ENA / GPIO32 junction | GND |
| 10 kΩ, 1/4 W resistor 2 | ENB / GPIO27 junction | GND |
| 1,000 µF / 50 V electrolytic | Positive lead to driver +12V / VS | Striped negative lead to driver GND |
| 100 nF / 50 V ceramic, marked 104, for Motor 1 | Motor 1 power terminal 1 | Motor 1 power terminal 2 |
| 100 nF / 50 V ceramic, marked 104, for Motor 2 | Motor 2 power terminal 1 | Motor 2 power terminal 2 |

The two 10 kΩ resistors hold the enable inputs LOW while the ESP32 starts.
No additional pull-down resistors are specified for IN1–IN4.

```text
GPIO32 ----+---- ENA       GPIO27 ----+---- ENB
           |                         |
         [10 kΩ]                   [10 kΩ]
           |                         |
          GND                       GND
```

The GPIO, enable input, and one resistor leg share a junction; the resistor
is not in series with the control wire. Its GND leg may share the electrolytic's
negative connection, provided that point is wired to driver GND.

The electrolytic bridges the driver's supply input and GND. It is not in
series with motor power and does not replace a sufficient power supply.
Keep it close to the driver and observe polarity. The available 50 V rating
is suitable for a 12 V supply; a 25 V part is also a common choice for a
regulated 12 V supply with appropriate transient margin.

Ceramics bridge the two motor power terminals and have no polarity. Keep
motor wires connected. Placement directly on the motor is preferred for noise
suppression; placement across the corresponding OUT1/OUT2 or OUT3/OUT4 terminal
block is possible but less effective with long motor leads. Neither motor
output is a fixed GND. Do not replace a capacitor with a direct wire bridge.

For both sides together: **four 10 kΩ resistors, four 100 nF ceramics, and
two 1,000 µF electrolytics**. Two ceramics cover only two of the four motors.

## Power distribution

| From | Connect to |
| --- | --- |
| 12 V supply positive, through a suitably rated fuse | Both L298N +12V / VS terminals |
| 12 V supply negative | Shared ground distribution |
| Both L298N GND terminals | Shared ground distribution |
| Both ESP32 GND pins | Shared ground distribution |
| All encoder GND wires | Shared ground distribution |
| USB power | Each ESP32 USB port |
| L298N 5V terminals | Leave disconnected in this arrangement |

On common modules, keep the **5V-EN regulator jumper installed only if the
module instructions permit onboard regulation at the actual supply voltage**.
This jumper is different from the ENA and ENB jumpers, which must be removed.
Confirm the battery's maximum charged voltage, not just its nominal rating.
Never connect 12 V to an ESP32. Use one power input per ESP32; do not also
feed its 5 V header while using ordinary USB power unless the board explicitly
supports that arrangement.

Use separate adequately sized power wires from each driver to the distribution
point so motor current does not flow through ESP32 headers or signal wiring.
Select fuse and wire ratings from the actual loads. Motor stall current and
cooling must be checked before accepting the L298N: its 2 A DC per-channel
rating is subject to thermal conditions, and the driver has substantial
voltage drop. This documentation does not establish motor/driver suitability.

## Perfboard and terminal blocks

Disconnect USB and 12 V power before soldering or checking continuity.
Ordinary perfboard pads are separate; make intended connections with soldered
wires. Adjacent terminal-block positions are usually separate unless linked.
Use secure, adequately sized conductors for motor current rather than Dupont
signal jumpers or small solder bridges.

All GND connections may join together. Different GPIO signals must stay
separate, including unused pins: some pins have boot or internal functions.
Remove accidental bridges. Encoder A and B must not be shorted together.
Across a disconnected 10 kΩ resistor, expect roughly 10 kΩ rather than a
continuity beep; measurements in an assembled circuit can include other paths.

## References

- [Espressif ESP32-DevKitC pinout and power options](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html)
- [Espressif GPIO limitations](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html)
- [ST L298 datasheet](https://www.st.com/resource/en/datasheet/l298.pdf)
- [Example common L298N module instructions; verify against the actual board](https://www.handsontec.com/dataspecs/L298N%20Motor%20Driver.pdf)
- [Pololu motor noise suppression guidance](https://www.pololu.com/docs/0J15/9)
