"""Cross-check both mobile motor maps against the serial sketch and PCB net map."""
import json
from pathlib import Path
import re

repo = Path(__file__).resolve().parents[1]
mobile = (repo / "firmware/kenny_mobile/kenny_mobile.ino").read_text()
serial = (repo / "firmware/kenny_esp32/kenny_esp32.ino").read_text()
mobile_table = mobile.split("Motor motors[MOTOR_COUNT] = {", 1)[1].split("};", 1)[0]
serial_table = serial.split("Motor motors[WHEEL_COUNT] = {", 1)[1].split("};", 1)[0]
pattern = r"\{\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*([+-]?\d+)\s*\}"
rows = [tuple(map(int, row)) for row in re.findall(pattern, mobile_table)]
old = [tuple(map(int, row)) for row in re.findall(pattern, serial_table)]
assert len(rows) == 8 and len(old) == 4
carrier, direct = rows[:4], rows[4:]
assert direct == old, "Mobile direct wiring must match the earlier independent-GPIO sketch"
parts = {part["ref"]: part for part in json.loads((repo / "hardware/kenny_carrier/circuit.json").read_text())}
gpio = {net: int(parts["U1"]["names"][pin].removeprefix("GPIO"))
        for pin, net in parts["U1"]["pins"].items()
        if net and parts["U1"]["names"][pin].startswith("GPIO")}
for i, wheel in enumerate(("LF", "RF", "LR", "RR")):
    pwm, bit1, bit2, a, b, sign = carrier[i]
    assert pwm == gpio["PWM_" + wheel]
    assert a == gpio["ENC_" + wheel + "_A"] and b == gpio["ENC_" + wheel + "_B"]
    assert parts["U2"]["pins"][str(bit1 + 1)] == wheel + "_IN1"
    assert parts["U2"]["pins"][str(bit2 + 1)] == wheel + "_IN2"
config = (repo / "firmware/kenny_mobile/config.h").read_text()
carrier_config = config.split("#if KENNY_WIRING == KENNY_CARRIER_MCP23017", 1)[1].split("#else", 1)[0]
direct_config = config.split("#else", 1)[1]
def number(text, name):
    return int(re.search(r"\b" + name + r"\s*=\s*(\d+)", text)[1])
assert number(carrier_config, "ESTOP_PIN") == gpio["ESTOP"]
assert number(carrier_config, "I2C_SDA") == gpio["SDA"]
assert number(carrier_config, "I2C_SCL") == gpio["SCL"]
assert number(carrier_config, "HX_DOUT") == gpio["HX_DOUT"]
assert number(carrier_config, "HX_SCK") == gpio["HX_SCK"]
assert number(direct_config, "ESTOP_PIN") == number(serial, "ESTOP_PIN")
print("Mobile pin maps match the earlier direct-wire sketch and carrier PCB motor/sensor/E-stop nets")
