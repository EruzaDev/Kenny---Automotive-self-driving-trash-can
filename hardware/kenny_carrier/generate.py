"""Generate the provisional Kenny carrier schematic, placement and routing input.

Run with KiCad 7 system Python. This overwrites generated files, not a routed PCB.
Footprint/pin ordering is explicitly PROVISIONAL until the actual DevKit is measured.
"""
from pathlib import Path
import csv
import json
import math
import re
import uuid

import pcbnew as p

ROOT = Path(__file__).resolve().parent
NAME = "kenny_carrier_revA"
NS = uuid.UUID("ecba51ed-5620-4889-8979-fd37f4d084d2")
COMPONENTS = []


def uid(text):
    return str(uuid.uuid5(NS, text))


def q(text):
    return json.dumps(str(text))


def add(ref, value, footprint, pins, xy, angle=0, description="", names=None):
    COMPONENTS.append(dict(ref=ref, value=value, footprint=footprint,
                           pins={str(k): v for k, v in pins.items()}, xy=xy, angle=angle,
                           description=description, names=names or {}))


RFP = "Resistor_SMD:R_0805_2012Metric"
CFP = "Capacitor_SMD:C_0805_2012Metric"
HEADER = "Connector_PinHeader_2.54mm:PinHeader_1x{:02d}_P2.54mm_Vertical"
TERMINAL = "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-4-5.08_1x04_P5.08mm_Horizontal"
DRIVER_TERMINAL = "TerminalBlock_Phoenix:TerminalBlock_Phoenix_PT-1,5-7-3.5-H_1x07_P3.50mm_Horizontal"
ESTOP_TERMINAL = "TerminalBlock_Phoenix:TerminalBlock_Phoenix_PT-1,5-2-3.5-H_1x02_P3.50mm_Horizontal"
BOARD_WIDTH, BOARD_HEIGHT = 118, 125


def circuit():
    # Top view, antenna UP, USB DOWN. Numbers are carrier pad numbers, NOT GPIOs.
    left = ["3V3", "EN", "36", "39", "34", "35", "32", "33", "25", "26", "27", "14",
            "12", "GND", "13", "SD2", "SD3", "CMD", "5V"]
    right = ["GND", "23", "22", "TX0", "RX0", "21", "GND", "19", "18", "5", "17", "16",
             "4", "0", "2", "15", "SD1", "SD0", "CLK"]
    gpio = {"3V3": "ESP_3V3", "EN": "ESP_EN", "5V": "USB_5V", "GND": "GND",
            "25": "PWM_LF", "26": "PWM_RF", "32": "PWM_LR", "33": "PWM_RR",
            "34": "ENC_LF_A", "27": "ENC_LF_B", "35": "ENC_RF_A", "14": "ENC_RF_B",
            "36": "ENC_LR_A", "13": "ENC_LR_B", "39": "ENC_RR_A", "23": "ENC_RR_B",
            "21": "SDA", "22": "SCL", "16": "HX_DOUT", "17": "HX_SCK", "18": "ESTOP"}
    add("U1", "ESP32-WROOM 38pin PROVISIONAL", "Kenny:ESP32_38_PROVISIONAL",
        {i + 1: gpio.get(name) for i, name in enumerate(left + right)}, (44, 20),
        description="2x19 female sockets; 2.54mm pitch; assumed 25.40mm row spacing; verify module",
        names={str(i + 1): (name if name in ("GND", "3V3", "5V", "EN") else "GPIO" + name)
               for i, name in enumerate(left + right)})
    directions = ["LF_IN1", "LF_IN2", "RF_IN1", "RF_IN2", "LR_IN1", "LR_IN2", "RR_IN1", "RR_IN2"]
    mcp = {i + 1: net for i, net in enumerate(directions)}
    mcp.update({9: "+3V3", 10: "GND", 11: None, 12: "SCL", 13: "SDA", 14: None,
                15: "GND", 16: "GND", 17: "GND", 18: "MCP_RESET", 19: None, 20: None})
    mcp.update({i: None for i in range(21, 29)})
    mcp_names = {str(i + 1): "GPB" + str(i) for i in range(8)}
    mcp_names.update({str(i + 21): "GPA" + str(i) for i in range(8)})
    mcp_names.update({"9": "VDD", "10": "VSS", "11": "NC", "12": "SCL", "13": "SDA",
                      "14": "NC", "15": "A0", "16": "A1", "17": "A2", "18": "~RESET",
                      "19": "INTB", "20": "INTA"})
    add("U2", "MCP23017-E/SO", "Package_SO:SOIC-28W_7.5x17.9mm_P1.27mm", mcp, (87, 68),
        description="I2C address 0x20, GPB0..7 independent motor directions", names=mcp_names)
    add("U3", "AP2112K-3.3TRG1", "Package_TO_SOT_SMD:SOT-23-5",
        {1: "USB_5V", 2: "GND", 3: "USB_5V", 4: None, 5: "+3V3"}, (84, 43),
        names={"1": "VIN", "2": "GND", "3": "EN", "4": "NC", "5": "VOUT"},
        description="Peripheral 3.3V LDO; input from DevKit USB 5V pin ONLY; 150mA initial budget")
    for index, wheel in enumerate(("LF", "RF", "LR", "RR")):
        y = 12 + 25 * index
        add("J" + str(index + 1), wheel + " ENCODER", TERMINAL,
            {1: "+3V3", 2: "GND", 3: wheel + "_A_RAW", 4: wheel + "_B_RAW"}, (8, y), 270,
            description="5.08mm 4-way terminal; 1 BLUE +3V3, 2 BLACK GND, 3 YELLOW A, 4 GREEN B")
        for channel, offset in (("A", 0), ("B", 1)):
            n = index * 2 + offset + 1
            sy = y + 10 + offset * 5
            add("R" + str(n), "1k", RFP, {1: wheel + "_" + channel + "_RAW", 2: "ENC_" + wheel + "_" + channel}, (27, sy),
                description="Encoder series resistor, together with 1nF gives 1us RC; NOT a level shifter")
            add("C" + str(n), "1nF 50V C0G", CFP, {1: "ENC_" + wheel + "_" + channel, 2: "GND"}, (33, sy))
        add("C" + str(9 + index), "100nF 16V X7R", CFP, {1: "+3V3", 2: "GND"}, (29, y + 1))
    add("J5", "MPU6050", HEADER.format(4), {1: "+3V3", 2: "GND", 3: "SDA", 4: "SCL"}, (103, 10),
        description="2.54mm Dupont: 3V3 GND SDA SCL; wire by signal, module header order can differ")
    add("J6", "HX711 MODULE", HEADER.format(4), {1: "+3V3", 2: "GND", 3: "HX_DOUT", 4: "HX_SCK"}, (103, 25),
        description="2.54mm Dupont: 3V3 GND DT SCK; raw load cell goes to HX711 E+/E-/A+/A-")
    for ref, wheels, y in (("J7", ("LF", "RF"), 55), ("J8", ("LR", "RR"), 87)):
        a, b = wheels
        add(ref, "L298N " + ("FRONT" if ref == "J7" else "REAR"), DRIVER_TERMINAL,
            {1: "GND", 2: "PWM_" + a, 3: a + "_IN1", 4: a + "_IN2", 5: b + "_IN1", 6: b + "_IN2", 7: "PWM_" + b},
            (103, y), 270, description="3.50mm 7-way terminal: GND ENA IN1 IN2 IN3 IN4 ENB; logic only")
    add("J9", "ESTOP NC", ESTOP_TERMINAL, {1: "GND", 2: "ESTOP"}, (79, 114),
        description="3.50mm 2-way terminal; normally-closed contact to GND; open=stop; external power cut remains required")
    for n, net in enumerate(directions, 9):
        j = n - 9
        add("R" + str(n), "10k", RFP, {1: net, 2: "GND"}, (96, 62 + (j % 4) * 3.5 + (j // 4) * 32),
            description="Direction pull-down; MCP pins are high impedance during reset")
    for n, wheel in enumerate(("LF", "RF", "LR", "RR"), 17):
        add("R" + str(n), "10k", RFP, {1: "PWM_" + wheel, 2: "GND"}, (96, (58.5 if (n - 17) % 2 == 0 else 76) + ((n - 17) // 2) * 32),
            description="Enable pull-down; REMOVE L298N ENA/ENB jumpers")
    add("R21", "4.7k", RFP, {1: "+3V3", 2: "SDA"}, (83, 52), description="I2C pull-up; depopulate if module pull-ups make total too low")
    add("R22", "4.7k", RFP, {1: "+3V3", 2: "SCL"}, (89, 52), description="I2C pull-up")
    add("R23", "10k", RFP, {1: "+3V3", 2: "MCP_RESET"}, (78, 82), description="MCP reset pull-up")
    add("R24", "10k", RFP, {1: "ESP_3V3", 2: "ESTOP"}, (88, 113), description="E-stop input pull-up")
    add("R25", "10k", RFP, {1: "HX_SCK", 2: "GND"}, (95, 34), description="Hold HX711 clock low during ESP reset")
    for ref, value, net, xy in (
        ("C13", "1uF 16V X7R", "USB_5V", (79, 43)),
        ("C14", "10uF 10V X5R", "+3V3", (89, 43)),
        ("C15", "100nF 16V X7R", "+3V3", (77, 73)),
        ("C16", "100nF 16V X7R", "+3V3", (96, 14)),
        ("C17", "100nF 16V X7R", "+3V3", (96, 28)),
        ("C18", "10uF 10V X5R", "+3V3", (89, 47)),
        ("C19", "100nF 16V X7R", "MCP_RESET", (84, 82)),
    ):
        add(ref, value, CFP, {1: net, 2: "GND"}, xy)
    for ref, net, xy in (("TP1", "+3V3", (77, 38)), ("TP2", "GND", (83, 38)), ("TP3", "USB_5V", (89, 38))):
        add(ref, net, "TestPoint:TestPoint_Pad_D2.0mm", {1: net}, xy, description="Supply measurement")


def schematic():
    root_uuid = uid("root")
    libs, instances, wires = [], [], []
    for index, c in enumerate(COMPONENTS):
        ref = c["ref"]
        count = len(c["pins"])
        dual = count > 8
        rows = math.ceil(count / 2) if dual else count
        height = max(5.08, rows * 2.54)
        body = f'(rectangle (start -12.7 {height/2}) (end 12.7 {-height/2}) (stroke (width 0.254) (type default)) (fill (type background)))'
        passive = ref.startswith(('R', 'C')) and count == 2
        if passive:
            if ref.startswith('R'):
                body = '(rectangle (start -2.54 1.016) (end 2.54 -1.016) (stroke (width 0.254) (type default)) (fill (type none)))'
            else:
                body = ''.join(f'(polyline (pts (xy {xx} 1.8) (xy {xx} -1.8)) (stroke (width 0.254) (type default)) (fill (type none)))' for xx in (-0.635, 0.635))
        pin_defs, coords = [], []
        for i, (number, net) in enumerate(c["pins"].items()):
            right = (dual and i >= rows) or (passive and i == 1)
            px = (5.08 if passive else 15.24) * (1 if right else -1)
            py = 0 if passive else (rows - 1) * 1.27 - (i % rows) * 2.54
            length = 4.445 if ref.startswith('C') and passive else 2.54
            pin_name = c["names"].get(number, number)
            kind = "passive"
            if ref == "U2":
                kind = "power_in" if number in ("9", "10") else "bidirectional" if number in [str(n) for n in list(range(1, 9)) + list(range(21, 29))] + ["13"] else "input"
                if number in ("11", "14"): kind = "no_connect"
                if number in ("19", "20"): kind = "output"
            pin_defs.append(f'(pin {kind} line (at {px} {py} {180 if right else 0}) (length {length}) (name {q(pin_name)} (effects (font (size 1 1)))) (number {q(number)} (effects (font (size 1 1)))))')
            coords.append((px, py, net, right))
        symbol = f'Kenny:{ref}'
        libs.append(f'''(symbol {q(symbol)} (pin_names (offset 0.5) {'hide' if passive else ''}) (in_bom yes) (on_board yes)
          (property "Reference" {q(ref)} (at 0 {height/2+3} 0) (effects (font (size 1.27 1.27))))
          (property "Value" {q(c['value'])} (at 0 {-height/2-3} 0) (effects (font (size 1.27 1.27))))
          (symbol "{ref}_0_1" {body}) (symbol "{ref}_1_1" {''.join(pin_defs)}))''')
        if index < 3:
            x, y = [(80, 65), (235, 65), (390, 40)][index]
        else:
            n = index - 3
            x, y = 50 + (n % 7) * 75, 150 + (n // 7) * 29
        instid = uid(ref)
        c["uuid"] = instid
        instances.append(f'''(symbol (lib_id {q(symbol)}) (at {x} {y} 0) (unit 1)
          (in_bom yes) (on_board yes) (dnp no) (uuid {instid})
          (property "Reference" {q(ref)} (at {x} {y-height/2-4} 0) (effects (font (size 1.27 1.27))))
          (property "Value" {q(c['value'])} (at {x} {y+height/2+3} 0) (effects (font (size 1.0 1.0))))
          (property "Footprint" {q(c['footprint'])} (at {x} {y} 0) (effects (font (size 1 1)) hide))
          {''.join(f'(pin {q(n)} (uuid {uid(ref+"pin"+n)}))' for n in c['pins'])}
          (instances (project {q(NAME)} (path "/{root_uuid}" (reference {q(ref)}) (unit 1)))))''')
        for j, (px, py, net, right) in enumerate(coords):
            xx, yy = x + px, y - py
            if net is None:
                wires.append(f'(no_connect (at {xx} {yy}) (uuid {uid(ref+str(j)+"nc")}))')
            else:
                end = xx + (5.08 if right else -5.08)
                wires.append(f'(wire (pts (xy {xx} {yy}) (xy {end} {yy})) (stroke (width 0) (type default)) (uuid {uid(ref+str(j)+"wire")}))')
                wires.append(f'(label {q(net)} (at {end} {yy} 0) (effects (font (size 1 1)) (justify {"left" if right else "right"} bottom)) (uuid {uid(ref+str(j)+"label")}))')
    text = f'''(kicad_sch (version 20230121) (generator eeschema) (uuid {root_uuid}) (paper "A2")
      (title_block (title "KENNY ESP32 CARRIER - PROVISIONAL REV A") (rev "A-PROTOTYPE")
        (comment 1 "ESP32 footprint and sensor-module pinout MUST be verified before fabrication")
        (comment 2 "USB powered logic only; external L298N motor power; MCP23017 directions"))
      (lib_symbols {''.join(libs)})
      (text "USB ONLY: U1 pin 19 -> U3 -> +3V3 peripherals. ESP_3V3 is separate.\\nMPU6050 0x68 / MCP23017 0x20. HX711 module, NOT raw load cell.\\nEncoder J1..J4: 1 BLUE +3V3, 2 BLACK GND, 3 YELLOW A, 4 GREEN B.\\nMotor red/white wires go directly to external L298N outputs.\\nPWM=25/26/32/33; direction pins now MCP GPB0..7. Firmware port REQUIRED."
        (at 330 80 0) (effects (font (size 1.25 1.25)) (justify left top)) (uuid {uid('notes')}))
      {''.join(instances)} {''.join(wires)}
      (sheet_instances (path "/" (page "1"))))'''
    (ROOT / (NAME + ".kicad_sch")).write_text(text)


def vec(x, y):
    return p.VECTOR2I(p.FromMM(x), p.FromMM(y))


def board():
    b = p.BOARD()
    b.SetCopperLayerCount(2)
    b.GetDesignSettings().m_MinClearance = p.FromMM(0.2)
    b.GetDesignSettings().m_TrackMinWidth = p.FromMM(0.25)
    b.GetDesignSettings().m_CopperEdgeClearance = p.FromMM(0.5)
    b.GetDesignSettings().m_ViasMinSize = p.FromMM(0.6)
    b.GetDesignSettings().m_MinThroughDrill = p.FromMM(0.3)
    nets = {}
    for c in COMPONENTS:
        for net in c["pins"].values():
            if net is not None and net not in nets:
                n = p.NETINFO_ITEM(b, net, len(nets) + 1)
                b.Add(n)
                nets[net] = n

    def shape(a, z, layer=p.F_SilkS, width=0.18, kind=p.SHAPE_T_SEGMENT):
        s = p.PCB_SHAPE()
        s.SetShape(kind)
        s.SetStart(vec(*a)); s.SetEnd(vec(*z)); s.SetLayer(layer); s.SetWidth(p.FromMM(width))
        b.Add(s)
        return s

    def text(value, x, y, size=1, angle=0, layer=p.F_SilkS):
        size = max(size, 0.8)
        t = p.PCB_TEXT(b)
        t.SetText(value); t.SetPosition(vec(x, y)); t.SetLayer(layer)
        t.SetTextSize(vec(size, size)); t.SetTextThickness(p.FromMM(0.15))
        t.SetTextAngle(p.EDA_ANGLE(angle, p.DEGREES_T))
        b.Add(t)

    # Local self-contained provisional DevKit footprint with explicit physical pad numbers.
    fp = p.FOOTPRINT(b)
    fp.SetFPID(p.LIB_ID("Kenny", "ESP32_38_PROVISIONAL"))
    fp.SetReference("U1"); fp.SetValue("ESP32_38_PROVISIONAL")
    for col in range(2):
        for row in range(19):
            pad = p.PAD(fp); pad.SetNumber(str(col * 19 + row + 1))
            pad.SetAttribute(p.PAD_ATTRIB_PTH)
            pad.SetShape(p.PAD_SHAPE_RECT if row == 0 else p.PAD_SHAPE_OVAL)
            layers = p.LSET.AllCuMask(); layers.AddLayer(p.F_Mask); layers.AddLayer(p.B_Mask)
            pad.SetSize(vec(1.8, 1.8)); pad.SetDrillSize(vec(1, 1)); pad.SetLayerSet(layers)
            pad.SetPosition(vec(col * 25.4, row * 2.54)); pad.SetPos0(vec(col * 25.4, row * 2.54)); fp.Add(pad)
    library = ROOT / "Kenny.pretty"
    library.mkdir(exist_ok=True)
    p.FootprintSave(str(library), fp)

    for c in COMPONENTS:
        lib, name = c["footprint"].split(":", 1)
        path = library if lib == "Kenny" else Path("/usr/share/kicad/footprints") / (lib + ".pretty")
        f = p.FootprintLoad(str(path), name)
        if not f: raise RuntimeError(c["footprint"])
        f.SetReference(c["ref"]); f.SetValue(c["value"])
        f.SetFPID(p.LIB_ID(lib, name))
        f.SetPath(p.KIID_PATH("/" + uid("root") + "/" + c["uuid"]))
        f.SetPosition(vec(*c["xy"])); f.SetOrientationDegrees(c["angle"])
        f.Value().SetVisible(False)
        f.Reference().SetTextSize(vec(0.8, 0.8)); f.Reference().SetTextThickness(p.FromMM(0.12))
        # Put identifiers on fab; connector pinouts have their own readable silkscreen.
        f.Reference().SetLayer(p.F_Fab)
        for pad in f.Pads():
            net = c["pins"].get(pad.GetNumber())
            if net: pad.SetNet(nets[net])
        b.Add(f)
    for index, (x, y) in enumerate(((4, 4), (114, 4), (4, 121), (114, 121)), 1):
        f = p.FootprintLoad("/usr/share/kicad/footprints/MountingHole.pretty", "MountingHole_3.2mm_M3")
        f.SetFPID(p.LIB_ID("MountingHole", "MountingHole_3.2mm_M3"))
        f.SetPosition(vec(x, y)); f.SetReference("H" + str(index)); f.Reference().SetVisible(False); f.Value().SetVisible(False)
        b.Add(f)
    for a, z in (((0, 0), (BOARD_WIDTH, 0)), ((BOARD_WIDTH, 0), (BOARD_WIDTH, BOARD_HEIGHT)),
                 ((BOARD_WIDTH, BOARD_HEIGHT), (0, BOARD_HEIGHT)), ((0, BOARD_HEIGHT), (0, 0))):
        shape(a, z, p.Edge_Cuts, 0.05)
    # Antenna no-copper/component region reaches the carrier's top edge.
    zone = p.ZONE(b); zone.SetIsRuleArea(True)
    zone.SetLayerSet(p.LSET.ExternalCuMask()); zone.SetDoNotAllowTracks(True); zone.SetDoNotAllowVias(True)
    zone.SetDoNotAllowCopperPour(True); zone.SetDoNotAllowPads(True); zone.SetDoNotAllowFootprints(True)
    outline = zone.Outline(); outline.NewOutline()
    for x, y in ((32, 0), (81, 0), (81, 17), (32, 17)):
        outline.Append(p.FromMM(x), p.FromMM(y))
    b.Add(zone)
    text("ANTENNA - KEEP CLEAR", 56.7, 5, 1.2)
    text("PROVISIONAL 38 PIN", 56.7, 29, 1)
    text("VERIFY ROWS 25.40mm", 56.7, 32, 0.9)
    text("USB END", 56.7, 74, 1.2)
    # Module envelope is on fabrication layer only; do not put parts under it.
    shape((39, 7), (74.4, 78), p.F_Fab, 0.15, p.SHAPE_T_RECT)
    for i, wheel in enumerate(("LEFT FRONT", "RIGHT FRONT", "LEFT REAR", "RIGHT REAR")):
        y = 12 + 25 * i
        text("J" + str(i + 1) + " " + wheel, 18, y - 5.5, 0.95)
        for j, label in enumerate(("1 3V3 BLU", "2 GND BLK", "3 A YEL", "4 B GRN")):
            text(label, 19, y + j * 5.08 + 2.1, 0.8)
    for x, y, heading, labels in (
        (103, 10, "J5 MPU6050", ("3V3", "GND", "SDA", "SCL")),
        (103, 25, "J6 HX711", ("3V3", "GND", "DT", "SCK")),
    ):
        text(heading, 95, y - 3, 1)
        for i, label in enumerate(labels): text(label, 108, y + i * 2.54, 0.7)
    text("J9 ESTOP", 81, 108.7, 0.85)
    text("GND   SIG", 81, 120.5, 0.8)
    for ref, y, name in (("J7", 55, "FRONT"), ("J8", 87, "REAR")):
        text(ref + " L298N " + name, 99, y - 4.6, 0.85)
        for i, label in enumerate(("G", "EA", "I1", "I2", "I3", "I4", "EB")):
            text(str(i+1) + " " + label, 113, y + i * 3.5, 0.8)
    text("USB POWER ONLY", 87, 35, 0.9)
    text("3V3  GND  5V", 83, 40, 0.8)
    text("MCP23017 0x20", 87, 56, 0.85)
    text("NO MOTOR POWER ON PCB", 53, 117, 0.85)
    text("KENNY", 56, 82, 3)
    text("CARRIER REV A", 56, 86, 0.9)
    # Simplified monochrome robot mark derived from user's supplied Kenny image.
    shape((51, 91), (61, 97), kind=p.SHAPE_T_RECT, width=0.4)
    shape((52, 98), (60, 103), kind=p.SHAPE_T_RECT, width=0.4)
    shape((56, 91), (56, 89), width=0.3)
    shape((56, 88.7), (56.5, 88.7), kind=p.SHAPE_T_CIRCLE, width=0.35)
    for x in (54, 58): shape((x, 93.4), (x+0.5, 93.4), kind=p.SHAPE_T_CIRCLE, width=0.4)
    for x in (53, 59): shape((x, 103.8), (x+0.8, 103.8), kind=p.SHAPE_T_CIRCLE, width=0.5)
    shape((52, 100), (50, 101.5), width=0.4); shape((60, 100), (62, 101.5), width=0.4)
    text("KENNY", 56, 100.5, 0.7)
    p.SaveBoard(str(ROOT / (NAME + "_placement.kicad_pcb")), b)
    p.ExportSpecctraDSN(b, str(ROOT / (NAME + ".dsn")))
    dsn_path = ROOT / (NAME + ".dsn")
    dsn = dsn_path.read_text().replace('(width 200)', '(width 250)')
    # Supply traces are routed wider; GND also receives a bottom copper plane later.
    start = dsn.index('(class kicad_default')
    finish = dsn.index('(circuit', start)
    section = dsn[start:finish]
    for net in ('+3V3', 'USB_5V', 'GND'):
        section = re.sub(r'(?<!\S)' + re.escape(net) + r'(?!\S)', '', section)
    dsn = dsn[:start] + section + dsn[finish:]
    power_class = '(class power +3V3 USB_5V GND (circuit (use_via Via[0-1]_600:300_um)) (rule (width 400) (clearance 200.1)))\n'
    dsn = dsn[:start] + power_class + dsn[start:]
    dsn_path.write_text(dsn)
    libraries = sorted({c['footprint'].split(':')[0] for c in COMPONENTS} | {'MountingHole'})
    entries = []
    for lib in libraries:
        uri = "${KIPRJMOD}/Kenny.pretty" if lib == "Kenny" else "${KICAD7_FOOTPRINT_DIR}/" + lib + ".pretty"
        entries.append(f'(lib (name {q(lib)})(type "KiCad")(uri {q(uri)})(options "")(descr ""))')
    (ROOT / "fp-lib-table").write_text('(fp_lib_table ' + ''.join(entries) + ')\n')
    return b


def main():
    circuit(); schematic(); board()
    (ROOT / "circuit.json").write_text(json.dumps(COMPONENTS, indent=2) + "\n")
    with (ROOT / "BOM.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(["Reference", "Value", "Footprint", "Description"])
        for c in COMPONENTS: writer.writerow([c["ref"], c["value"], c["footprint"], c["description"]])
    project = {
        'meta': {'filename': NAME + '.kicad_pro', 'version': 1},
        'board': {'design_settings': {'rules': {
            'min_clearance': 0.2, 'min_track_width': 0.25, 'min_via_diameter': 0.6,
            'min_through_hole_diameter': 0.3, 'min_copper_edge_clearance': 0.5,
            'min_hole_to_hole': 0.25, 'min_silk_text_height': 0.8,
            'min_silk_text_thickness': 0.12, 'silk_pad_clearance': 0.1,
        }}},
        'net_settings': {'meta': {'version': 3}, 'classes': [
            {'name': 'Default', 'clearance': 0.2, 'track_width': 0.25, 'via_diameter': 0.6, 'via_drill': 0.3},
            {'name': 'Power', 'clearance': 0.2, 'track_width': 0.4, 'via_diameter': 0.6, 'via_drill': 0.3},
        ], 'netclass_patterns': [{'netclass': 'Power', 'pattern': net} for net in ('GND', '+3V3', 'USB_5V')]},
    }
    for suffix in ('', '_placement'):
        (ROOT / (NAME + suffix + '.kicad_pro')).write_text(json.dumps(project, indent=2) + '\n')
    print(f"Generated {len(COMPONENTS)} components. Provisional {BOARD_WIDTH} x {BOARD_HEIGHT} mm, 2 layers.")


if __name__ == "__main__":
    main()
