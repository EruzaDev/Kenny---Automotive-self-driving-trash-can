"""Check PCB connections against the circuit specification and selected invariants.

These are connectivity checks, not a replacement for native schematic ERC or
electrical/thermal validation of the exact purchased modules.
"""
import json
import os
from pathlib import Path
import re

os.environ.setdefault('KICAD7_FOOTPRINT_DIR', '/usr/share/kicad/footprints')
import pcbnew as p
from generate import ROOT, NAME
from finalize import verify_netlist


def main():
    board = p.LoadBoard(str(ROOT / (NAME + '.kicad_pcb')))
    count = verify_netlist(board)
    components = {c['ref']: c for c in json.loads((ROOT / 'circuit.json').read_text())}
    pins = {ref: c['pins'] for ref, c in components.items()}
    for ref in ('J1','J2','J3','J4','J7','J8','J9'):
        assert components[ref]['footprint'].startswith('TerminalBlock_')
    for ref in ('J5','J6'):
        assert components[ref]['footprint'].startswith('Connector_PinHeader_2.54mm:')
    esp_nets = {net for net in pins['U1'].values() if net}
    for i, wheel in enumerate(('LF', 'RF', 'LR', 'RR')):
        assert pins[f'J{i+1}'] == {'1': '+3V3', '2': 'GND', '3': wheel+'_A_RAW', '4': wheel+'_B_RAW'}
        for j, channel in enumerate(('A', 'B')):
            raw, clean = f'{wheel}_{channel}_RAW', f'ENC_{wheel}_{channel}'
            assert clean in esp_nets and raw not in esp_nets
            assert pins[f'R{2*i+j+1}'] == {'1': raw, '2': clean}
            assert pins[f'C{2*i+j+1}'] == {'1': clean, '2': 'GND'}
        assert pins[f'R{17+i}'] == {'1': 'PWM_'+wheel, '2': 'GND'}
    for i, direction in enumerate(('LF_IN1','LF_IN2','RF_IN1','RF_IN2','LR_IN1','LR_IN2','RR_IN1','RR_IN2')):
        assert pins['U2'][str(i+1)] == direction
        assert pins[f'R{i+9}'] == {'1': direction, '2': 'GND'}
    assert pins['J5'] == {'1': '+3V3', '2': 'GND', '3': 'SDA', '4': 'SCL'}
    assert pins['J6'] == {'1': '+3V3', '2': 'GND', '3': 'HX_DOUT', '4': 'HX_SCK'}
    assert pins['U1']['25'] == 'SDA' and pins['U1']['22'] == 'SCL'
    assert pins['U1']['31'] == 'HX_DOUT' and pins['U1']['30'] == 'HX_SCK'
    assert pins['U1']['28'] == 'ESTOP'
    assert all(pins['U2'][str(n)] == 'GND' for n in (15,16,17))
    assert pins['U3'] == {'1': 'USB_5V', '2': 'GND', '3': 'USB_5V', '4': None, '5': '+3V3'}
    footprints = {f.GetReference(): f for f in board.GetFootprints()}
    sockets = {pad.GetNumber(): pad for pad in footprints['U1'].Pads()}
    assert len(sockets) == 38
    assert abs(p.ToMM(sockets['20'].GetPosition().x - sockets['1'].GetPosition().x) - 25.4) < 0.001
    assert abs(p.ToMM(sockets['19'].GetPosition().y - sockets['1'].GetPosition().y) - 45.72) < 0.001
    for c in components.values():
        fp = footprints[c['ref']]
        assert {pad.GetNumber() for pad in fp.Pads()} == set(c['pins'])
    assert len([f for f in board.GetFootprints() if f.GetReference().startswith('H')]) == 4
    report = ROOT / 'DRC.txt'
    assert p.WriteDRCReport(board, str(report), p.EDA_UNITS_MILLIMETRES, True)
    drc = report.read_text()
    for kind in ('DRC violations', 'unconnected pads', 'Footprint errors'):
        assert f'Found 0 {kind}' in drc, f'KiCad check failed: {kind}; see DRC.txt'
    result = dict(connected_pads=count, footprints=len(footprints), drc_violations=0,
                  unconnected_pads=0, footprint_errors=0, connectivity_checks='passed',
                  fabrication_status='PROVISIONAL - module dimensions and electrical validation pending',
                  native_schematic_erc='not run (KiCad 7 CLI has no ERC subcommand)')
    (ROOT / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
