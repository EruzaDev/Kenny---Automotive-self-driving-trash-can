"""Import a Freerouting SES into the generated placement and run KiCad DRC."""
import argparse
import json
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pcbnew as p
from generate import ROOT, NAME, BOARD_WIDTH, BOARD_HEIGHT, vec


def parse(text):
    stack, root = [], None
    for token in re.findall(r'"(?:\\.|[^"\\])*"|[()]|[^\s()]+', text):
        if token == '(':
            node = []
            if stack: stack[-1].append(node)
            else: root = node
            stack.append(node)
        elif token == ')':
            stack.pop()
        else:
            stack[-1].append(token[1:-1] if token.startswith('"') else token)
    assert not stack
    return root


def children(node, name):
    return [item for item in node if isinstance(item, list) and item[0] == name]


def verify_netlist(board):
    components = json.loads((ROOT / 'circuit.json').read_text())
    expected = {(c['ref'], pin): net for c in components for pin, net in c['pins'].items() if net}
    root = ET.parse(ROOT / (NAME + '.net.xml')).getroot()
    actual = {}
    for net in root.findall('./nets/net'):
        for pin in net.findall('node'):
            actual[(pin.attrib['ref'], pin.attrib['pin'])] = net.attrib['name'].removeprefix('/')
    # KiCad generates synthetic names for explicitly unconnected symbol pins.
    unexpected = {key: value for key, value in actual.items()
                  if key not in expected and not value.startswith('unconnected-')}
    assert not unexpected, f'Unexpected schematic connections: {unexpected}'
    actual = {key: value for key, value in actual.items() if key in expected}
    assert actual == expected, f'Schematic mismatch: {set(expected.items()) ^ set(actual.items())}'
    pcb = {(fp.GetReference(), pad.GetNumber()): pad.GetNetname()
           for fp in board.GetFootprints() for pad in fp.Pads() if pad.GetNetCode()}
    assert pcb == expected, f'PCB mismatch: {set(expected.items()) ^ set(pcb.items())}'
    return len(expected)


def main():
    os.environ.setdefault('KICAD7_FOOTPRINT_DIR', '/usr/share/kicad/footprints')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    b = p.LoadBoard(str(ROOT / (NAME + '_placement.kicad_pcb')))
    nets = {n.GetNetname(): n for n in b.GetNetInfo().NetsByNetcode().values()}
    tree = parse(args.session.read_text())
    routes = children(tree, 'routes')[0]
    resolution = children(routes, 'resolution')[0]
    assert resolution[1] == 'um'
    divisor = 1000 * float(resolution[2])
    # Reject any unexpected placement changes; this import only supports routing.
    by_ref = {fp.GetReference(): fp for fp in b.GetFootprints()}
    placement = children(tree, 'placement')[0]
    for component in children(placement, 'component'):
        for place in children(component, 'place'):
            fp = by_ref[place[1]]
            assert abs(p.ToMM(fp.GetPosition().x) - float(place[2])/divisor) < 0.001
            assert abs(p.ToMM(fp.GetPosition().y) + float(place[3])/divisor) < 0.001
    for net in children(children(routes, 'network_out')[0], 'net'):
        for wire in children(net, 'wire'):
            path = children(wire, 'path')[0]
            assert path[1] in ('F.Cu', 'B.Cu')
            layer = p.F_Cu if path[1] == 'F.Cu' else p.B_Cu
            width = float(path[2]) / divisor
            points = [(float(path[i])/divisor, -float(path[i+1])/divisor) for i in range(3, len(path), 2)]
            for a, z in zip(points, points[1:]):
                track = p.PCB_TRACK(b); track.SetStart(vec(*a)); track.SetEnd(vec(*z))
                track.SetWidth(p.FromMM(width)); track.SetLayer(layer); track.SetNet(nets[net[1]]); b.Add(track)
        for item in children(net, 'via'):
            assert item[1] == 'Via[0-1]_600:300_um'
            via = p.PCB_VIA(b); via.SetPosition(vec(float(item[2])/divisor, -float(item[3])/divisor))
            via.SetWidth(p.FromMM(0.6)); via.SetDrill(p.FromMM(0.3)); via.SetViaType(p.VIATYPE_THROUGH)
            via.SetLayerPair(p.F_Cu, p.B_Cu); via.SetNet(nets[net[1]]); b.Add(via)
    count = verify_netlist(b)
    plane = p.ZONE(b); plane.SetLayer(p.B_Cu); plane.SetNet(nets['GND'])
    plane.SetLocalClearance(p.FromMM(0.25)); plane.SetThermalReliefGap(p.FromMM(0.3))
    plane.SetThermalReliefSpokeWidth(p.FromMM(0.3)); plane.SetMinThickness(p.FromMM(0.25))
    outline = plane.Outline(); outline.NewOutline()
    for x,y in ((0.5,0.5),(BOARD_WIDTH-0.5,0.5),(BOARD_WIDTH-0.5,BOARD_HEIGHT-0.5),(0.5,BOARD_HEIGHT-0.5)):
        outline.Append(p.FromMM(x),p.FromMM(y))
    b.Add(plane)
    p.ZONE_FILLER(b).Fill(b.Zones())
    board_path = ROOT / (NAME + '.kicad_pcb')
    b.SetFileName(str(board_path))
    p.SaveBoard(str(board_path),b)
    p.WriteDRCReport(b, str(ROOT / 'DRC.txt'), p.EDA_UNITS_MILLIMETRES, True)
    (ROOT / (NAME + '.ses')).write_text(args.session.read_text())
    print(f'Schematic/PCB/net definition agree for {count} connected pads. Wrote {board_path.name}')


if __name__ == '__main__':
    main()
