"""Export review drawings/Gerbers and a provisional design archive."""
from pathlib import Path
import subprocess
import zipfile
from generate import ROOT, NAME


def run(*args):
    subprocess.run(args, check=True)


def main():
    board = str(ROOT / (NAME + '.kicad_pcb'))
    schematic = str(ROOT / (NAME + '.kicad_sch'))
    run('python3', str(ROOT / 'verify.py'))
    run('kicad-cli', 'sch', 'export', 'pdf', '-o', str(ROOT / 'schematic.pdf'), schematic)
    for name, layers in (('board-top', 'F.Cu,F.Silkscreen,Edge.Cuts'),
                         ('board-bottom', 'B.Cu,Edge.Cuts'),
                         ('assembly', 'F.Fab,Edge.Cuts'),
                         ('socket-fit-1to1', 'F.Cu,Edge.Cuts')):
        run('kicad-cli', 'pcb', 'export', 'svg', '--layers', layers, '--page-size-mode', '2',
            '--exclude-drawing-sheet', '-o', str(ROOT / (name + '.svg')), board)
    run('rsvg-convert', '-w', '1400', '-o', str(ROOT / 'board-top.png'), str(ROOT / 'board-top.svg'))
    # Same geometry as the KiCad top-layer plot, with a readable PCB-colored background.
    # This is a layout illustration, not a simulated manufactured/assembled board.
    preview = (ROOT / 'board-top.svg').read_text()
    marker = '</desc>'
    if marker not in preview:
        raise RuntimeError('Unexpected KiCad SVG header')
    from generate import BOARD_WIDTH, BOARD_HEIGHT
    preview = preview.replace(marker, marker + f'\n<rect x="0" y="0" width="{BOARD_WIDTH}" height="{BOARD_HEIGHT}" fill="#15553c"/>', 1)
    preview = preview.replace('#C83434', '#ceaa60').replace('#F0F0F0', '#f5fff6').replace('#840000', '#ceaa60')
    (ROOT / 'board-preview.svg').write_text(preview)
    run('rsvg-convert', '-w', '1400', '-o', str(ROOT / 'board-preview.png'), str(ROOT / 'board-preview.svg'))
    run('rsvg-convert', '-f', 'pdf', '-o', str(ROOT / 'socket-fit-1to1.pdf'), str(ROOT / 'socket-fit-1to1.svg'))
    run('rsvg-convert', '-f', 'pdf', '-o', str(ROOT / 'assembly.pdf'), str(ROOT / 'assembly.svg'))
    gerbers = ROOT / 'review_gerbers'
    gerbers.mkdir(exist_ok=True)
    (gerbers / 'PROVISIONAL-NOT-FABRICATION-APPROVED.txt').write_text(
        'Review exports only. Verify actual ESP32 38-pin footprint and pin order,\n'
        'sensor module supplies/pinouts and electrical rules before manufacture.\n'
        'See ../README.md and validation.json. Existing direct-wire firmware does\n'
        'not match this carrier. No motor power may be connected to this PCB.\n')
    run('kicad-cli', 'pcb', 'export', 'gerbers', '--layers',
        'F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts',
        '-o', str(gerbers) + '/', board)
    run('kicad-cli', 'pcb', 'export', 'drill', '--excellon-separate-th', '--generate-map',
        '-o', str(gerbers) + '/', board)
    archive_path = ROOT / 'Kenny-PCB-RevA-PROVISIONAL.zip'
    allowed = {'.kicad_pcb', '.kicad_sch', '.kicad_pro', '.kicad_mod', '.pdf', '.png', '.svg',
               '.csv', '.json', '.md', '.py', '.xml', '.dsn', '.ses', '.txt'}
    with zipfile.ZipFile(archive_path, 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(ROOT.rglob('*')):
            if not path.is_file() or '__pycache__' in path.parts or '-backups' in str(path):
                continue
            if path.suffix in allowed or path.name == 'fp-lib-table' or path.parent == gerbers:
                archive.write(path, 'Kenny-PCB-RevA-PROVISIONAL/' + str(path.relative_to(ROOT)))
    desktop = Path.home() / 'Desktop'
    if desktop.is_dir():
        (desktop / archive_path.name).write_bytes(archive_path.read_bytes())
        (desktop / 'Kenny-PCB-Preview.png').write_bytes((ROOT / 'board-preview.png').read_bytes())
        print('Desktop archive:', desktop / archive_path.name)
    print('Project archive:', archive_path)


if __name__ == '__main__':
    main()
