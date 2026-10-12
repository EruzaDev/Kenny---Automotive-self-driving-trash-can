"""Package the mobile source and guide for copying to Arduino IDE."""
from pathlib import Path
import shutil
from zipfile import ZipFile, ZIP_DEFLATED

repo = Path(__file__).resolve().parents[1]
sketch = repo / "firmware/kenny_mobile"
page = (sketch / "web/index.html").read_text()
import re
embedded = re.sub(r'<script src="([^"]+)"></script>',
                  lambda match: '<script>\n' + (sketch / 'web' / match[1]).read_text() + '\n</script>', page)
assert embedded in (sketch / "web_ui.h").read_text(), "Run scripts/build_mobile_ui.py before packaging"
output = repo / "artifacts/mobile"
output.mkdir(parents=True, exist_ok=True)
archive = output / "Kenny-ESP32-Mobile-Controller.zip"
with ZipFile(archive, "w", ZIP_DEFLATED) as package:
    for path in sorted(sketch.rglob("*")):
        if path.is_file() and "__pycache__" not in path.parts:
            package.write(path, "Kenny-ESP32-Mobile-Controller/firmware/kenny_mobile/" + str(path.relative_to(sketch)))
    package.write(repo / "docs/ESP32_MOBILE_CONTROLLER.md", "Kenny-ESP32-Mobile-Controller/SETUP.md")
    for relative in ('docs/ESP32_MOBILE_CONTROLLER.md', 'docs/ESP32_FIRMWARE.md',
                     'hardware/kenny_carrier/README.md', 'scripts/build_mobile_ui.py',
                      'tests/mobile_control_test.cpp', 'tests/check_mobile_browser.py',
                      'tests/check_mobile_aruco.py', 'scripts/vendor_mobile_aruco.py'):
        package.write(repo / relative, 'Kenny-ESP32-Mobile-Controller/' + relative)
    preview = output / "joystick-preview.png"
    if preview.exists():
        package.write(preview, "Kenny-ESP32-Mobile-Controller/joystick-preview-MOCK-TELEMETRY.png")
with ZipFile(archive) as package:
    assert package.testzip() is None
desktop = Path.home() / "Desktop"
if desktop.is_dir():
    shutil.copyfile(archive, desktop / archive.name)
    if preview.exists():
        shutil.copyfile(preview, desktop / "Kenny-Mobile-Joystick-Preview.png")
print("Packaged:", archive)
