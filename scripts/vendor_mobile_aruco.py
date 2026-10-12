"""Fetch pinned js-aruco2 sources and license notices for offline builds."""
from pathlib import Path
from urllib.request import urlopen

REVISION = "0491d5d228746411d0e9dd602b98f48636a644ba"
BASE = f"https://raw.githubusercontent.com/damianofalcioni/js-aruco2/{REVISION}/"
target = Path(__file__).resolve().parents[1] / "firmware/kenny_mobile/web/vendor"
target.mkdir(parents=True, exist_ok=True)
for source, name in (("src/cv.js", "cv.js"), ("src/aruco.js", "aruco.js"),
                     ("src/dictionaries/aruco_4x4_1000.js", "aruco_4x4_1000.js"),
                     ("src/dictionaries/aruco_5x5_1000.js", "aruco_5x5_1000.js"),
                     ("src/dictionaries/aruco_6x6_1000.js", "aruco_6x6_1000.js"),
                     ("LICENSE.txt", "LICENSE.txt")):
    with urlopen(BASE + source, timeout=30) as response:
        (target / name).write_bytes(response.read())
print("Vendored js-aruco2", REVISION)
