"""Verify the phone scanner against independently generated OpenCV markers.

Run in the Playwright environment; system python3 supplies OpenCV/numpy.
No camera, ESP32 or motor hardware is actuated.
"""
from functools import partial
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
import json
from pathlib import Path
import re
import subprocess
import threading

from playwright.sync_api import sync_playwright

REPO = Path(__file__).resolve().parents[1]


def marker_image(bits=4, marker_id=23, rotation=0, blank=False):
    code = f"""
import cv2, numpy as np, sys
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_{bits}X{bits}_1000)
marker = cv2.aruco.drawMarker(d, {marker_id}, 240)
marker = np.rot90(marker, {rotation})
image = np.full((800, 1000), 255, np.uint8)
if not {blank!r}: image[200:440, 300:540] = marker
sys.stdout.buffer.write(cv2.imencode('.png', image)[1].tobytes())
"""
    return subprocess.check_output(["python3", "-c", code])


def main():
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(SimpleHTTPRequestHandler,
                                 directory=str(REPO / 'firmware/kenny_mobile/web')))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch()
            page = browser.new_page(viewport={'width':390, 'height':844})
            errors = []
            page.on('pageerror', lambda error: errors.append(str(error)))
            status = dict(ready=True, estop=False, armed=False, boot=1, uptime=0,
                          wiring='Test', watchdog=300, cpr=1320, faults=0,
                          rpm=[0]*4, counts=[0]*4, pwm=[0]*4)
            page.route('**/api/*', lambda route: route.fulfill(
                content_type='application/json', body=json.dumps(
                    status if route.request.url.endswith('/status') else {'ok':True})))
            page.goto(f'http://127.0.0.1:{server.server_port}/')
            page.wait_for_function("typeof AR !== 'undefined'")
            assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')

            def upload(data):
                page.locator('#markerPhoto').set_input_files(
                    dict(name='marker.png', mimeType='image/png', buffer=data))
                page.wait_for_function("document.getElementById('scanStatus').textContent.includes('Source')")

            for bits in (4, 5, 6):
                for rotation in (0, 1, 2, 3):
                    page.select_option('#dictionary', f'{bits},50')
                    upload(marker_image(bits, rotation=rotation))
                    result = page.locator('#markerResults').inner_text()
                    assert 'ID / value: 23' in result, result
                    center = re.search(r'Center: \(([\d.]+), ([\d.]+)\)', result)
                    assert center and abs(float(center[1])-420) < 5 and abs(float(center[2])-320) < 5, result
                    assert 'Source 1000 × 800 px' in page.locator('#scanStatus').inner_text()
                    assert page.locator('.marker-result').count() == 1

            page.select_option('#dictionary', '4,50')
            upload(marker_image(marker_id=0))
            assert 'ID / value: 0' in page.locator('#markerResults').inner_text()
            upload(marker_image(blank=True))
            assert page.locator('.marker-result').count() == 0
            assert 'No markers detected' in page.locator('#scanStatus').inner_text()
            upload(marker_image(marker_id=80))
            assert page.locator('.marker-result').count() == 0  # Outside selected 50-ID dictionary.
            page.select_option('#dictionary', '4,100')
            upload(marker_image(marker_id=80))
            assert 'ID / value: 80' in page.locator('#markerResults').inner_text()

            # Feed real video frames through getUserMedia's interface, not a mock detector.
            page.evaluate("""() => {
              window.scanFeed=document.createElement('canvas');
              scanFeed.width=640;scanFeed.height=480;
              const c=scanFeed.getContext('2d'); c.fillStyle='white';c.fillRect(0,0,640,480);
              navigator.mediaDevices.getUserMedia=async () => {
                window.testStream=scanFeed.captureStream(10);
                window.feedTimer=setInterval(()=>c.fillRect(0,0,640,480),100);
                return testStream;
              };
            }""")
            page.locator('#cameraStart').click()
            page.wait_for_function("document.getElementById('scanStatus').textContent.includes('Source 640')")
            assert page.locator('#arm').is_disabled()
            page.locator('#cameraStop').click()
            assert page.evaluate("testStream.getTracks().every(t=>t.readyState==='ended')")
            page.evaluate('clearInterval(feedTimer)')
            page.wait_for_function("!document.getElementById('arm').disabled")

            # Exercise the actual single embedded firmware page on its insecure HTTP origin.
            header = (REPO / 'firmware/kenny_mobile/web_ui.h').read_text()
            embedded = header.split('R"KENNYHTML(', 1)[1].rsplit(')KENNYHTML";', 1)[0]
            page.route('http://192.168.4.1/', lambda route: route.fulfill(
                content_type='text/html', body=embedded))
            page.goto('http://192.168.4.1/')
            assert not page.evaluate('window.isSecureContext')
            page.locator('#cameraStart').click()
            assert 'Live camera requires HTTPS' in page.locator('#scanStatus').inner_text()
            upload(marker_image())
            assert 'ID / value: 23' in page.locator('#markerResults').inner_text()
            assert not errors, errors
            browser.close()
            print('ArUco checks passed: OpenCV IDs, rotations, source coordinates, blank images, dictionary bounds and camera cleanup')
    finally:
        server.shutdown()


if __name__ == '__main__':
    main()
