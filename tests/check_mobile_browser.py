"""Exercise the actual mobile page in Chromium against a simulated ESP32 API.

Requires playwright and its Chromium browser. No ESP32 or motor is actuated.
Writes a mock-telemetry preview to artifacts/mobile/joystick-preview.png.
"""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from urllib.parse import parse_qs

from playwright.sync_api import sync_playwright

REPO = Path(__file__).resolve().parents[1]


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *_):
        pass


def main():
    server = ThreadingHTTPServer(("127.0.0.1", 0), partial(QuietHandler, directory=str(REPO / "firmware/kenny_mobile/web")))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    state = dict(armed=False, estop=False, ready=True, faults=0, boot=42, uptime=12000,
                 wiring="Carrier PCB / MCP23017", v=0, w=0, cpr=1320,
                 maxV=0.1, maxW=0.6, watchdog=300, counts=[0]*4, rpm=[0]*4, pwm=[0]*4)
    requests = []
    reject_drive = False
    active_lease = 0
    hold_next_drive = False
    held_route = None

    def api(route):
        nonlocal active_lease, hold_next_drive, held_route
        path = route.request.url.split("/api/")[-1]
        body = parse_qs(route.request.post_data or "")
        requests.append((path, body))
        code, result = 200, {"ok": True}
        if path == "status":
            result = state
        elif path == "arm":
            if state["armed"] or state["estop"] or not state["ready"]:
                code, result = 409, {"ok": False, "error": "Controller unavailable"}
            else:
                state["armed"] = True
                active_lease += 1
                result = {"ok": True, "lease": active_lease}
        elif path == "drive":
            if reject_drive or not state["armed"] or int(body["lease"][0]) != active_lease:
                code, result = 409, {"ok": False}
            else:
                state["v"] = float(body["v"][0]); state["w"] = float(body["w"][0])
                if hold_next_drive and (state['v'] or state['w']):
                    hold_next_drive = False
                    held_route = route
                    return
        elif path == "stop":
            state["armed"] = False; state["v"] = state["w"] = 0
        route.fulfill(status=code, content_type="application/json", body=json.dumps(result))

    try:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch()
            context = browser.new_context(viewport={"width":390,"height":844}, is_mobile=True, has_touch=True)
            context.route("**/api/*", api)
            page = context.new_page()
            errors = []
            page.on("pageerror", lambda error: errors.append(str(error)))
            page.goto(f"http://127.0.0.1:{server.server_port}/")
            page.wait_for_function("document.getElementById('connection').textContent.includes('Connected')")
            assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
            assert not page.get_by_role("button", name="Arm controller").is_disabled()
            assert not any(path == "drive" for path, _ in requests)
            output = REPO / "artifacts/mobile"
            output.mkdir(parents=True, exist_ok=True)
            page.screenshot(path=str(output / "joystick-preview.png"), full_page=True)

            def arm():
                page.wait_for_function("!document.getElementById('arm').disabled")
                page.locator("#arm").click()
                page.wait_for_function("document.getElementById('mode').textContent === 'READY'")

            arm()
            box = page.locator("#pad").bounding_box()
            cx, cy = box["x"] + box["width"]/2, box["y"] + box["height"]/2
            touch = context.new_cdp_session(page)
            touch.send('Input.dispatchTouchEvent', {'type':'touchStart','touchPoints':[{'x':cx,'y':cy,'id':1}]})
            touch.send('Input.dispatchTouchEvent', {'type':'touchMove','touchPoints':[{'x':cx,'y':cy-70,'id':1}]})
            page.wait_for_function("parseFloat(document.getElementById('linear').textContent) > 0")
            page.wait_for_timeout(180)
            assert any(path == "drive" and float(body["v"][0]) > 0 for path, body in requests)
            touch.send('Input.dispatchTouchEvent', {'type':'touchEnd','touchPoints':[]})
            page.wait_for_timeout(160)
            assert state["v"] == 0 and state["w"] == 0

            hold_next_drive = True
            page.mouse.move(cx,cy); page.mouse.down(); page.mouse.move(cx,cy-70)
            page.wait_for_timeout(90)
            assert held_route is not None
            page.mouse.up(); page.wait_for_timeout(50)
            assert state['v'] == 0 and state['w'] == 0  # Release bypasses an outstanding drive request.
            held_route.fulfill(status=409,content_type='application/json',body='{"ok":false}')
            page.wait_for_timeout(80)
            assert page.locator('#mode').inner_text() == 'READY'

            page.mouse.move(cx,cy); page.mouse.down(); page.mouse.move(cx-65,cy)
            page.wait_for_timeout(180)
            assert state["w"] > 0  # Left drag -> positive turn rate.
            page.mouse.up(); page.wait_for_timeout(120)
            page.mouse.move(cx,cy); page.mouse.down(); page.mouse.move(cx,cy+70)
            page.wait_for_timeout(180)
            assert state["v"] < 0
            page.mouse.up()
            page.locator("#stop").click()
            page.wait_for_timeout(120)
            assert not state["armed"]
            assert page.locator("#mode").inner_text() == "DISARMED"

            arm()
            page.evaluate("window.dispatchEvent(new Event('blur'))")
            page.wait_for_timeout(160)
            assert not state["armed"]
            arm()
            reject_drive = True
            page.wait_for_function("document.getElementById('mode').textContent === 'DISARMED'")
            page.wait_for_timeout(150)
            assert not state["armed"]
            reject_drive = False
            state["estop"] = True; state["faults"] = 2
            page.wait_for_function("document.getElementById('estop').textContent.includes('OPEN')")
            assert page.locator("#arm").is_disabled()
            assert not errors, errors
            browser.close()
            print("Browser checks passed: phone layout, touch drag, neutral startup, forward/reverse/turn, release-zero with an outstanding request, STOP, blur, rejected command, E-stop lockout; no JS errors")
    finally:
        server.shutdown(); server.server_close()


if __name__ == "__main__":
    main()
