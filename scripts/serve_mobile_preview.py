"""Serve a phone-accessible scanner preview with simulated controller telemetry."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path

WEB = Path(__file__).resolve().parents[1] / 'firmware/kenny_mobile/web'


class PreviewHandler(SimpleHTTPRequestHandler):
    def reply(self, data, code=200):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/api/status':
            self.reply(dict(ready=False, estop=False, armed=False, boot=1, uptime=0,
                            wiring='PHONE TEST PREVIEW · no robot connected',
                            watchdog=300, cpr=1320, faults=0,
                            rpm=[0]*4, counts=[0]*4, pwm=[0]*4))
        else:
            super().do_GET()

    def do_POST(self):
        if self.path == '/api/stop':
            self.reply({'ok': True})
        else:
            self.reply({'ok': False, 'error': 'Scanner preview: no robot connected'}, 409)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()
    server = ThreadingHTTPServer(('0.0.0.0', args.port), partial(PreviewHandler, directory=str(WEB)))
    print(f'Mobile scanner preview listening on port {args.port}', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
