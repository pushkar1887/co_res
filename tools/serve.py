"""Local dev server for the project (PLAN §12).

Serves the repo root at http://127.0.0.1:8000/ and accepts
POST /save?path=results/<folder>/<file>  (body = file text), writing ONLY inside results/.
Usage: python tools/serve.py [port]
"""
import http.server
import re
import sys
from pathlib import Path
from urllib.parse import urlparse, parse_qs

ROOT = Path(__file__).resolve().parent.parent
RESULTS = ROOT / "results"
SAFE = re.compile(r"^results/[A-Za-z0-9_\-]+/[A-Za-z0-9_\-.]+\.(csv|json|txt)$")


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                      ".js": "text/javascript", ".mjs": "text/javascript", ".json": "application/json",
                      ".csv": "text/csv", ".html": "text/html"}

    def __init__(self, *a, **kw):
        super().__init__(*a, directory=str(ROOT), **kw)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_POST(self):
        url = urlparse(self.path)
        rel = parse_qs(url.query).get("path", [""])[0]
        if url.path != "/save" or not SAFE.match(rel):
            self.send_error(400, "only POST /save?path=results/<folder>/<file>.(csv|json|txt)")
            return
        target = (ROOT / rel).resolve()
        if RESULTS.resolve() not in target.parents:
            self.send_error(400, "outside results/")
            return
        n = int(self.headers.get("Content-Length", 0))
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(self.rfile.read(n))
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"saved")


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    print(f"serving {ROOT} on http://127.0.0.1:{port}/")
    http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()
