#!/usr/bin/env python3
"""ota.py -- bench image server + one-shot remote control for the board's OTA loader.

Serves ESP app images over HTTP on :8000 with an index the loader menu reads:
DOWNLOAD IMAGE GETs /index.txt, shows one line per image and pulls the chosen
one into ota_0 (a loader image then copies itself into the factory slot on its
first boot).

  tools/ota.py                     serve: images in build/ and ota_loader/build/
  tools/ota.py DIR-OR-FILE ...     serve these instead (a directory means its *.bin)
  tools/ota.py cmd "update build/tv-doom.bin"     queue a command for the board
  tools/ota.py cmd --show / --clear               inspect / drop the queued command

The index is built per request, so a rebuilt or newly copied image shows up
without restarting the server. Only real app images are listed (ESP image
magic byte 0xE9), which keeps partition tables and otadata blobs out.

Index format: one line per image, "<path>|<label>". <path> is relative to the
repo root, which is also the HTTP root; <label> is at most 25 characters for
the loader's 26-column display.

REMOTE CONTROL (/cmd). With the loader's SETTINGS -> SERVER RC on, the board
GETs /cmd the moment its WiFi comes up (every boot, or a manual join) and
executes the reply. The queue holds ONE command and hands it out exactly
once: the GET that delivers it also clears it, so no later boot can replay it
(a failed update cannot loop). An unclaimed command expires after --ttl
seconds (default 600) so a forgotten one can't fire a day later. Every
delivery is logged here with the board's address. Commands the loader runs:
  update <path|url>   pull that image into ota_0 and reboot into it (paths as
                      in the index, e.g. build/tv-doom.bin)
  run                 boot the game
  reboot              restart the loader
  set <key> <value>   snd|lsleep|bledbg|srvrc on|off,
                      carmode|cardrv|tcard|bletx <index>
                      bletx: BLE transmit power, 0 = +9 dBm (default) .. 5 = -21 dBm
  testcard <index>    set TEST CARD and boot the game into the RF test-card
                      generator (0 = plain game, 1 = cycle all cards)
`cmd` talks to the running server (--server host:port, default 127.0.0.1:8000).
"""
import argparse, http.server, os, socket, sys, time, urllib.error, urllib.parse, urllib.request
from datetime import datetime

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT = ["build", "ota_loader/build"]
ARGS = []                                  # image dirs/files (serve mode)

# ---------------------------------------------------------------- images

def is_app_image(path):
    try:
        with open(path, "rb") as f:
            return f.read(1) == b"\xe9"
    except OSError:
        return False

def images(args):
    """Repo-relative paths of the app images to serve, in argument order."""
    out = []
    for a in args or DEFAULT:
        p = os.path.abspath(os.path.join(ROOT, a))
        if os.path.isdir(p):
            files = sorted(os.path.join(p, f) for f in os.listdir(p) if f.endswith(".bin"))
        else:
            files = [p]
        for f in files:
            rel = os.path.relpath(f, ROOT)
            if rel.startswith("..") or not is_app_image(f):
                continue
            out.append(rel)
    return out

def label(rel):
    st = os.stat(os.path.join(ROOT, rel))
    name = os.path.splitext(os.path.basename(rel))[0]
    return f"{name[:12]:<12}{st.st_size // 1024:>5}K {datetime.fromtimestamp(st.st_mtime):%H:%M}"

def index_text():
    return "".join(f"{rel}|{label(rel)}\n" for rel in images(ARGS))

# ---------------------------------------------------------------- one-shot command queue

VERBS = {"update": (1, 1), "run": (0, 0), "reboot": (0, 0), "set": (2, 2),
         "testcard": (1, 1)}   # arg count min,max
SET_KEYS = {"snd": "onoff", "lsleep": "onoff", "bledbg": "onoff", "srvrc": "onoff",
            "carmode": "index", "cardrv": "index", "tcard": "index",
            "bletx": "index"}

def check_command(text):
    """None if the loader will accept it, else the reason it would not."""
    words = text.split()
    if not words:
        return "empty command"
    verb, args = words[0], words[1:]
    if verb not in VERBS:
        return f"unknown verb {verb!r} (one of {', '.join(VERBS)})"
    lo, hi = VERBS[verb]
    if not lo <= len(args) <= hi:
        return f"{verb} takes {lo} argument(s)"
    if verb == "update" and not args[0].startswith("http://"):
        p = os.path.join(ROOT, args[0])
        if not os.path.isfile(p):
            return f"no such file under the server root: {args[0]}"
        if not is_app_image(p):
            return f"not an ESP app image: {args[0]}"
    if verb == "set":
        kind = SET_KEYS.get(args[0])
        if not kind:
            return f"unknown key {args[0]!r} (one of {', '.join(SET_KEYS)})"
        if kind == "onoff" and args[1] not in ("on", "off", "0", "1"):
            return f"{args[0]} takes on|off"
        if kind == "index" and not args[1].isdigit():
            return f"{args[0]} takes an index"
    if verb == "testcard" and not args[0].isdigit():
        return "testcard takes an index (0 = off, 1 = cycle, 2.. = one card)"
    if len(text) > 90:
        return "command longer than the loader's 95-byte buffer"
    return None

class CommandQueue:
    """One pending command, handed out once."""
    def __init__(self):
        self.cmd = None
        self.expires = 0.0

    def put(self, cmd, ttl):
        if self.cmd:
            log(f"replacing queued command {self.cmd!r}")
        self.cmd = cmd
        self.expires = time.time() + ttl if ttl > 0 else float("inf")

    def peek(self):
        if self.cmd and time.time() > self.expires:
            log(f"queued command {self.cmd!r} expired unclaimed")
            self.cmd = None
        return self.cmd

    def take(self):
        cmd = self.peek()
        self.cmd = None
        return cmd

    def clear(self):
        self.cmd = None

QUEUE = CommandQueue()

def log(msg):
    print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)

# ---------------------------------------------------------------- HTTP

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=ROOT, **kw)

    def reply(self, code, body=""):
        data = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path, _, query = self.path.partition("?")
        q = urllib.parse.parse_qs(query)
        if path == "/index.txt":
            return self.reply(200, index_text())
        if path == "/cmd":
            if "peek" in q:                      # CLI --show: look, don't consume
                return self.reply(200, (QUEUE.peek() or "") + "\n")
            cmd = QUEUE.take()                   # the board: delivered exactly once
            who = self.client_address[0]
            if cmd:
                log(f"/cmd -> {who}: {cmd!r}  (delivered, queue now empty)")
                return self.reply(200, cmd + "\n")
            log(f"/cmd -> {who}: nothing queued")
            return self.reply(200, "")
        return super().do_GET()

    def do_POST(self):
        path, _, query = self.path.partition("?")
        if path != "/cmd":
            return self.reply(404, "not found\n")
        n = int(self.headers.get("Content-Length") or 0)
        cmd = " ".join(self.rfile.read(n).decode(errors="replace").split())
        err = check_command(cmd)
        if err:
            return self.reply(400, err + "\n")
        ttl = float(urllib.parse.parse_qs(query).get("ttl", ["600"])[0])
        QUEUE.put(cmd, ttl)
        log(f"queued {cmd!r}" + (f" for {ttl:.0f} s" if ttl > 0 else " (no expiry)"))
        return self.reply(200, f"queued: {cmd}\n")

    def do_DELETE(self):
        if self.path.partition("?")[0] != "/cmd":
            return self.reply(404, "not found\n")
        old = QUEUE.peek()
        QUEUE.clear()
        log(f"cleared {old!r}" if old else "clear: queue was empty")
        return self.reply(200, "cleared\n")

    def log_message(self, fmt, *args):        # image/index traffic: keep the default log
        if not self.path.startswith("/cmd"):
            super().log_message(fmt, *args)

def my_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 80))
        return s.getsockname()[0]
    finally:
        s.close()

# ---------------------------------------------------------------- CLI

def cmd_main(argv):
    ap = argparse.ArgumentParser(prog="ota.py cmd", description="queue a one-shot command for the board")
    ap.add_argument("command", nargs="*", help="e.g. update build/tv-doom.bin | run | reboot | set snd off")
    ap.add_argument("--show", action="store_true", help="print the queued command without consuming it")
    ap.add_argument("--clear", action="store_true", help="drop the queued command")
    ap.add_argument("--ttl", type=float, default=600, help="seconds until an unclaimed command expires (0 = never)")
    ap.add_argument("--server", default="127.0.0.1:8000")
    a = ap.parse_args(argv)
    url = f"http://{a.server}/cmd"
    try:
        if a.show:
            with urllib.request.urlopen(url + "?peek=1", timeout=3) as r:
                body = r.read().decode().strip()
            print(f"queued: {body!r}" if body else "queue empty")
        elif a.clear:
            req = urllib.request.Request(url, method="DELETE")
            with urllib.request.urlopen(req, timeout=3) as r:
                print(r.read().decode().strip())
        elif a.command:
            text = " ".join(a.command)
            err = check_command(text)
            if err:
                sys.exit(f"rejected: {err}")
            req = urllib.request.Request(f"{url}?ttl={a.ttl:g}", data=text.encode(), method="POST")
            with urllib.request.urlopen(req, timeout=3) as r:
                print(r.read().decode().strip(),
                      f"-- delivered to the board once it connects with SERVER RC on"
                      + (f" (expires in {a.ttl:.0f} s)" if a.ttl > 0 else ""))
        else:
            ap.print_help()
    except urllib.error.HTTPError as e:
        sys.exit(f"server refused: {e.read().decode().strip()}")
    except urllib.error.URLError as e:
        sys.exit(f"no server at {a.server} ({e.reason}) -- start tools/ota.py first")

def serve_main(argv):
    ap = argparse.ArgumentParser(prog="ota.py", description="serve images + the /cmd queue")
    ap.add_argument("paths", nargs="*", help="image dirs/files (default: build/ and ota_loader/build/)")
    ap.add_argument("--port", type=int, default=8000)
    a = ap.parse_args(argv)
    global ARGS
    ARGS = a.paths
    if not images(ARGS):
        sys.exit("no app images found: build the game and/or the loader first")
    print(f"index at http://{my_ip()}:{a.port}/index.txt  (loader menu: DOWNLOAD IMAGE)")
    print(index_text(), end="")
    print(f"remote control: tools/ota.py cmd \"update build/tv-doom.bin\" | run | reboot | set ... "
          f"(loader SETTINGS -> SERVER RC on)", flush=True)
    http.server.HTTPServer(("", a.port), Handler).serve_forever()

if __name__ == "__main__":
    if sys.argv[1:2] == ["cmd"]:
        cmd_main(sys.argv[2:])
    else:
        serve_main(sys.argv[1:])
