#!/usr/bin/env python3
"""perfmon.py -- capture + summarize tv-doom performance stats from serial.

Parses the firmware's periodic log lines (display fps/wait/fill, video encode,
present accounting, gap histogram, input-latency probe, jit) and prints a
one-screen summary, flagging errors/resets. Reusable for A/B runs: run once per
configuration, compare summaries.

Usage:
  tools/perfmon.py                      # 30 s capture, default port
  tools/perfmon.py --secs 60            # longer window (e.g. while mashing fire)
  tools/perfmon.py --raw                # echo every serial line while capturing
  tools/perfmon.py --port /dev/cu.usbmodemXXX
  .venv/bin/python tools/blelog.py | tools/perfmon.py --stdin --secs 60
                                        # tvplug2: USB is dead, feed it the BLE
                                        # log (SETTINGS -> BLE DEBUGGING must be ON)
"""
import argparse, glob, re, sys, time

# "auto" = first /dev/cu.usbmodem* (tvplug2 native USB-Serial-JTAG; the number
# changes with the USB port). The v1 rig's external UART adapter was
# /dev/cu.usbmodem5B7B0552861 -- pass it via --port if scoping that board.
DEFAULT_PORT = "auto"

RE_DISPLAY = re.compile(r"display: (\d+) fps \(wait (\d+) ms, fill (\d+) ms, worst gap (\d+) ms\)")
RE_ENCODE  = re.compile(r"encode avg (\d+) ms")
RE_JIT     = re.compile(r"jit: sleep avg (\d+) ms, budget (\d+) ms")
RE_PRESENT = re.compile(r"present: (\d+) frames / (\d+) refreshes; drops (\d+), "
                        r"repeats game-late (\d+) engine-late (\d+)")
RE_GAPHIST = re.compile(r"gap hist \(([^)]*)\): ([\d ]+)")
RE_INLAT   = re.compile(r"inlat: input->onscreen: min=(\d+) avg=(\d+) max=(\d+) ms \(n=(\d+)\)")
RE_DMA     = re.compile(r"dma: stalls (\d+) \(realigns (\d+)\)")
RE_DMAEVT  = re.compile(r"DMA STALL|DMA recovered")
RE_ERROR   = re.compile(r"Guru Meditation|abort\(\)|assert failed|Backtrace:|rst:0x|^E \(", re.M)

def stats(vals):
    return f"min {min(vals)}  avg {sum(vals)/len(vals):.1f}  max {max(vals)}" if vals else "-"

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--secs", type=float, default=30)
    ap.add_argument("--raw", action="store_true", help="echo every serial line")
    ap.add_argument("--stdin", action="store_true",
                    help="read log lines from stdin (e.g. piped from tools/blelog.py) "
                         "instead of a serial port")
    args = ap.parse_args()

    if args.stdin:
        import select
        args.port = "stdin"
        port = None
        def readline():
            r, _, _ = select.select([sys.stdin], [], [], 1.0)   # 1 s poll, like serial timeout
            return sys.stdin.readline() if r else ""
    else:
        try:
            import serial
        except ImportError:
            sys.exit("pyserial not installed (pip install pyserial)")
        if args.port == "auto":
            ports = sorted(glob.glob("/dev/cu.usbmodem*"))
            if not ports:
                sys.exit("no /dev/cu.usbmodem* found (board unplugged?); use --port")
            args.port = ports[0]
        try:
            port = serial.Serial(args.port, args.baud, timeout=1)
        except Exception as e:
            sys.exit(f"cannot open {args.port}: {e}")
        def readline():
            return port.readline().decode("utf-8", "replace")

    fps, wait, fill, gap, enc, jit_sleep = [], [], [], [], [], []
    inlat = []            # (min, avg, max, n) per logged window
    pres = [0, 0, 0, 0, 0]  # frames, refreshes, drops, game-late, engine-late
    hist_lbl, hist = None, None
    dma = None            # (stalls, realigns) -- counters, keep the last seen
    dma_events = []       # DMA STALL / DMA recovered lines, as they happen
    errors, nlines = [], 0

    print(f"capturing {args.secs:.0f}s from {args.port} "
          f"(mash buttons if you want inlat samples) ...", flush=True)
    end = time.time() + args.secs
    while time.time() < end:
        line = readline().rstrip()
        if not line:
            continue
        nlines += 1
        if args.raw:
            print(line)
        if (m := RE_DISPLAY.search(line)):
            fps.append(int(m[1])); wait.append(int(m[2]))
            fill.append(int(m[3])); gap.append(int(m[4]))
        elif (m := RE_ENCODE.search(line)):
            enc.append(int(m[1]))
        elif (m := RE_JIT.search(line)):
            jit_sleep.append(int(m[1]))
        elif (m := RE_PRESENT.search(line)):
            for i in range(5):
                pres[i] += int(m[i + 1])
        elif (m := RE_GAPHIST.search(line)):
            hist_lbl = m[1]
            buckets = [int(v) for v in m[2].split()]
            hist = [a + b for a, b in zip(hist, buckets)] if hist else buckets
        elif (m := RE_INLAT.search(line)):
            inlat.append(tuple(int(v) for v in m.groups()))
            if not args.raw:
                print("  " + line[line.index("inlat"):])
        elif (m := RE_DMA.search(line)):
            dma = (int(m[1]), int(m[2]))
        if RE_DMAEVT.search(line):
            dma_events.append(line)
            if not args.raw:
                print("  !! " + line)
        if RE_ERROR.search(line):
            errors.append(line)
            if not args.raw:
                print("  !! " + line)
    if port:
        port.close()

    print(f"\n== perfmon summary ({args.secs:.0f}s, {nlines} lines) ==")
    print(f"fps        : {stats(fps)}")
    print(f"fill ms    : {stats(fill)}")
    print(f"wait ms    : {stats(wait)}")
    print(f"worst gap  : {stats(gap)}")
    print(f"encode ms  : {stats(enc)}")
    if jit_sleep:
        print(f"jit sleep  : {stats(jit_sleep)}")
    if inlat:
        n = sum(v[3] for v in inlat)
        avg = sum(v[1] * v[3] for v in inlat) / n
        print(f"inlat ms   : min {min(v[0] for v in inlat)}  avg {avg:.0f}  "
              f"max {max(v[2] for v in inlat)}   ({n} presses)")
    else:
        print("inlat ms   : (no samples -- no button presses seen)")
    if pres[0]:
        print(f"present    : {pres[0]} frames / {pres[1]} refreshes "
              f"({pres[1]/pres[0]:.2f} fields/present), drops {pres[2]}, "
              f"repeats game-late {pres[3]} engine-late {pres[4]}")
    if hist:
        print(f"gap hist   : ({hist_lbl})\n             {' '.join(map(str, hist))}")
    if dma or dma_events:
        stalls, realigns = dma if dma else ("?", "?")
        print(f"dma probe  : stalls {stalls}, realigns {realigns}, "
              f"events this window {len(dma_events)}")
        for e in dma_events[:5]:
            print("             " + e)
    print(f"errors     : {len(errors) or 'none'}")
    for e in errors[:5]:
        print("             " + e)

if __name__ == "__main__":
    main()
