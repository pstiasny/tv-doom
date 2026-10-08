#!/usr/bin/env python3
"""blediag.py -- windowed log sampler for the notify-storm firmware.

Holds the connection (suppresses the interfering advertisement) but only
subscribes in 3 s bursts every ~13 s: the board's log ring fills quietly
between bursts (pad connect attempts happen in clean air) and each burst
flushes it here. The storm noise is filtered from the output.

Run:  .venv/bin/python tools/blediag.py
Then work the pad during the QUIET phases; paste the gamepad/NimBLE lines.
"""
import asyncio, sys
from bleak import BleakClient, BleakScanner

NAME = "tvdoom-log"
CHR  = "b0766711-4e1c-369f-8b4a-0cd27a43519e"
NOISE = ("initiated: notify", "att_handle=3")

def on_log(_, data: bytearray):
    for line in data.decode("utf-8", "replace").splitlines(True):
        if not any(n in line for n in NOISE) and line.strip():
            sys.stdout.write(line)
    sys.stdout.flush()

async def run():
    while True:
        print("[diag] scanning...", file=sys.stderr)
        dev = await BleakScanner.find_device_by_name(NAME, timeout=15)
        if not dev:
            continue
        try:
            async with BleakClient(dev) as cl:
                print("[diag] connected; sampling 3s bursts -- work the pad "
                      "during QUIET phases", file=sys.stderr)
                while cl.is_connected:
                    await cl.start_notify(CHR, on_log)
                    await asyncio.sleep(3)
                    await cl.stop_notify(CHR)
                    print("[diag] --- quiet phase (10 s): try the pad NOW ---",
                          file=sys.stderr)
                    await asyncio.sleep(10)
        except Exception as e:
            print(f"[diag] {e}; reconnecting", file=sys.stderr)
        await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
