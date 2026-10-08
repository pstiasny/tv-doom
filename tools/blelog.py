#!/usr/bin/env python3
"""blelog.py -- wireless log console over BLE (the board's USB is dead).

The game advertises "tvdoom-log" with a notify characteristic fed by a
vprintf hook (components/gamepad/blelog.c). This connects, subscribes and
prints the stream: a wireless `idf.py monitor`.

Needs bleak (in the repo venv): .venv/bin/pip install bleak
Run:          .venv/bin/python tools/blelog.py
Ctrl-C to quit; auto-reconnects when the board reboots.
"""
import asyncio, sys
from bleak import BleakClient, BleakScanner

NAME = "tvdoom-log"
CHR  = "b0766711-4e1c-369f-8b4a-0cd27a43519e"

def on_log(_, data: bytearray):
    sys.stdout.write(data.decode("utf-8", "replace"))
    sys.stdout.flush()

async def run():
    while True:
        print(f"[blelog] scanning for {NAME}...", file=sys.stderr)
        dev = await BleakScanner.find_device_by_name(NAME, timeout=15)
        if not dev:
            continue
        try:
            async with BleakClient(dev) as cl:
                print(f"[blelog] connected ({dev.address})", file=sys.stderr)
                await cl.start_notify(CHR, on_log)
                while cl.is_connected:
                    await asyncio.sleep(1)
        except Exception as e:
            print(f"[blelog] {e}; reconnecting", file=sys.stderr)
        await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
