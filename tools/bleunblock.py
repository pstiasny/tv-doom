#!/usr/bin/env python3
"""bleunblock.py -- hold a bare connection to tvdoom-log WITHOUT subscribing.

Rescue tool for the notify-storm firmware (2026-09-14): a subscribed client
triggers a log->notify->log feedback loop that exhausts NimBLE's mbufs and
starves the pad's connect (rc=6 ENOMEM). A bare connection still stops the
advertising (which also interferes with the pad) but starts no storm.

Run this, wait for HOLDING, then connect the pad and press the Xbox button.
"""
import asyncio, sys
from bleak import BleakClient, BleakScanner

async def run():
    while True:
        print("[unblock] scanning for tvdoom-log...", file=sys.stderr)
        dev = await BleakScanner.find_device_by_name("tvdoom-log", timeout=15)
        if not dev:
            continue
        try:
            async with BleakClient(dev) as cl:
                print("[unblock] HOLDING bare connection (no subscribe). "
                      "Now connect the pad, then press the Xbox button.",
                      file=sys.stderr)
                while cl.is_connected:
                    await asyncio.sleep(1)
        except Exception as e:
            print(f"[unblock] {e}; retrying", file=sys.stderr)
        await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
