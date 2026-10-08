#!/usr/bin/env python3
"""blectl.py -- send a command to the running game over BLE (no pad, no UART).

  tools/blectl.py reboot
  tools/blectl.py loader                       # just enter the menu
  tools/blectl.py "loader do=update"           # menu -> join saved wifi -> pull+flash
  tools/blectl.py "loader ssid=Net pass=pw url=http://host:8000/x.bin do=update"

The full unattended debug flow: one BLE write stages wifi/url/command in NVS,
the board reboots into the loader, joins, executes, and comes back up.
"""
import asyncio, sys
from bleak import BleakClient, BleakScanner

NAME = "tvdoom-log"
CMD  = "b0766712-4e1c-369f-8b4a-0cd27a43519e"

async def run(cmd):
    import asyncio
    for attempt in range(5):
        print(f"[blectl] scanning for {NAME}...", file=sys.stderr)
        dev = await BleakScanner.find_device_by_name(NAME, timeout=20)
        if not dev:
            continue
        try:
            await _send(dev, cmd); return
        except Exception as e:
            print(f"[blectl] attempt {attempt}: {e}; retrying", file=sys.stderr)
            await asyncio.sleep(2)
    sys.exit("could not deliver command")

async def _send(dev, cmd):
    import asyncio
    async with BleakClient(dev) as cl:
        # Settle: let the board's pad scanner notice the connection and defer
        # its scan bursts (otherwise they starve the link before our write).
        await asyncio.sleep(3)
        if not cl.is_connected:
            raise RuntimeError("dropped during settle")
        await cl.write_gatt_char(CMD, cmd.encode() + b"\0", response=True)
        print(f"[blectl] sent {cmd!r}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    asyncio.run(run(" ".join(sys.argv[1:])))
