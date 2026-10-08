#!/usr/bin/env python3
"""flashwad.py -- write a WAD into its flash partition, offset read from partitions.csv.

The WAD lives in a raw data partition that the firmware mmaps at runtime, so it
is flashed separately from the app and survives `idf.py flash`. Rather than
hardcoding the offset (it moves whenever the partition table is reshuffled),
this reads partitions.csv and refuses to write a file that would overrun the
partition.

Usage:
  tools/flashwad.py -p /dev/cu.usbmodemXXX                    # -> the 'wad' partition
  tools/flashwad.py -p /dev/cu.usbmodemXXX wads/doom2.wad
  tools/flashwad.py -p /dev/cu.usbmodemXXX --dry-run          # print the command only
  tools/flashwad.py --list                                    # parsed table, no port needed

Needs esptool on the path -- source $IDF_PATH/export.sh first.
"""
import argparse, os, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_WAD = os.path.join(ROOT, "wads", "doom.wad")
DEFAULT_PARTITIONS = os.path.join(ROOT, "partitions.csv")
CHIP = "esp32s3"
BAUD = 921600

# Offsets default to "packed after the previous entry", aligned per type.
ALIGN = {"app": 0x10000, "data": 0x1000}


def parse_size(tok):
    """Accept 0x1000, 4096, 16K, 2M -- the forms esp-idf's gen_esp32part.py takes."""
    tok = tok.strip()
    mult = {"K": 1024, "M": 1024 * 1024}.get(tok[-1:].upper())
    if mult:
        return int(tok[:-1], 0) * mult
    return int(tok, 0)


def parse_partitions(path):
    parts, cursor = [], 0
    with open(path) as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            fields = [c.strip() for c in line.split(",")]
            if len(fields) < 5:
                sys.exit(f"{path}:{lineno}: expected 5 columns, got {len(fields)}")
            name, ptype, subtype, offset, size = fields[:5]
            try:
                size = parse_size(size)
                if offset:
                    offset = parse_size(offset)
                else:
                    align = ALIGN.get(ptype, 0x1000)
                    offset = (cursor + align - 1) // align * align
            except ValueError as e:
                sys.exit(f"{path}:{lineno}: {e}")
            cursor = offset + size
            parts.append({"name": name, "type": ptype, "subtype": subtype,
                          "offset": offset, "size": size})
    return parts


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("wad", nargs="?", default=DEFAULT_WAD)
    ap.add_argument("--partition", default="wad", help="partition name (default: wad)")
    ap.add_argument("--partitions", default=DEFAULT_PARTITIONS)
    ap.add_argument("-p", "--port",
                    help="serial port, e.g. /dev/cu.usbmodem11101 (required "
                         "unless --list); never guessed")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--list", action="store_true", help="print the table and exit")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    # --list only reads the CSV, so it is the one mode that needs no hardware.
    if not args.port and not args.list:
        ap.error("the following arguments are required: -p/--port")

    parts = parse_partitions(args.partitions)

    if args.list:
        print(f"{'name':10} {'type':6} {'subtype':8} {'offset':>10} {'size':>10}")
        for p in parts:
            print(f"{p['name']:10} {p['type']:6} {p['subtype']:8} "
                  f"0x{p['offset']:08x} 0x{p['size']:08x}")
        return

    target = next((p for p in parts if p["name"] == args.partition), None)
    if not target:
        sys.exit(f"no partition named '{args.partition}' in {args.partitions} "
                 f"(have: {', '.join(p['name'] for p in parts)})")

    if not os.path.exists(args.wad):
        sys.exit(f"{args.wad}: not found")
    nbytes = os.path.getsize(os.path.realpath(args.wad))
    if nbytes > target["size"]:
        sys.exit(f"{args.wad} is {nbytes} bytes, partition '{target['name']}' holds "
                 f"only {target['size']} (0x{target['size']:x}) -- grow it first")

    cmd = [sys.executable, "-m", "esptool", "--chip", CHIP, "-p", args.port,
           "-b", str(args.baud), "--before", "default-reset", "--after", "hard-reset",
           "write-flash", hex(target["offset"]), args.wad]

    print(f"{os.path.basename(args.wad)}: {nbytes/1024/1024:.1f} MB -> "
          f"'{target['name']}' @ 0x{target['offset']:x} "
          f"({100*nbytes/target['size']:.0f}% of 0x{target['size']:x}) on {args.port}")
    if args.dry_run:
        print(" ".join(cmd))
        return

    # esptool streams its own progress; let it inherit stdout.
    sys.exit(subprocess.call(cmd))


if __name__ == "__main__":
    main()
