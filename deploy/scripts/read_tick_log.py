#!/usr/bin/env python3
"""Read a per-tick hardware log written by `isaaclab::TickLogger`.

The file is self-describing: field names, offsets, counts and dtypes live in the header, so this
reader does not need to be kept in step with the C++ schema by hand. Adding or reordering a field
in the recorder changes the header, and the arrays come back under their new names.

    python read_tick_log.py tick_20260901_183000.bin --summary
    python read_tick_log.py tick_20260901_183000.bin --npz out.npz

Anything that needs the policy's own view of a run -- the exact 96-d `info`, the depth frame the
encoder was actually handed, the latent, the GRU hidden state, the raw action -- comes from here;
none of it is on any topic a bag can record.
"""

from __future__ import annotations

import argparse
import struct
import sys

import numpy as np

MAGIC = b"PKRTICK\x01"
NAME_BYTES = 24
DTYPES = {0: np.float32, 1: np.float64, 2: np.uint64}
DTYPE_BYTES = {0: 4, 1: 8, 2: 8}


def read(path: str) -> dict[str, np.ndarray]:
    with open(path, "rb") as handle:
        blob = handle.read()

    if blob[:8] != MAGIC:
        raise ValueError(f"{path}: not a tick log (magic {blob[:8]!r})")

    record_bytes, field_count = struct.unpack_from("<II", blob, 8)
    cursor = 16
    fields = []
    for _ in range(field_count):
        name = blob[cursor:cursor + NAME_BYTES].split(b"\0", 1)[0].decode()
        offset, count = struct.unpack_from("<II", blob, cursor + NAME_BYTES)
        dtype = blob[cursor + NAME_BYTES + 8]
        cursor += NAME_BYTES + 9
        fields.append((name, offset, count, dtype))

    declared, dropped = struct.unpack_from("<QQ", blob, cursor)
    cursor += 16

    payload = len(blob) - cursor
    n = payload // record_bytes
    if payload % record_bytes:
        # A run killed mid-write leaves a partial record; keep the whole ones.
        print(f"[warn] {payload % record_bytes} trailing bytes are a partial record, ignored",
              file=sys.stderr)
    if declared and n != declared:
        # The trailer is patched on a clean close, so a mismatch means the process was killed.
        print(f"[warn] header declares {declared} records, file holds {n}", file=sys.stderr)
    if dropped:
        print(f"[warn] {dropped} records were DROPPED during the run: the writer fell behind and "
              f"the tick sequence has gaps. Check the `tick` field for jumps.", file=sys.stderr)

    raw = np.frombuffer(blob, dtype=np.uint8, count=n * record_bytes, offset=cursor)
    raw = raw.reshape(n, record_bytes)

    out: dict[str, np.ndarray] = {}
    for name, offset, count, dtype in fields:
        if count == 0:
            continue
        width = DTYPE_BYTES[dtype] * count
        column = raw[:, offset:offset + width].copy()
        values = column.view(DTYPES[dtype]).reshape(n, count)
        out[name] = values[:, 0] if count == 1 else values
    out["_dropped"] = np.asarray(dropped)
    return out


def summarize(data: dict[str, np.ndarray]) -> None:
    tick = data["tick"].astype(np.int64)
    t = data["t_mono"]
    n = len(tick)
    print(f"records            {n}")
    if n < 2:
        return

    gaps = np.diff(tick)
    print(f"tick range         {tick[0]} .. {tick[-1]}"
          + ("" if (gaps == 1).all() else f"   MISSING: {int((gaps - 1).sum())} ticks in {int((gaps > 1).sum())} gaps"))

    dt = np.diff(t) * 1e3
    print(f"control period     mean {dt.mean():.2f} ms   p50 {np.median(dt):.2f}   "
          f"p99 {np.percentile(dt, 99):.2f}   max {dt.max():.2f}   (target 20.00)")

    # The measurement this log exists for: how old is the image the encoder was handed?
    if "t_camera" in data and np.any(data["t_camera"] > 0):
        cam = data["t_camera"]
        live = cam > 0
        # t_mono is monotonic-since-start, t_camera is an absolute ROS stamp: align by their offset
        # over the run, which is constant unless a clock steps.
        age = (cam[live] - cam[live][0]) - (t[live] - t[live][0])
        age = -age
        print(f"depth frame age    mean {age.mean()*1e3:.1f} ms   p50 {np.median(age)*1e3:.1f}   "
              f"max {age.max()*1e3:.1f}   (fixed buffer expects < ~35 ms; the bug gave 0-580)")
        fresh = np.diff(cam[live])
        held = int((fresh == 0).sum())
        print(f"                   {held}/{len(fresh)} ticks reused the previous camera frame")

    for key in ("info", "image", "latent", "hidden", "action_raw", "action_proc",
                "joint_pos", "joint_vel", "joint_tau"):
        if key not in data:
            continue
        v = data[key]
        finite = np.isfinite(v).all()
        print(f"{key:<18} shape {str(v.shape):<14} "
              f"|.|max {np.abs(v).max():9.4f}   rms {np.sqrt((v.astype(np.float64)**2).mean()):8.4f}"
              + ("" if finite else "   NON-FINITE VALUES PRESENT"))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    parser.add_argument("--summary", action="store_true", help="print a per-field overview")
    parser.add_argument("--npz", help="save every field to this .npz")
    args = parser.parse_args()

    data = read(args.path)
    print(f"fields: {', '.join(k for k in data if not k.startswith('_'))}")
    if args.summary or not args.npz:
        summarize(data)
    if args.npz:
        np.savez_compressed(args.npz, **{k: v for k, v in data.items() if not k.startswith("_")})
        print(f"wrote {args.npz}")


if __name__ == "__main__":
    main()
