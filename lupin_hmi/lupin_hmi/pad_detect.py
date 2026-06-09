"""Detect how an Xbox-style gamepad is attached: USB cable vs Bluetooth.

Why this matters
----------------
``joy_node`` (SDL2) reports buttons/axes in whatever order the *kernel driver*
exposes them, and the driver is chosen by the physical transport:

  * Wired (USB-C / USB) → ``xpad`` → the "standard" Xbox layout
    (A=0 B=1 X=2 Y=3, LB=4 RB=5, right-stick = axes 3/4, triggers = axes 2/5).
  * Bluetooth → the Microsoft-HID path → a *shifted* layout
    (A=0 B=1 X=3 Y=4 with index 2 skipped, LB=6 RB=7, right-stick X = axis 2).

So the transport bus is the ground-truth key for which mapping file to load.
We read it from ``/proc/bus/input/devices`` (``Bus=0003`` = USB, ``Bus=0005`` =
Bluetooth), falling back to sysfs. Pure stdlib — safe to import from a launch
file with no ROS dependency.
"""

from __future__ import annotations

import glob
import os

# Microsoft vendor id — covers Xbox One / Series wired + Bluetooth pads.
XBOX_VENDORS = {"045e"}

BUS_USB = "0003"
BUS_BLUETOOTH = "0005"
_BUS_TO_MODE = {BUS_USB: "usb", BUS_BLUETOOTH: "bluetooth"}

_PROC_DEVICES = "/proc/bus/input/devices"


def _read(path: str) -> str | None:
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return None


def _parse_proc_devices(text: str) -> list[dict]:
    """Split /proc/bus/input/devices into one dict per device block.

    Each block looks like::

        I: Bus=0005 Vendor=045e Product=0b13 Version=0903
        N: Name="Xbox Wireless Controller"
        H: Handlers=event5 js0
    """
    blocks: list[dict] = []
    cur: dict = {}
    for line in text.splitlines():
        if not line.strip():
            if cur:
                blocks.append(cur)
                cur = {}
            continue
        tag = line[:1]
        if tag == "I":
            for tok in line[2:].split():
                if "=" in tok:
                    key, val = tok.split("=", 1)
                    cur[key.lower()] = val.lower()
        elif tag == "N" and "Name=" in line:
            cur["name"] = line.split("Name=", 1)[1].strip().strip('"')
        elif tag == "H" and "Handlers=" in line:
            cur["handlers"] = line.split("Handlers=", 1)[1].split()
    if cur:
        blocks.append(cur)
    return blocks


def _joystick_pads() -> list[dict]:
    """Return device blocks that own a jsN handler, richest source first.

    Each dict carries (where available): ``name``, ``vendor``, ``bus``,
    ``js`` (e.g. ``js0``), ``mode`` ('usb'|'bluetooth'|None).
    """
    pads: list[dict] = []

    text = _read(_PROC_DEVICES)
    if text:
        for blk in _parse_proc_devices(text):
            js = next((h for h in blk.get("handlers", []) if h.startswith("js")), None)
            if not js:
                continue
            pads.append({
                "name": blk.get("name", ""),
                "vendor": blk.get("vendor", ""),
                "bus": blk.get("bus"),
                "js": js,
                "mode": _BUS_TO_MODE.get(blk.get("bus")),
            })
        if pads:
            return pads

    # Fallback: sysfs (some minimal/container kernels don't expose /proc one).
    for js_path in sorted(glob.glob("/sys/class/input/js*")):
        dev = os.path.join(js_path, "device")
        bus = (_read(os.path.join(dev, "id", "bustype")) or "").lower() or None
        pads.append({
            "name": _read(os.path.join(dev, "name")) or "",
            "vendor": (_read(os.path.join(dev, "id", "vendor")) or "").lower(),
            "bus": bus,
            "js": os.path.basename(js_path),
            "mode": _BUS_TO_MODE.get(bus),
        })
    return pads


def _pick(pads: list[dict]) -> dict | None:
    """Prefer a Microsoft pad with a resolvable mode, then any pad."""
    if not pads:
        return None
    xbox = [p for p in pads if p.get("vendor") in XBOX_VENDORS]
    for group in (xbox, pads):
        for pad in group:
            if pad.get("mode"):
                return pad
    return xbox[0] if xbox else pads[0]


def detect_pad_mode() -> str | None:
    """Return ``'usb'``, ``'bluetooth'``, or ``None`` if no pad / undetectable."""
    pad = _pick(_joystick_pads())
    return pad.get("mode") if pad else None


def describe_pads() -> str:
    """One-line human-readable summary of detected joystick(s) for logging."""
    pads = _joystick_pads()
    if not pads:
        return "no joystick devices found under /proc/bus/input or /sys/class/input"
    parts = []
    for pad in pads:
        mode = pad.get("mode") or f"bus={pad.get('bus')}?"
        name = pad.get("name") or "unknown"
        parts.append(f"{pad.get('js')}:{name} [{mode}]")
    return "; ".join(parts)


if __name__ == "__main__":  # pragma: no cover - manual probe helper
    print(f"detected mode: {detect_pad_mode()}")
    print(describe_pads())
