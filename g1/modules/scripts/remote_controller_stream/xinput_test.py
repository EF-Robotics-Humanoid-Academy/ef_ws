"""Read an Xbox-compatible controller through Windows XInput."""

from __future__ import annotations

import ctypes
import time
from ctypes import wintypes


ERROR_SUCCESS = 0

BUTTONS = {
    0x0001: "DPAD_UP",
    0x0002: "DPAD_DOWN",
    0x0004: "DPAD_LEFT",
    0x0008: "DPAD_RIGHT",
    0x0010: "START",
    0x0020: "BACK",
    0x0040: "LEFT_STICK",
    0x0080: "RIGHT_STICK",
    0x0100: "LB",
    0x0200: "RB",
    0x1000: "A",
    0x2000: "B",
    0x4000: "X",
    0x8000: "Y",
}


class XInputGamepad(ctypes.Structure):
    _fields_ = [
        ("buttons", wintypes.WORD),
        ("left_trigger", wintypes.BYTE),
        ("right_trigger", wintypes.BYTE),
        ("left_x", ctypes.c_short),
        ("left_y", ctypes.c_short),
        ("right_x", ctypes.c_short),
        ("right_y", ctypes.c_short),
    ]


class XInputState(ctypes.Structure):
    _fields_ = [
        ("packet_number", wintypes.DWORD),
        ("gamepad", XInputGamepad),
    ]


def load_xinput():
    for dll_name in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
        try:
            dll = ctypes.WinDLL(dll_name)
            function = dll.XInputGetState
            function.argtypes = [wintypes.DWORD, ctypes.POINTER(XInputState)]
            function.restype = wintypes.DWORD
            return function, dll_name
        except (AttributeError, OSError):
            continue
    raise RuntimeError("No Windows XInput DLL was found.")


def main() -> None:
    get_state, dll_name = load_xinput()
    print("Using:", dll_name)
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            state = XInputState()
            if get_state(0, ctypes.byref(state)) != ERROR_SUCCESS:
                print("No XInput controller connected.", flush=True)
            else:
                gamepad = state.gamepad
                buttons = [
                    name
                    for mask, name in BUTTONS.items()
                    if gamepad.buttons & mask
                ]
                axes = {
                    "LX": gamepad.left_x,
                    "LY": gamepad.left_y,
                    "RX": gamepad.right_x,
                    "RY": gamepad.right_y,
                    "LT": gamepad.left_trigger,
                    "RT": gamepad.right_trigger,
                }
                print("axes:", axes, "buttons:", buttons, flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
