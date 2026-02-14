#!/usr/bin/env python3
"""
usb_controller.py — USB gamepad teleop for Unitree G1.

Requires: pip install pygame

Controls (Xbox-style layout):
    Left stick Y  : forward / backward (vx)
    Left stick X  : strafe left / right (vy)
    Right stick X : turn left / right (vyaw)

    A  : Damp (soft stop — motors go limp)
    B  : BalanceStand (stand still, balanced)
    X  : Start / Dev Mode (re-enter walking-ready FSM-200)
    Y  : ZeroTorque (emergency — kills all motors, robot WILL fall)

    Start button : StopMove (zero velocity, keep standing)
"""
from __future__ import annotations

import argparse
import time

import pygame

from hanger_boot_sequence import hanger_boot_sequence


# Xbox button indices (standard Linux xpad mapping)
BTN_A = 0
BTN_B = 1
BTN_X = 2
BTN_Y = 3
BTN_START = 7

# Axis indices
AXIS_LX = 0   # left stick X  -> vy (strafe)
AXIS_LY = 1   # left stick Y  -> vx (forward/back, inverted)
AXIS_RX = 3   # right stick X -> vyaw (turn)

MAX_VX = 0.5    # m/s
MAX_VY = 0.3    # m/s
MAX_VYAW = 0.8  # rad/s
DEADZONE = 0.1
SEND_HZ = 10


def apply_deadzone(value: float, dz: float) -> float:
    if abs(value) < dz:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dz) / (1.0 - dz)


def main() -> None:
    parser = argparse.ArgumentParser(description="USB gamepad teleop for G1.")
    parser.add_argument("--iface", default="eth0", help="network interface")
    parser.add_argument("--joy", type=int, default=0, help="joystick index")
    args = parser.parse_args()

    bot = hanger_boot_sequence(iface=args.iface)

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise SystemExit("No joystick detected. Connect a USB gamepad and retry.")

    joy = pygame.joystick.Joystick(args.joy)
    joy.init()
    print(f"Using: {joy.get_name()}  (axes={joy.get_numaxes()}, buttons={joy.get_numbuttons()})")
    print("A=Damp  B=BalanceStand  X=DevMode  Y=ZeroTorque  Start=StopMove")

    active = True  # False after Damp/ZeroTorque (don't send Move commands)
    dt = 1.0 / SEND_HZ

    try:
        while True:
            pygame.event.pump()

            # --- buttons ---
            if joy.get_button(BTN_Y):
                print("[Y] ZeroTorque — EMERGENCY")
                bot.ZeroTorque()
                active = False
                time.sleep(0.5)
                continue

            if joy.get_button(BTN_A):
                print("[A] Damp")
                bot.Damp()
                active = False
                time.sleep(0.5)
                continue

            if joy.get_button(BTN_X):
                print("[X] Dev Mode — Squat2StandUp + Start")
                bot.Damp()
                time.sleep(0.5)
                bot.Squat2StandUp()
                time.sleep(1.0)
                bot.Start()
                active = True
                time.sleep(0.5)
                continue

            if joy.get_button(BTN_B):
                print("[B] BalanceStand")
                bot.BalanceStand(0)
                active = True
                time.sleep(0.5)
                continue

            if joy.get_numbuttons() > BTN_START and joy.get_button(BTN_START):
                bot.StopMove()
                time.sleep(0.2)
                continue

            # --- sticks ---
            if active:
                lx = apply_deadzone(joy.get_axis(AXIS_LX), DEADZONE)
                ly = apply_deadzone(joy.get_axis(AXIS_LY), DEADZONE)
                rx = apply_deadzone(joy.get_axis(AXIS_RX), DEADZONE)

                vx = -ly * MAX_VX     # stick up = negative axis = forward
                vy = -lx * MAX_VY     # stick left = negative axis = strafe left (positive vy)
                vyaw = -rx * MAX_VYAW  # stick left = negative axis = turn left (positive vyaw)

                bot.Move(vx, vy, vyaw)

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        bot.StopMove()
        pygame.quit()


if __name__ == "__main__":
    main()
