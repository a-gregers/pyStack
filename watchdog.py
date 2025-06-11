#!/usr/bin/env python3
"""
Detached watchdog: watches a parent PID and emergency-stops motors on exit,
using low-level Motor() instantiation with retries, DLL reload, and device enumeration.
"""
import argparse
import time
import os
import sys
import ctypes
import logging

import pyThorlabsAPT.thorlabs_apt.core as apt_core
from pyThorlabsAPT.thorlabs_apt.core import Motor

def parent_alive(pid: int) -> bool:
    """Return True if the process with PID is still running."""
    if os.name == 'nt':
        STILL_ACTIVE = 259
        h = ctypes.windll.kernel32.OpenProcess(0x1000, False, pid)
        if not h:
            return False
        code = ctypes.c_ulong()
        ctypes.windll.kernel32.GetExitCodeProcess(h, ctypes.byref(code))
        return code.value == STILL_ACTIVE
    else:
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False

def main():
    # ─── Parse arguments ───────────────────────────────────────────────────────
    parser = argparse.ArgumentParser(description="Emergency-stop watchdog")
    parser.add_argument('--pid',      type=int,   required=True,
                        help="PID of the GUI process to watch")
    parser.add_argument('--axes',     type=str,   required=True,
                        help="Comma-separated axis:serial pairs, e.g. X:1234,Y:5678")
    parser.add_argument('--interval', type=float, default=1.0,
                        help="Polling interval in seconds")
    args = parser.parse_args()

    # ─── Configure logging ──────────────────────────────────────────────────────
    LOGFILE = os.path.join(os.path.dirname(__file__), "watchdog.log")
    logging.basicConfig(
        filename=LOGFILE,
        level=logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s"
    )
    logging.info("=== WATCHDOG STARTUP ===")
    logging.info(f"Watching GUI PID={args.pid}, axes={args.axes}, interval={args.interval}s")

    # ─── Parse axis→serial dict ────────────────────────────────────────────────
    serials = {}
    for pair in args.axes.split(','):
        try:
            axis, sn = pair.split(':')
            serials[axis] = int(sn)
        except Exception as e:
            logging.error(f"Invalid axis:serial pair '{pair}': {e}")

    # ─── Wait until the GUI truly exits (freeing its APT handles) ───────────────
    iteration = 0
    while parent_alive(args.pid):
        if iteration % max(1, int(5.0/args.interval)) == 0:
            logging.info(f"Parent PID {args.pid} still alive (check #{iteration})")
        iteration += 1
        time.sleep(args.interval)

    logging.info(f"Detected GUI PID {args.pid} has exited after {iteration} checks")
    logging.info(f"Entering emergency-stop routine for axes: {list(serials.keys())}")

    # ─── Emergency-stop each axis ──────────────────────────────────────────────
    for axis, serial in serials.items():
        logging.info(f"[WATCHDOG] Axis {axis}: serial {serial} — stop sequence")
        motor = None

        # Retry instantiation up to 5×
        for attempt in range(1, 6):
            try:
                # 1) Clean up any old APT state
                apt_core._cleanup()
                # 2) Reload the DLL
                apt_core._lib = apt_core._load_library()
                # 3) Re-enumerate all hardware so InitHWDevice can find your motor :contentReference[oaicite:1]{index=1}
                apt_core.list_available_devices()
                logging.info(f"[WATCHDOG] Axis {axis}: performed list_available_devices()")
                # 4) Now instantiate the Motor
                motor = Motor(serial)
                logging.info(f"[WATCHDOG] Axis {axis}: Motor({serial}) init OK on attempt {attempt}")
                break
            except Exception as e:
                logging.error(f"[WATCHDOG] Axis {axis}: Motor init attempt {attempt} failed: {e}", exc_info=True)
                time.sleep(0.2)

        if motor is None:
            logging.error(f"[WATCHDOG] Axis {axis}: Could not open Motor({serial}) after 5 attempts")
            continue

        # ── Profiled stop if still moving ───────────────────────────────────────
        try:
            if motor.is_in_motion:
                logging.info(f"[WATCHDOG] Axis {axis}: is_in_motion=True, calling stop_profiled()")
                motor.stop_profiled()
                while motor.is_in_motion:
                    time.sleep(0.05)
                logging.info(f"[WATCHDOG] Axis {axis}: Motion ended")
        except Exception as e:
            logging.error(f"[WATCHDOG] Axis {axis}: stop_profiled() error: {e}", exc_info=True)

        # ── Final disable ───────────────────────────────────────────────────────
        try:
            motor.disable()
            logging.info(f"[WATCHDOG] Axis {axis}: disable() succeeded")
        except Exception as e:
            logging.error(f"[WATCHDOG] Axis {axis}: disable() failed: {e}", exc_info=True)

    logging.info("=== WATCHDOG COMPLETED ===")

if __name__ == "__main__":
    main()
