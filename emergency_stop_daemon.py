#!/usr/bin/env python3
"""
Watches a parent PID and, if it ever exits, immediately disables
each Thorlabs motor channel it knows about, then quits.
"""
import argparse
import os
import sys
import logging, os
import sys
import time
from pyThorlabsAPT.thorlabs_apt.core import Motor

# ─── Configure logging ───────────────────────────────────────────────────────
LOGFILE = os.path.join(os.path.dirname(__file__), "emergency_stop_daemon.log")
logging.basicConfig(
    filename=LOGFILE,
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s"
)
logging.info("=== WATCHDOG STARTUP ===")
logging.info(f"[WATCHDOG] raw sys.argv = {sys.argv!r}")


# ─── Bypass __main__ guard: always run main() ───────────────────────────────
def _run_watchdog():
    try:
        # parse args and start the loop
        parser = argparse.ArgumentParser(description="Emergency-stop watchdog")
        parser.add_argument('--pid',    type=int,  required=True,
                            help="PID of the GUI process to watch")
        parser.add_argument('--axes',   type=str,  required=True,
                            help="Comma-separated axis:serial pairs")
        parser.add_argument('--interval', type=float, default=1.0,
                            help="Polling interval in seconds")
        args = parser.parse_args()
        logging.info(f"[WATCHDOG] Parsed args: pid={args.pid}, axes={args.axes}, interval={args.interval}")

        # prepare motors
        serials = {axis: int(sn) for axis, sn in (pair.split(':') for pair in args.axes.split(','))}
        motors = {axis: Motor(sn) for axis, sn in serials.items()}

        # wait for parent to die
        while True:
            # liveliness probe
            alive = False
            if os.name == 'nt':
                import ctypes
                STILL_ACTIVE = 259
                h = ctypes.windll.kernel32.OpenProcess(0x1000, 0, args.pid)
                if h:
                    code = ctypes.c_ulong()
                    ctypes.windll.kernel32.GetExitCodeProcess(h, ctypes.byref(code))
                    alive = (code.value == STILL_ACTIVE)
            else:
                try:
                    os.kill(args.pid, 0)
                    alive = True
                except OSError:
                    alive = False
            if not alive:
                break
            time.sleep(args.interval)

        logging.info(f"[WATCHDOG] Detected parent PID {args.pid} gone; disabling channels.")
        for axis, motor in motors.items():
            try:
                motor.disable()
                logging.info(f"[WATCHDOG] Disabled axis {axis} (serial {motor.serial_number})")
            except Exception as e:
                logging.error(f"[WATCHDOG] Error disabling axis {axis}: {e}", exc_info=True)

    except Exception as e:
        logging.error(f"[WATCHDOG] Watchdog failed at top level: {e}", exc_info=True)

# Invoke immediately—no guard needed
_run_watchdog()

# def is_parent_alive(pid: int) -> bool:
#     if os.name == 'nt':
#         import ctypes
#         STILL_ACTIVE = 259
#         h = ctypes.windll.kernel32.OpenProcess(0x1000, 0, pid)
#         if not h:
#             return False
#         code = ctypes.c_ulong()
#         ctypes.windll.kernel32.GetExitCodeProcess(h, ctypes.byref(code))
#         return code.value == STILL_ACTIVE
#     else:
#         # Unix
#         try:
#             os.kill(pid, 0)
#             return True
#         except OSError:
#             return False

# def main():
#     p = argparse.ArgumentParser(description="Emergency-stop watchdog")
#     p.add_argument('--pid',    type=int,  required=True,
#                    help="PID of the GUI process to watch")
#     p.add_argument('--axes',   type=str,  required=True,
#                    help="Comma-separated axis:serial pairs, e.g. X:1234,Y:5678")
#     p.add_argument('--interval', type=float, default=1.0,
#                    help="Polling interval in seconds")
#     try:
#         args = p.parse_args()
#     except SystemExit as e:
#         logging.error(f"[WATCHDOG] Failed to parse arguments: {e}")
#         return
    
#     # Log the real args we received
#     logging.info(f"[WATCHDOG] Script launched, watching PID {args.pid} for axes {args.axes}")

#     # Parse axis→serial into integers
#     serials = {axis: int(sn) for axis, sn in
#                (pair.split(':') for pair in args.axes.split(','))}

#     # Instantiate a Motor for each serial; no need to call connect_device()
#     motors = {axis: Motor(sn) for axis, sn in serials.items()}

#     # Poll until parent exits
#     while is_parent_alive(args.pid):
#         time.sleep(args.interval)
        
#     # Parent has exited (crash, force-kill, etc.)
#     logging.info(f"[WATCHDOG] Detected parent PID {args.pid} gone; disabling channels.")

#     # Parent is gone → instantly disable each motor channel
#     for axis, motor in motors.items():
#         try:
#             motor.disable()
#             logging.info(f"[WATCHDOG] Disabled axis {axis} (serial {motor.serial_number})")
#         except Exception as e:
#             logging.error(f"[WATCHDOG] Error disabling axis {axis}: {e}")

# if __name__ == '__main__':
#     logging.info("[WATCHDOG] __main__ guard hit — starting main()")
#     try:
#         main()
#     except Exception as e:
#         logging.error(f"[WATCHDOG] main() threw exception: {e}")
