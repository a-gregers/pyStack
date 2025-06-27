import traceback, sys, logging, faulthandler
import os, json, time, argparse, atexit, PyQt5, threading
import pyThorlabsAPT.driver as driver_real
import pyThorlabsAPT.driver_virtual as driver_virtual
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtWidgets import (
    QDialog, QListWidget, QListWidgetItem,
    QVBoxLayout, QHBoxLayout, QPushButton,
    QStackedWidget, QSizePolicy, QCheckBox
)
from pyThorlabsAPT.thorlabs_apt import core as apt_core
from dataclasses import dataclass


apt_core._lib = apt_core._load_library()
dirname = os.path.dirname(PyQt5.__file__)
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = os.path.join(dirname, 'plugins', 'platforms')
faulthandler.enable(all_threads=True)
atexit.register(apt_core._cleanup)
# — optionally configure logging to a file or console:
logging.basicConfig(
    level=logging.ERROR,
    format='%(asctime)s [%(levelname)s] %(message)s'
)

# 
# Catch any exception inside Qt callbacks & print it to console
def exception_hook(exctype, value, tb):
    # log full traceback
    logging.error("Unhandled exception in Qt callback", exc_info=(exctype, value, tb))
    # now delegate to the original handler so Python will exit
    sys._excepthook(exctype, value, tb)

sys._excepthook = sys.excepthook
sys.excepthook = exception_hook
# 

print(f"[DEBUG] {time.strftime('%H:%M:%S')}  :  Entering pyThorlabsAPT (main module).")

UM_TO_MM = 1e-3      # microns to millimeters
MM_TO_UM = 1e3       # millimeters to microns

# Define a simple config to drive axis widget construction
@dataclass
class AxisConfig:
    label: str
    iface: "interface"
    allow_drive: bool

class interface(QtCore.QObject):
    """
    A wrapper around a single Thorlabs device. Emits signals when things change.
    """
    sig_list_devices_updated = QtCore.pyqtSignal(list)
    sig_update_position       = QtCore.pyqtSignal(float)
    sig_step_size             = QtCore.pyqtSignal(float)
    sig_change_moving_status  = QtCore.pyqtSignal(int)
    sig_change_homing_status  = QtCore.pyqtSignal(int)
    sig_stage_info            = QtCore.pyqtSignal(list)
    sig_close                 = QtCore.pyqtSignal()
    sig_connected             = QtCore.pyqtSignal(bool)

    SIG_MOVEMENT_STARTED = 1
    SIG_MOVEMENT_ENDED   = 2
    SIG_HOMING_STARTED   = 1
    SIG_HOMING_ENDED     = 2

    def __init__(self, use_virtual=False, parent=None):
        super().__init__(parent)
        self.use_virtual = use_virtual
        self.instrument = None

        # Default settings:
        self.settings = {
            'step_size': 1.0,  # in mm
            'ramp': {
                'ramp_step_size': 1,
                'ramp_wait_1': 1,
                'ramp_send_trigger': True,
                'ramp_wait_2': 1,
                'ramp_numb_steps': 10,
                'ramp_repeat': 1,
                'ramp_reverse': 1,
                'ramp_send_initial_trigger': 1,
                'ramp_reset': 1
            }
        }
        self.list_devices = []
        self.connected_device_name = ''
        self.output = {'Position': 0.0}
        self._units = {'mm': 1, 'deg': 2}

        # Pick driver:
        if self.use_virtual:
            self.instrument = driver_virtual.pyThorlabsAPT()
        else:
            self.instrument = driver_real.pyThorlabsAPT()

        # Immediately refresh device list:
        QtCore.QTimer.singleShot(0,   self.refresh_list_devices)
        QtCore.QTimer.singleShot(200, self.refresh_list_devices)
        
        # Track the last‐set velocity (mm/s) for threshold logic:
        self._last_velocity = 0.0
        self._last_acceleration = 0.0

    def refresh_list_devices(self):
        """
        Search for devices and emit the list as ['IDN --> Name', …].
        """
        try:
            raw_list = self.instrument.list_devices()
        except Exception:
            raw_list = []
        formatted = [f"{dev[1]} --> {dev[0]}" for dev in raw_list]
        self.list_devices = formatted
        self.sig_list_devices_updated.emit(formatted)

    def connect_device(self, device_fullname):
        """
        Connect to the chosen device name (IDN).
        """
        if device_fullname == '':
            return
        device_name = device_fullname.split(' --> ')[0].strip()
        try:
            msg, code = self.instrument.connect_device(device_name)
            if code == 1:
                self.connected_device_name = device_name
                self.read_position()
                self.read_stage_info()
                self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
                self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
                self.sig_connected.emit(True)
            else:
                self.connected_device_name = ''
                self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
                self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
                self.sig_connected.emit(False)
        except Exception as e:
            self.connected_device_name = ''
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
            self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
            print(f"[ERROR] connect_device('{device_fullname}') failed: {e}")
            self.sig_connected.emit(False)

    def disconnect_device(self):
        """
        Disconnect if currently connected.
        """
        try:
            _msg, _code = self.instrument.disconnect_device()
        except Exception as e:
            print(f"[ERROR] disconnect_device failed: {e}")
        self.connected_device_name = ''
        self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
        self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
        self.sig_connected.emit(False)

    def is_device_moving(self):
        try:
            return bool(getattr(self.instrument, 'is_in_motion', False))
        except Exception:
            return False

    def stop_any_movement(self):
        if not self.is_device_moving():
            return

        # threshold = 100 µm/s to in mm/s:
        threshold_mm_s = 100.0 * UM_TO_MM
        try:
            # read back whatever max-vel is currently set
            _, current_acc, current_maxv = self.instrument.get_velocity_parameters()

            if current_maxv > threshold_mm_s:
                # above threshold : smooth, profiled stop
                self.instrument.stop_profiled()
            else:
                # below threshold : zero-vel “sudden” stop
                # (set max_vel = 0 so motor brakes instantly)
                # self.instrument.set_velocity_profile(0.0, current_acc, 0.0)
                self.set_velocity_profile(0.0, current_acc)
                self.move_velocity_continuous(1)
                self.instrument.stop_profiled()
        except Exception as e:
            print(f"[ERROR] stop_any_movement failed: {e}")
        finally:
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
            
    def set_velocity_profile(self, vel_mm_s: float, accn_mm_s2: float):
        try:
            # remember what we just commanded
            self._last_velocity     = vel_mm_s
            self._last_acceleration = accn_mm_s2
            self.instrument.set_velocity_parameters(0.0, accn_mm_s2, vel_mm_s)
        except Exception as e:
            # Log any hardware/API errors so they're visible
            print(f"[ERROR] set_velocity_profile({vel_mm_s}, {accn_mm_s2}) failed: {e}")

    def move_velocity_continuous(self, direction: int):
        """
        Start continuous motion (no target stops) in 'direction':
            direction = +1 for forward (MOVE_FWD), or -1 for reverse (MOVE_REV).
        Internally calls move_velocity(1) or move_velocity(2).
        """
        try:
            code = 1 if direction > 0 else 2
            self.instrument.move_velocity(code)
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_STARTED)
        except Exception as e:
            print(f"[ERROR] move_velocity_continuous(dir={direction}) failed: {e}")
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)

    def stop_profiled_continuous(self):
        """
        When called, the motor decelerates smoothly to a stop (profiled).
        """
        try:
            self.instrument.stop_profiled()
        except Exception:
            pass
        
        # Notify GUI of movement end
        self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)

        # We do NOT emit SIG_MOVEMENT_ENDED here; you already handle polling to emit it.

    def home(self):
        """
        Send the home command and hand off to PollerThread for live updates.
        """
        try:
            if self.is_device_moving():
                return
            # tell GUI we're starting homing
            self.sig_change_homing_status.emit(self.SIG_HOMING_STARTED)
            # also treat it as a movement start so PollerThread kicks in
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_STARTED)
            # actually fire the home command (non-blocking)
            self.instrument.move_home()
        except Exception:
            # if we get an error, make sure GUI knows homing/motion ended
            self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)

    def move_single_step(self, direction):
        """
        direction = +1 or -1; uses self.settings['step_size'] (in mm).
        """
        try:
            if self.is_device_moving():
                return
            step_mm = self.settings['step_size'] * direction
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_STARTED)
            self.instrument.move_by(step_mm)
            self._poll_move()
        except Exception:
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)

    def _poll_move(self):
        """
        Internal: poll until is_in_motion is False, then emit read_position().
        """
        try:
            if self.instrument.is_in_motion:
                QtCore.QTimer.singleShot(50, self._poll_move)
            else:
                self.read_position()
                self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
        except Exception:
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)

    def set_step_size(self, s_mm: str):
        """
        s_mm is a string in mm. Convert to float & store.
        """
        try:
            val = float(s_mm)
            self.settings['step_size'] = val
            self.sig_step_size.emit(val)
            return True
        except:
            return False

    def set_position(self, text_mm: str):
        """
        text_mm is a string representing desired absolute position (in mm).
        """
        if self.is_device_moving():
            return
        try:
            pos = float(text_mm)
            self.instrument.position = pos
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_STARTED)
            QtCore.QTimer.singleShot(50, self._poll_setpos_finished)
        except Exception as e:
            print(f"[ERROR] set_position failed: {e}")

    def _poll_setpos_finished(self):
        # always grab & emit the latest position
        try:
            self.read_position()
            if self.instrument.is_in_motion:
                QtCore.QTimer.singleShot(50, self._poll_setpos_finished)
            else:
                # final update already emitted above
                self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
        except Exception as e:
            print(f"[ERROR] _poll_setpos_finished failed: {e}")
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)

    def read_position(self):
        """
        Poll the motor and emit the updated position (in mm).
        """
        try:
            pos = float(self.instrument.position)
        except:
            pos = 0.0
        self.output['Position'] = pos
        self.sig_update_position.emit(pos)
        return pos

    def get_stage_info(self):
        """
        Return a list: [min_pos_mm, max_pos_mm, unit_str, pitch].
        """
        try:
            info = list(self.instrument.get_stage_axis_info())
            unit_key = [k for k, v in self._units.items() if v == info[2]][0]
            info[2] = unit_key
            return info
        except:
            return [0.0, 0.0, 'mm', 1.0]

    def read_stage_info(self):
        info = self.get_stage_info()
        self.sig_stage_info.emit(info)
        return info

    def set_stage_info(self, min_pos, max_pos, units, pitch):
        """
        Called when user types stage-limits + hits “Set X Stage”.
        """
        try:
            mn = float(min_pos)
            mx = float(max_pos)
            pt = float(pitch)
            un = self._units.get(units, 1)
            self.instrument.set_stage_axis_info(mn, mx, un, pt)
            self.read_stage_info()
            return True
        except:
            return False

    def close(self):
        """
        Called when the app is quitting.
        """
        try:
            self.instrument.disconnect_device()
        except:
            pass
        self.sig_close.emit()
        
class PollerThread(QtCore.QThread):
    positionUpdated = QtCore.pyqtSignal(str, float)        # axis, position_mm
    settled         = QtCore.pyqtSignal(str)               # axis (motion has truly stopped)
    limitsUpdated   = QtCore.pyqtSignal(str, float, float) # axis, min_mm, max_mm
    switchUpdated   = QtCore.pyqtSignal(str, bool, bool)   # axis, rev_active, fwd_active
    hardwareStatusUpdated = QtCore.pyqtSignal(str, bool, bool)

    def __init__(self, interfaces: dict[str, interface], poll_hz: int = 20):
        super().__init__()
        self.interfaces      = interfaces
        self._interval_ms  = int(1000 / poll_hz)
        self._poll_hz      = poll_hz
        self._running        = True
        self._poll_flags     = {ax: False for ax in interfaces}
        self._last_pos       = {ax: None  for ax in interfaces}
        self._settle_counts  = {ax: 0     for ax in interfaces}
        self._tol         = 0.001 * UM_TO_MM      # 1 µm tolerance
        self._req_counts     = 3
        self._paused = {ax: False for ax in interfaces}
        self._running = True

    def run(self):
        loop_ct = 0
        while self._running:
            for ax, iface in self.interfaces.items():
                #  RACE-PROOF LOCK: if GUI code holds it, skip polling 
                if not iface._lock.acquire(blocking=False):
                    self.msleep(self._interval_ms)
                    continue

                try:
                    # Only poll if GUI+driver say “connected” and __init__ ran
                    if (not iface.connected_device_name
                            or not getattr(iface.instrument, 'connected', False)
                            or not hasattr(iface.instrument, '_serial_number')):
                        self.msleep(self._interval_ms)
                        continue
                
                    try:
                        # read the *live* endstop engagement
                        rev = iface.instrument.is_forward_hardware_limit_switch_active
                        fwd = iface.instrument.is_reverse_hardware_limit_switch_active
                    except Exception as e:
                        # if the device just re-plugged (Unknown serial), skip this cycle
                        if "Unknown serial number" in str(e):
                            self.msleep(self._interval_ms)
                            continue
                        # otherwise log & fall back
                        logging.error(f"[PollerThread:{ax}] switch read error: {e}")
                        # skip this cycle (don’t kill the other axes)
                        self.msleep(self._interval_ms)
                        continue
            
                    # APT’s “forward” code is actually your REV‐limit,
                    # and APT’s “reverse” code is your FWD‐limit
    
                    self.switchUpdated.emit(ax, rev, fwd)
                    
                    try:
                        in_motion = iface.instrument.is_in_motion
                        homed     = iface.instrument.has_homing_been_completed
                        self.hardwareStatusUpdated.emit(ax, in_motion, homed)
                    except Exception as e:
                        logging.error(f"[PollerThread:{ax}] status read error: {e}")
                        # skip this cycle (don’t kill the connection)
                        self.msleep(self._interval_ms)
                        continue
    
                    # Numeric position & settle logic (on demand) 
                    if self._poll_flags[ax]:
                        try:
                            pos = iface.read_position()
                            self.positionUpdated.emit(ax, pos)
                        except:
                            # maybe device disconnected
                            continue
    
                        moving = iface.is_device_moving()
                        if not moving:
                            last = self._last_pos[ax]
                            if last is not None and abs(pos - last) < self._tol:
                                self._settle_counts[ax] += 1
                                if self._settle_counts[ax] >= self._req_counts:
                                    self.settled.emit(ax)
                                    self._poll_flags[ax]    = False
                                    self._settle_counts[ax] = 0
                            else:
                                self._settle_counts[ax] = 0
                        else:
                            self._settle_counts[ax] = 0
                        self._last_pos[ax] = pos
    
                    # Every once in a while, refresh limits
                    if loop_ct % self._poll_hz == 0:
                        try:
                            mn, mx, *_ = iface.get_stage_info()
                            self.limitsUpdated.emit(ax, mn, mx)
                        except:
                            pass
                except Exception as e:
                    print("[ERROR]: {e}")
                finally:
                    iface._lock.release()

            loop_ct += 1
            self.msleep(self._interval_ms)

    def stop(self):
        self._running = False
        self.wait()
        
    def pause_axis(self, axis: str):
        self._paused[axis] = True

    def resume_axis(self, axis: str):
        self._paused[axis] = False

    def start_poll(self, axis: str):
        self._last_pos[axis]      = None
        self._settle_counts[axis] = 0
        self._poll_flags[axis]    = True

    def stop_poll(self, axis: str):
        self._poll_flags[axis] = False

class ConnectDevicesDialog(QDialog):
    """
    Dialog to list all available Thorlabs devices, and
    connect/disconnect them in bulk.
    """
    def __init__(self, gui, parent=None):
        super().__init__(parent)
        self.gui = gui
        self.setWindowTitle("Manage Device Connections")
        # whenever any axis finishes a USB rescan, re-populate
        for ax in ("X","Y","Z"):
            getattr(self.gui, f"iface_{ax}").sig_list_devices_updated.connect(
                self._populate_list
            )

        # 1) Fetch the full device list once:
        #    instrument.list_devices() returns [(model, serial),...]
        self.instrument = driver_real.pyThorlabsAPT()
        raw = []
        try:
            raw = self.instrument.list_devices()
        except Exception:
            pass
        # Format: [(serial, model), ...]
        devices = [(str(sn), hw) for hw, sn in raw]

        vlay = QVBoxLayout(self)

        # “Select All” toggle
        self.chk_select_all = QCheckBox("Select All")
        self.chk_select_all.toggled.connect(self._on_select_all)

        # “Refresh List” button
        btn_refresh = QPushButton("Refresh List")
        btn_refresh.clicked.connect(self._do_refresh_list)

        # pack them side-by-side
        row_top = QHBoxLayout()
        row_top.addWidget(self.chk_select_all)
        row_top.addWidget(btn_refresh)
        vlay.addLayout(row_top)

        # 2) Device list (we’ll fill it via a helper so Refresh works)
        self.list = QListWidget()
        vlay.addWidget(self.list)

        # 3) Buttons at the bottom
        btn_connect_sel    = QPushButton("Connect Selected")
        btn_disconnect_sel = QPushButton("Disconnect Selected")
        btn_close          = QPushButton("Close")

        # Hook them up:
        btn_connect_sel.clicked.connect(self._do_connect_selected)
        btn_disconnect_sel.clicked.connect(self._do_disconnect_selected)
        btn_close.clicked.connect(self.accept)

        vlay.addWidget(self.list)
        hlay = QHBoxLayout()
        for w in (btn_connect_sel, btn_disconnect_sel, btn_close):
            hlay.addWidget(w)
        vlay.addLayout(hlay)
        
        # finally, populate for the first time
        self._populate_list()

    def _on_select_all(self, checked: bool):
        """Toggle every row’s checkbox on/off."""
        for i in range(self.list.count()):
            self.list.item(i).setCheckState(Qt.Checked   if checked
                                            else Qt.Unchecked)
    def _selected_serials(self):
        # returns a list of all serials whose checkbox is Qt.Checked
        out = []
        for i in range(self.list.count()):
            item = self.list.item(i)
            if item.checkState() == Qt.Checked:
                serial = item.text()
                out.append(serial)
        return out

    def _do_connect_selected(self):
        """
        Synchronize each axis to exactly the set of checked serials:
          – Connect axes whose combo.currentText() is checked but not yet connected
          – Disconnect axes whose combo.currentText() is unchecked but is connected
        """
        selected = set(self._selected_serials())

        for axis in ("X", "Y", "Z"):
            iface = getattr(self.gui, f"iface_{axis}")
            combo = getattr(self.gui, f"combo_Device{axis}")
            current_serial = iface.connected_device_name   # "" if none

            want = combo.currentText()   # the serial user has assigned to this axis

            # 1) Connect if they checked it but it isn’t connected yet
            if want in selected and not current_serial:
                # ensure the combo actually shows it (usually true)
                idx = combo.findText(want)
                if idx >= 0:
                    combo.setCurrentIndex(idx)
                # this calls iface.connect_device() under the hood :contentReference[oaicite:1]{index=1}
                self.gui._connect_clicked(axis)

            # 2) Disconnect if they unchecked it but it’s currently up
            elif current_serial and current_serial not in selected:
                self.gui._connect_clicked(axis)

        self.accept()

    def _do_disconnect_selected(self):
        """
        Only disconnect the axes whose current serial is among the checked items.
        """
        # grab the set of checked serial-numbers
        selected = set(self._selected_serials())

        for axis in ("X","Y","Z"):
            iface = getattr(self.gui, f"iface_{axis}")
            current = iface.connected_device_name  # e.g. "26006151" or "" if none

            # if this axis is up and its serial is checked: disconnect it
            if current and current in selected:
                # reuse the same toggle slot you have for connect/disconnect
                self.gui._connect_clicked(axis)

        self.accept()

    def _populate_list(self):
        """
        Re-scan via iface.instrument.list_devices() and rebuild the QListWidget.
        """
        from PyQt5.QtCore    import Qt
        from PyQt5.QtWidgets import QListWidgetItem
    
        # 1) ask the hardware wrapper to re-enumerate
        try:
            raw_list = self.instrument.list_devices()
        except Exception:
            raw_list = []
    
        # 2) raw_list is a list of [model,serial] (virtual) or (hwtype,serial) (real)
        #    so we normalize it to (serial,hw) pairs:
        devices = [(str(dev[1]), str(dev[0])) for dev in raw_list]
    
        # 3) fill the list widget
        self.list.clear()
        for serial, hw in devices:
            item = QListWidgetItem(f"{serial}")
            item.setCheckState(Qt.Checked if self.chk_select_all.isChecked()
                                else Qt.Unchecked)
            self.list.addItem(item)

    def _do_refresh_list(self):
        """
        Ask each axis‐interface to rescan its USB, then
        rely on sig_list_devices_updated to _populate_list.
        """
        # lock out polling while we rescan
        locks = []
        for ax in ("X","Y","Z"):
            iface = getattr(self.gui, f"iface_{ax}")
            iface._lock.acquire()
            locks.append(iface._lock)

        try:
            # nothing else needed — the dialog only uses _populate_list()
            # and that immediately hits .instrument.list_devices()
            self.instrument = driver_real.pyThorlabsAPT()
            self._populate_list()
        finally:
            for ax in ("X","Y","Z"):
                getattr(self.gui, f"iface_{ax}")._lock.release()

# ---------------------------------------------------------------------------------
class MultiAxisGui(QtWidgets.QWidget):
    def __init__(self, iface_x: interface, iface_y: interface, iface_z: interface, parent=None):
        super().__init__(parent)
        
        # Only show these serials per axis if they’re present:
        self.assigned_serials = {
            'X': '26006151',
            'Y': '26006139',
            'Z': '26002801',
        }

        # Minimum allowed velocity (µm/s) and acceleration (µm/s²) by axis:
        self._min_vel_disp = {'X': 1.0, 'Y': 1.0, 'Z': 0.1}
        self._min_acc_disp = {'X': 1.0, 'Y': 1.0, 'Z': 0.1}

        self.iface_x = iface_x
        self.iface_y = iface_y
        self.iface_z = iface_z

        # Keep track of display‐unit multiplier (always mm for simplicity here):
        self.display_unit = "mm"
        
        # Unified PollerThread for all axes 
        interfaces = {
            "X": self.iface_x,
            "Y": self.iface_y,
            "Z": self.iface_z,
        }
        
        for ax, iface in interfaces.items():
            iface._lock = threading.Lock()

        self.poller = PollerThread(interfaces, poll_hz=50)
        self.poller.positionUpdated.connect(self._on_position_change)
        self.poller.settled       .connect(self._on_motion_settled)
        self.poller.limitsUpdated .connect(self._on_limits_update)
        self.poller.switchUpdated.connect(self.on_limit_switches)
        self.poller.hardwareStatusUpdated.connect(self._on_hardware_status_update)
        self.poller.start()
        
        # only update the UI when the change exceeds this (in mm)
        self.display_resolution_mm = 0.001 * UM_TO_MM   # e.g. 1 µm
    
        # remember last value shown on screen for each axis
        self.last_displayed = {'X': None, 'Y': None, 'Z': None}
        
        # Top‐level layout: axis groups stacked vertically
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        btn_manage = QtWidgets.QPushButton("Manage Connections…")
        btn_manage.setFocusPolicy(Qt.NoFocus)
        btn_manage.setToolTip("Connect or disconnect multiple motors at once")
        btn_manage.clicked.connect(self._show_manage_dialog)
        layout.addWidget(btn_manage)

        # Build X‐axis group:
        axis_configs = [
            AxisConfig("X", self.iface_x, False),
            AxisConfig("Y", self.iface_y, False),
            AxisConfig("Z", self.iface_z, True),
        ]
        
        for cfg in axis_configs:
            grp = self._build_axis_group(cfg.label, cfg.iface, allow_drive=cfg.allow_drive)
            layout.addWidget(grp)
        
        #  Preset controls 
        hbtn = QtWidgets.QHBoxLayout()
        btn_load = QtWidgets.QPushButton("Load Preset…")
        btn_go   = QtWidgets.QPushButton("Go to Preset")
        lbl_status = QtWidgets.QLabel("Preset: None")
        hbtn.addWidget(btn_load)
        hbtn.addWidget(btn_go)
        hbtn.addWidget(lbl_status)
        layout.addLayout(hbtn)
        btn_load        .setFocusPolicy(QtCore.Qt.NoFocus)
        btn_go        .setFocusPolicy(QtCore.Qt.NoFocus)
            
        self.button_LoadPreset = btn_load
        self.button_GoPreset  = btn_go
        # Start with it grayed out until both X & Y are live:
        self.button_GoPreset.setEnabled(False)
        self.label_PresetStatus = lbl_status
           
        btn_load.clicked.connect(self.load_preset)
        btn_go.clicked.connect(self.go_preset)
        default = os.path.join(os.path.dirname(__file__), "target.json")
        if os.path.exists(default):
            try:
                with open(default, 'r') as f:
                    data = json.load(f)
                self._apply_preset_dict(data, show_message=False)
                self.label_PresetStatus.setText(f"Preset: {os.path.basename(default)}")
            except Exception:
                # leave status as “None” on parse error
                pass

        # Tooltip for keyboard shortcuts:
        tips = (
            "Keyboard Shortcuts:\n"
            "  A / D  : Move X left / right  (or hold for continuous)\n"
            "  W / S  : Move Y forward / back  (or hold for continuous)\n"
            "  ↑ / ↓  : Move Z up / down  (or hold for continuous)\n"
            "  H / J / K : Home X / Home Y / Home Z\n"
            "  Space : Stop ALL motors\n"
        )
        self.setToolTip(tips)
        
        # Track when the user actually requested a home, per-axis:
        self._homing_requested = {ax: False for ax in ("X","Y","Z")}

        # Ensure keyPressEvent works:
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setFocus()
        
    def _build_axis_group(self, axis_label: str, iface: interface, allow_drive: bool) -> QtWidgets.QGroupBox:
        """
        Build a QGroupBox containing all controls for one axis (Label e.g. "X").
        `allow_drive`=True only for Z; False for X/Y.
        """
        grp = QtWidgets.QGroupBox(f"{axis_label}-Axis Controls")
        vlay = QtWidgets.QVBoxLayout(grp)
        vlay.setSpacing(4)
        vlay.setContentsMargins(4, 4, 4, 4)

        # --- Row 1: Device selector, Refresh, Connect/Disconnect ---
        hlay1 = QtWidgets.QHBoxLayout()
        label_dev = QtWidgets.QLabel(f"{axis_label} Device:")
        combo_dev = QtWidgets.QComboBox()
        combo_dev.setSizeAdjustPolicy(
            QtWidgets.QComboBox.AdjustToContents
        )
        # status indicator next to connect:
        lbl_status1 = QtWidgets.QLabel("Status:")
        status_display = QtWidgets.QLabel("Not connected")
        setattr(self, f"label_Status{axis_label}", status_display)

        setattr(self, f"combo_Device{axis_label}", combo_dev)
        setattr(self, f"iface_{axis_label}", iface)

        for w in [label_dev, combo_dev, lbl_status1, status_display]:
            hlay1.addWidget(w)
        hlay1.addStretch(1)
        vlay.addLayout(hlay1)
        
        # Container for everything *below* Row 1
        container = QtWidgets.QWidget()
        setattr(self, f"container_Controls{axis_label}", container)
        ctrl_vlay = QtWidgets.QVBoxLayout(container)
        ctrl_vlay.setContentsMargins(0,0,0,0)
        ctrl_vlay.setSpacing(4)

        # --- Row 2: Mode selector (Jog / Drive) ---
        hlay2 = QtWidgets.QHBoxLayout()
        label_mode = QtWidgets.QLabel("Mode:")
        combo_mode = QtWidgets.QComboBox()
        if allow_drive:
            combo_mode.addItems(["Jog", "Drive"])
        else:
            combo_mode.addItems(["Jog"])
        combo_mode.setCurrentText("Jog")

        setattr(self, f"combo_Mode{axis_label}", combo_mode)

        hlay2.addWidget(label_mode)
        hlay2.addWidget(combo_mode)
        hlay2.addStretch(1)
        ctrl_vlay.addLayout(hlay2)

        # --- Row 3: Jog sub-mode (Continuous / Single) — only visible when Mode=Jog ---
        hlay3 = QtWidgets.QHBoxLayout()
        radio_cont = QtWidgets.QRadioButton("Continuous")
        radio_single = QtWidgets.QRadioButton("Single‐Click")
        radio_cont.setChecked(True)
        label_vel = QtWidgets.QLabel("Velocity (µm/s):")
        edit_vel = QtWidgets.QLineEdit("1.0")
        label_acc = QtWidgets.QLabel("Acceleration (µm/s²):")
        edit_acc = QtWidgets.QLineEdit("1.0")

        setattr(self, f"radio_Cont{axis_label}", radio_cont)
        setattr(self, f"radio_Single{axis_label}", radio_single)
        setattr(self, f"edit_Velocity{axis_label}", edit_vel)
        setattr(self, f"edit_Accel{axis_label}", edit_acc)
        
        # create a shared double-validator
        double_validator = QDoubleValidator(self)
        double_validator.setNotation(QDoubleValidator.StandardNotation)
        double_validator.setDecimals(6)  # up to six decimal places 
        edit_vel.setValidator(double_validator)
        edit_acc.setValidator(double_validator)
        
        # Whenever they toggle sub-mode, show/hide the “By” container accordingly:
        radio_single.toggled.connect(lambda checked, ax=axis_label: getattr(self, f"container_By{ax}").setVisible(checked))
        radio_cont.toggled.connect(lambda checked, ax=axis_label: getattr(self, f"container_By{ax}").setVisible(False))

        for w in [radio_cont, radio_single, label_vel, edit_vel, label_acc, edit_acc]:
            hlay3.addWidget(w)
        hlay3.addStretch(1)
        # For axes without Drive, ensure the Mode stays on Jog
        if not allow_drive:
            combo_mode.setCurrentText("Jog")

        # Create a container widget for the Jog options, set its layout to hlay3, then add it:
        widget_row3 = QtWidgets.QWidget()
        widget_row3.setLayout(hlay3)
        setattr(self, f"widget_JogOptions{axis_label}", widget_row3)
        # Now add that widget (which owns hlay3) into vlay:
        ctrl_vlay.addWidget(widget_row3)

        # --- Row 4: Position display, Move < / >, Step size, Home, Stop ---
        hlay4 = QtWidgets.QHBoxLayout()
        label_pos = QtWidgets.QLabel(f"{axis_label} Position:")
        edit_pos = QtWidgets.QLineEdit("0.000")
        edit_pos.setAlignment(QtCore.Qt.AlignRight)
        btn_move_neg = QtWidgets.QPushButton("<")
        btn_move_pos = QtWidgets.QPushButton(">")
        btn_move_neg.setFocusPolicy(QtCore.Qt.NoFocus)
        btn_move_pos.setFocusPolicy(QtCore.Qt.NoFocus)
        label_by = QtWidgets.QLabel("By:")  # we'll set text and show/hide later
        edit_step = QtWidgets.QLineEdit(str(iface.settings['step_size']*MM_TO_UM))
        btn_home = QtWidgets.QPushButton(f"Home {axis_label}")
        btn_stop = QtWidgets.QPushButton(f"Stop {axis_label}")
        btn_stop.setEnabled(False)

        setattr(self, f"edit_Position{axis_label}", edit_pos)
        setattr(self, f"button_MovePos{axis_label}", btn_move_pos)
        setattr(self, f"edit_StepSize{axis_label}", edit_step)
        setattr(self, f"button_Home{axis_label}", btn_home)
        setattr(self, f"button_Stop{axis_label}", btn_stop)
        
        # Position (µm) – 3 decimal places
        pos_validator = QDoubleValidator(self)
        pos_validator.setNotation(QDoubleValidator.StandardNotation)
        pos_validator.setDecimals(3)
        edit_pos.setValidator(pos_validator)

        # Step-size “By:” (µm) – 6 decimal places
        step_validator = QDoubleValidator(self)
        step_validator.setNotation(QDoubleValidator.StandardNotation)
        step_validator.setDecimals(6)
        edit_step.setValidator(step_validator)

        # We’ll add label_pos, edit_pos, btn_move_neg, btn_move_pos first:
        for w in [label_pos, edit_pos, btn_move_neg, btn_move_pos]:
            hlay4.addWidget(w)

        # Create a sub-widget for “By (unit): [step]” so we can show/hide it
        container_by = QtWidgets.QWidget()
        h_by = QtWidgets.QHBoxLayout(container_by)
        h_by.setContentsMargins(0, 0, 0, 0)
        h_by.addWidget(label_by)
        h_by.addWidget(edit_step)
        container_by.hide()  # single-click will show it
        setattr(self, f"container_By{axis_label}", container_by)
        hlay4.addWidget(container_by)

        # Finally add Home and Stop buttons
        for w in [btn_home, btn_stop]:
            hlay4.addWidget(w)
        hlay4.addStretch(1)
        ctrl_vlay.addLayout(hlay4)
        
        # ---- Row 5: Limit-switch indicators ---------
        hlay5 = QtWidgets.QHBoxLayout()
        lbl_rev = QtWidgets.QLabel("Rev Limit:")
        cb_rev = QtWidgets.QCheckBox()
        cb_rev.setEnabled(False)
        setattr(self, f"checkbox_RevLimit{axis_label}", cb_rev)
        hlay5.addWidget(lbl_rev)
        hlay5.addWidget(cb_rev)
 
        lbl_fwd = QtWidgets.QLabel("Fwd Limit:")
        cb_fwd = QtWidgets.QCheckBox()
        cb_fwd.setEnabled(False)
        setattr(self, f"checkbox_FwdLimit{axis_label}", cb_fwd)
        hlay5.addWidget(lbl_fwd)
        hlay5.addWidget(cb_fwd)
 
        ctrl_vlay.addLayout(hlay5)
        
        # ---- Row 6: Homing indicator & Status display ---
        hlay6 = QtWidgets.QHBoxLayout()
    
        lbl_homing = QtWidgets.QLabel("Homing:")
        cb_homing = QtWidgets.QCheckBox()
        cb_homing.setEnabled(False)
        setattr(self, f"checkbox_Homing{axis_label}", cb_homing)
        hlay6.addWidget(lbl_homing)
        hlay6.addWidget(cb_homing)
    
        # Reset motor button shares this row now:
        btn_reset = QtWidgets.QPushButton("Reset motor")
        btn_reset.setFocusPolicy(QtCore.Qt.NoFocus)
        setattr(self, f"button_ResetHome{axis_label}", btn_reset)
        btn_reset.clicked.connect(lambda _, ax=axis_label: self._on_reset_homing(ax))
        hlay6.addWidget(btn_reset)
        ctrl_vlay.addLayout(hlay6)

        # --- Connect signals & slots for this axis ---
        iface.sig_list_devices_updated.connect(lambda lst, ax=axis_label: self._on_list_devices_updated(ax, lst))
        iface.sig_connected.connect(lambda st, ax=axis_label: self._on_connection_status_change(ax, st))
        iface.sig_update_position.connect(lambda val, ax=axis_label: self._on_position_change(ax, val))
        iface.sig_step_size.connect(lambda val, ax=axis_label: self._on_step_size_change(ax, val))
        iface.sig_stage_info.connect(lambda info, ax=axis_label: self._on_stage_info_change(ax, info))

        # --- Wire up the buttons and mode changes ---
        # btn_connect.clicked.connect(lambda _, ax=axis_label: self._connect_clicked(ax))

        combo_mode.currentTextChanged.connect(lambda text, ax=axis_label: self._on_mode_changed(ax, text))

        # Arrow buttons: we connect pressed/released so we can implement continuous vs single:
        btn_move_neg.pressed.connect(lambda ax=axis_label: self._arrow_pressed(ax, -1))
        btn_move_pos.pressed.connect(lambda ax=axis_label: self._arrow_pressed(ax, +1))
        btn_move_neg.released.connect(lambda ax=axis_label: self._arrow_released(ax, -1))
        btn_move_pos.released.connect(lambda ax=axis_label: self._arrow_released(ax, +1))

        edit_step.returnPressed.connect(lambda ax=axis_label: self._press_enter_step(ax))
        edit_vel.returnPressed.connect(lambda ax=axis_label: self._press_enter_velocity(ax))
        edit_acc.returnPressed.connect(lambda ax=axis_label: self._press_enter_accel(ax))
        
        edit_pos.returnPressed.connect(lambda ax=axis_label: self._move_to_position(ax))
        edit_pos.returnPressed.disconnect()
        edit_pos.editingFinished.connect(lambda ax=axis_label: self._move_to_position(ax))
        edit_step.returnPressed.disconnect()
        edit_step.editingFinished.connect(lambda ax=axis_label: self._press_enter_step(ax))
        edit_vel.returnPressed.disconnect()
        edit_vel.editingFinished.connect(lambda ax=axis_label: self._press_enter_velocity(ax))
        edit_acc.returnPressed.disconnect()
        edit_acc.editingFinished.connect(lambda ax=axis_label: self._press_enter_accel(ax))
        
        btn_home.clicked.connect(lambda _, ax=axis_label: self._home_clicked(ax))
        btn_stop.clicked.connect(lambda _, ax=axis_label: self._stop_clicked(ax))
        
        btn_move_neg      .setFocusPolicy(QtCore.Qt.NoFocus)
        btn_move_pos       .setFocusPolicy(QtCore.Qt.NoFocus)
        btn_home          .setFocusPolicy(QtCore.Qt.NoFocus)
        btn_stop          .setFocusPolicy(QtCore.Qt.NoFocus)
        btn_reset    .setFocusPolicy(QtCore.Qt.NoFocus)

        # Initially hide/show the Jog options row...
        if combo_mode.currentText() == "Drive":
            widget_row3.hide()
        else:
            widget_row3.show()
    
        #  Reserve full space up front with a QStackedWidget 
        placeholder = QtWidgets.QWidget()
        stack = QStackedWidget()
        stack.addWidget(placeholder)     # index 0: empty placeholder
        stack.addWidget(container)       # index 1: your real controls
        stack.setCurrentIndex(0)         # start by showing the placeholder
        stack.setSizePolicy(QSizePolicy.Preferred,
                            QSizePolicy.Preferred)
        setattr(self, f"stack_Controls{axis_label}", stack)
    
        vlay.addWidget(stack)
        return grp

    # ----------------------------------------------------------------------------
    # Handlers for list update, connection status, position, etc.

    @QtCore.pyqtSlot(str, float)
    def on_position_updated(self, axis: str, pos_mm: float):
        iface = getattr(self, f"iface_{axis}")
        if not iface.connected_device_name or not iface.instrument.connected:
            return
        edit = getattr(self, f"edit_Position{axis}", None)
        # only overwrite if the user is NOT actively editing
        if edit and not edit.hasFocus():
            edit.setText(f"{pos_mm * MM_TO_UM:.3f}")
    
    @QtCore.pyqtSlot(str, bool, bool)
    def on_limit_switches(self, axis: str, rev_active: bool, fwd_active: bool):
        iface = getattr(self, f"iface_{axis}")
        if not iface.connected_device_name or not iface.instrument.connected:
            return
        cb_rev = getattr(self, f"checkbox_RevLimit{axis}", None)
        cb_fwd = getattr(self, f"checkbox_FwdLimit{axis}", None)
        if cb_rev:
            cb_rev.setChecked(rev_active)
        if cb_fwd:
            cb_fwd.setChecked(fwd_active)
    
    @QtCore.pyqtSlot(str)
    def _on_motion_settled(self, axis: str):
        btn = getattr(self, f"button_Stop{axis}", None)
        if btn:
            btn.setEnabled(False)
        iface = getattr(self, f"iface_{axis}")
        iface.sig_change_moving_status.emit(iface.SIG_MOVEMENT_ENDED)
    
    @QtCore.pyqtSlot(str, float, float)
    def _on_limits_update(self, axis: str, mn: float, mx: float):
        # piggy-back on your existing stage-info handler
        self._on_stage_info_change(axis, [mn, mx, None, None])

    def _on_list_devices_updated(self, axis: str, lst: list):
        combo: QtWidgets.QComboBox = getattr(self, f"combo_Device{axis}")
        combo.clear()
    
        # pick the serial we’re allowed to show for this axis
        assigned = self.assigned_serials.get(axis)
        if not assigned:
            return
    
        # lst entries look like "SERIAL --> MODEL"
        filtered = [item for item in lst
                    if item.startswith(f"{assigned} :") or item.split(' --> ')[0] == assigned]
        for i,v in enumerate(filtered):
            filtered[i] = filtered[i].split(' --> ')[0]
        combo.addItems(filtered)

    @QtCore.pyqtSlot(str, bool)
    def _on_connection_status_change(self, axis: str, connected: bool):
        # btn_connect = getattr(self, f"button_Connect{axis}")
        status_lbl = getattr(self, f"label_Status{axis}")

        # toggle Connect ↔ Disconnect text
        # if connected:
        #     btn_connect.setText(f"Disconnect {axis}")
        # else:
        #     btn_connect.setText(f"Connect {axis}")

        stack = getattr(self, f"stack_Controls{axis}", None)
        if stack is not None:
            stack.setCurrentIndex(1 if connected else 0)

        # update the status text
        if not connected:
            status_lbl.setText("Not connected")
        else:
            status_lbl.setText("Idle")
            
        # now update our “Go to Preset” button
        self._update_preset_button_state()

    def _update_preset_button_state(self):
        """Enable Go to Preset only when both X and Y are connected."""
        ok = bool(self.iface_x.connected_device_name and
                  self.iface_y.connected_device_name)
        self.button_GoPreset.setEnabled(ok)

    def _on_position_change(self, axis: str, val_mm: float):
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_Position{axis}")
        # only update display if the user isn’t actively editing
        if edit and not edit.hasFocus():
            val_disp = val_mm * MM_TO_UM
            edit.setText(f"{val_disp:.3f}")

    def _on_step_size_change(self, axis: str, val_mm: float):
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_StepSize{axis}")
        val_disp = val_mm * MM_TO_UM
        edit.setText(f"{val_disp:.3f}")
            
    @QtCore.pyqtSlot(str, bool, bool)
    def _on_hardware_status_update(self, axis: str, in_motion: bool, homed: bool):
        iface      = getattr(self, f"iface_{axis}")
        lbl_status = getattr(self, f"label_Status{axis}", None)
        if not iface.connected_device_name or not iface.instrument.connected:
            if lbl_status:
                lbl_status.setText("Not connected")
            return
        cb_homing  = getattr(self, f"checkbox_Homing{axis}", None)
        btn_stop   = getattr(self, f"button_Stop{axis}",    None)
        now = time.time()
    
        # 1) If the motor begins moving, start polling its position:
        if in_motion:
            self.poller.start_poll(axis)
    
        # 2) Update Homing checkbox & Status label as before:
        # only treat as actual homing if the user requested it
        # any in_motion after a Home click is “Homing”
        is_homing = in_motion and self._homing_requested.get(axis, False)
        # any other in_motion is a normal move
        is_moving = in_motion and not is_homing
    
        if cb_homing:     cb_homing.setChecked(is_homing)
        if lbl_status:
            lbl_status.setText(
                "Homing" if is_homing else
                "Moving" if is_moving else
                "Idle"
            )
    
        # 3) Enable Stop button for any motion:
        if btn_stop:
            btn_stop.setEnabled(in_motion)

    def _on_stage_info_change(self, axis: str, info: list):
        pass  # no stage info fields shown currently
        
    @QtCore.pyqtSlot(str)
    def _on_reset_homing(self, axis: str):
        """
        Simple channel toggle to clear any stuck homing bit:
         1) disable the HW channel
         2) re-enable it
         3) clear the PollerThread flag so UI sees Idle immediately
        """
        iface = getattr(self, f"iface_{axis}")
        if not iface.connected_device_name:
            QtWidgets.QMessageBox.warning(self, "Reset Homing",
                                          f"Axis {axis}: not connected")
            return

        try:
            # 1) drop any active polling on that axis
            #    (so when we re-enable it, the UI will poll it fresh)
            self.poller._poll_flags[axis] = False

            # 2) toggle the channel completely off:on
            iface.instrument.disable()   # clears any latched homing bit
            iface.instrument.enable()
            # we’ve interrupted/cleared any hanging home—stop treating
            # future moves as homing
            self._homing_requested[axis] = False

            QtWidgets.QMessageBox.information(
                self, "Reset Homing",
                f"Axis {axis}: channel disabled and re-enabled successfully."
            )
        except Exception as e:
            QtWidgets.QMessageBox.critical(
                self, "Reset Homing Error",
                f"Axis {axis}: could not toggle channel:\n{e}"
            )

    def _connect_clicked(self, axis: str):
        combo: QtWidgets.QComboBox = getattr(self, f"combo_Device{axis}")
        iface: interface = getattr(self, f"iface_{axis}")
        
        # BLOCK any polling while we mutate this interface
        iface._lock.acquire()
        try:
            text = combo.currentText()
            if iface.connected_device_name == "":
                iface.connect_device(text)
            else:
                # stop polling & disconnect
                self.poller.stop_poll(axis)
                iface.disconnect_device()
                # clear the stored name so poller/gates skip this axis
                iface.connected_device_name = ""
                # update your status label (replace with your widget’s name)
                getattr(self, f"label_Status{axis}").setText("Not connected")
    
                # purge any stale _serial_number so our guard fails
                if hasattr(iface.instrument, '_serial_number'):
                    del iface.instrument._serial_number
    
                # rebuild the driver wrapper cleanly
                if iface.use_virtual:
                    iface.instrument = driver_virtual.pyThorlabsAPT()
                else:
                    iface.instrument = driver_real.pyThorlabsAPT()
                iface.refresh_list_devices()
        finally:
            iface._lock.release()

    def _on_mode_changed(self, axis: str, new_mode: str):
        """
        Called when the Mode combo changes (Jog <-> Drive).
        If new_mode == "Drive", hide the Jog sub-mode row; else show it.
        Also stop any active position-poll timers for this axis.
        """
        widget_row3: QtWidgets.QWidget = getattr(self, f"widget_JogOptions{axis}")
        # Show or hide the jog options row
        if new_mode == "Drive":
            widget_row3.hide()
            # stop live‐polling this axis
            self.poller.stop_poll(axis)
        else:
            widget_row3.show()

    def _press_enter_position(self, axis: str):
        """
        User typed a new Position (in µm). Convert to mm and send.
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_Position{axis}")
        # Reject anything the validator won’t accept:
        if not edit.hasAcceptableInput():
            QtWidgets.QMessageBox.warning(
                self, "Invalid characters",
                f"{axis}-axis: please enter only digits and at most one decimal point for Position."
            )
            edit.selectAll(); edit.setFocus()
            return
        iface: interface = getattr(self, f"iface_{axis}")
        if iface.connected_device_name == "":
            return
        try:
            val_disp = float(edit.text())       # µm
            pos_mm   = val_disp / MM_TO_UM      # mm
            iface.set_position(str(pos_mm))
        except ValueError:
            pass
        self.window().activateWindow()
        self.setFocus(QtCore.Qt.TabFocusReason)
        
    def _move_to_position(self, axis: str):
        """
        Read the user-typed value from edit_Position{axis},
        convert from µm to mm, and command the motor.
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_Position{axis}")
        try:
            val_disp = float(edit.text())           # µm
        except ValueError:
            return
        # convert display µm back to mm
        pos_mm = val_disp / MM_TO_UM
        iface: interface = getattr(self, f"iface_{axis}")
        self.window().activateWindow()
        self.setFocus(QtCore.Qt.TabFocusReason)
        # only issue if device is connected
        if iface.connected_device_name:
            iface.set_position(str(pos_mm))

    def _press_enter_step(self, axis: str):
        """
        User typed a new Step‐Size (in µm). Convert to mm and send.
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_StepSize{axis}")
        if not edit.hasAcceptableInput():
            QtWidgets.QMessageBox.warning(
                self, "Invalid characters",
                f"{axis}-axis: please enter only digits and at most one decimal point for Step size."
            )
            edit.selectAll(); edit.setFocus()
            return
        iface: interface = getattr(self, f"iface_{axis}")
        try:
            val_disp = float(edit.text())        # µm
            step_mm  = val_disp / MM_TO_UM       # mm
            iface.set_step_size(str(step_mm))
        except ValueError:
            pass
        self.window().activateWindow()
        self.setFocus(QtCore.Qt.TabFocusReason)
        
    def _press_enter_velocity(self, axis: str):
        """User typed a new Velocity (in µm/s). Convert to mm/s and send."""
        validated = self._read_jog_params(axis)
        if validated is None:
            return
        vel_mm_s, acc_mm_s2 = validated
        iface = getattr(self, f"iface_{axis}")
        iface.set_velocity_profile(vel_mm_s, acc_mm_s2)
        self.window().activateWindow()
        self.setFocus(QtCore.Qt.TabFocusReason)

    def _press_enter_accel(self, axis: str):
        """User typed a new Acceleration (in µm/s²). Convert to mm/s² and send."""
        validated = self._read_jog_params(axis)
        if validated is None:
            return
        vel_mm_s, acc_mm_s2 = validated
        iface = getattr(self, f"iface_{axis}")
        iface.set_velocity_profile(vel_mm_s, acc_mm_s2)
        self.window().activateWindow()
        self.setFocus(QtCore.Qt.TabFocusReason)

    def _home_clicked(self, axis: str):
        iface: interface = getattr(self, f"iface_{axis}")
        # mark this as a “real” homing move
        self._homing_requested[axis] = True
        iface.home()

    def _stop_clicked(self, axis: str):
        iface: interface = getattr(self, f"iface_{axis}")
        iface.stop_any_movement()
        
    def _stop_all(self):
        """Stop any motion on X, Y, and Z immediately."""
        for ax in ("X", "Y", "Z"):
            iface = getattr(self, f"iface_{ax}")
            try:
                iface.stop_any_movement()
            except Exception:
                # you can log or ignore if one axis isn’t moving
                pass

    def _read_jog_params(self, axis: str):
        # grab the widgets
        vel_field = getattr(self, f"edit_Velocity{axis}")
        acc_field = getattr(self, f"edit_Accel{axis}")
    
        # 1) Make sure the current text is acceptable to the validator
        for field, name in ((vel_field, "velocity"), (acc_field, "acceleration")):
            if not field.hasAcceptableInput():
                QtWidgets.QMessageBox.warning(
                    self, "Invalid characters",
                    f"{axis}-axis: please enter only digits and at most one decimal point for {name}."
                )
                field.selectAll()
                field.setFocus()
                return None
    
        # 2) Now parse and clamp as before…
        try:
            disp_vel = float(vel_field.text())
        except ValueError:
            # this shouldn’t normally happen, but just in case
            QtWidgets.QMessageBox.warning(
                self, "Invalid input",
                f"{axis}-axis: velocity isn’t a valid number."
            )
            vel_field.selectAll(); vel_field.setFocus()
            return None
    
        if disp_vel < self._min_vel_disp[axis]:
            QtWidgets.QMessageBox.warning(
                self, "Value too small",
                f"{axis}-axis minimum velocity is {self._min_vel_disp[axis]} µm/s.\n"
                "Using this minimum value instead."
            )
            vel_field.setText(f"{self._min_vel_disp[axis]:.1f}")
            disp_vel = self._min_vel_disp[axis]
    
        vel_mm_s = disp_vel * UM_TO_MM
    
        try:
            disp_acc = float(acc_field.text())
        except ValueError:
            QtWidgets.QMessageBox.warning(
                self, "Invalid input",
                f"{axis}-axis: acceleration isn’t a valid number."
            )
            acc_field.selectAll(); acc_field.setFocus()
            return None
    
        if disp_acc < self._min_acc_disp[axis]:
            QtWidgets.QMessageBox.warning(
                self, "Value too small",
                f"{axis}-axis minimum acceleration is {self._min_acc_disp[axis]} µm/s².\n"
                "Using this minimum value instead."
            )
            acc_field.setText(f"{self._min_acc_disp[axis]:.1f}")
            disp_acc = self._min_acc_disp[axis]
    
        acc_mm_s2 = disp_acc * UM_TO_MM
    
        return vel_mm_s, acc_mm_s2
    
    def _start_continuous(self, axis: str, direction: int):
        """
        Programs the motor ramp (velocity + accel) and starts continuous motion,
        then tells the PollerThread to begin polling this axis.
        """
        iface = getattr(self, f"iface_{axis}")
        vel, acc = self._read_jog_params(axis)
        iface.set_velocity_profile(vel, acc)
        iface.move_velocity_continuous(direction)
            
    def _apply_preset_dict(self, data: dict, show_message: bool=True):
        # 1) Extract—and treat data['x'],['y'] as µm:
        x_um   = float(data['x'])
        y_um   = float(data['y'])
        vel_um = float(data['velocity'])
        acc_um = float(data['acceleration'])
        # 2) Convert to mm (you already have UM_TO_MM = 1e-3)
        x_mm   = x_um  * UM_TO_MM
        y_mm   = y_um  * UM_TO_MM
        vel_mm = vel_um * UM_TO_MM
        acc_mm = acc_um * UM_TO_MM
        # 3) Store
        self._preset = dict(x_mm=x_mm, y_mm=y_mm, vel=vel_mm, acc=acc_mm)
        # 4) Update UI
        self.label_PresetStatus.setText(f"Preset: {x_um:.0f}×{y_um:.0f} µm")
        if show_message:
            QtWidgets.QMessageBox.information(
                self, "Preset Loaded",
                f"X={x_um:.0f} µm, Y={y_um:.0f} µm\n"
                f"Vel={vel_um:.1f} µm/s, Acc={acc_um:.1f} µm/s²"
            )

            
    def load_preset(self):
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load Motion Preset", "", "JSON files (*.json);;All Files (*)"
        )
        if not fname:
            return

        try:
            with open(fname, 'r') as f:
                data = json.load(f)
            self._apply_preset_dict(data, show_message=True)
        except Exception as e:
            QtWidgets.QMessageBox.warning(
                self, "Load Failed", f"Could not parse preset: {e}"
            )
            
    def go_preset(self):
        if not hasattr(self, '_preset'):
            QtWidgets.QMessageBox.warning(
                self, "No Preset", "Please load a preset file first."
            )
            return
        
        pr = self._preset
        
        # X axis move
        self.iface_x.set_velocity_profile(pr['vel'], pr['acc'])
        self.iface_x.instrument.position = pr['x_mm']
        self.poller.start_poll('X')
        
        # Y axis move
        self.iface_y.set_velocity_profile(pr['vel'], pr['acc'])
        self.iface_y.instrument.position = pr['y_mm']
        self.poller.start_poll('Y')
        
        self.window().activateWindow()
        self.setFocus(QtCore.Qt.TabFocusReason)
    
    def _show_manage_dialog(self):
        dlg = ConnectDevicesDialog(self, parent=self)
        dlg.exec_()

    def _arrow_pressed(self, axis: str, direction: int):
        """
        Called when < or > button is pressed.  
        Decide behavior based on Mode and Sub‐mode.
        """
        iface: interface = getattr(self, f"iface_{axis}")
        if iface.connected_device_name == "":
            return

        mode = getattr(self, f"combo_Mode{axis}").currentText()

        # --- Jog Mode ---
        if mode == "Jog":
            # (a) SINGLE‐CLICK: use the shared helper to read params, then one step
            if getattr(self, f"radio_Single{axis}").isChecked():
                if not iface.is_device_moving():
                    # pull in the latest user-typed step size
                    self._press_enter_step(axis)
                    vel, acc = self._read_jog_params(axis)
                    iface.set_velocity_profile(vel, acc)
                    iface.move_single_step(direction)
                return

            # Continuous:
            if getattr(self, f"radio_Cont{axis}").isChecked():
                self._start_continuous(axis, direction)
                return

        # --- Drive Mode (Z axis continuous) ---
        if mode == "Drive" and axis == "Z":
            self._start_continuous(axis, direction)
            return

    def _arrow_released(self, axis: str, direction: int):
        """
        Called when < or > button is released.  Stop any QTimer for that axis.
        """
        
        # # CRASHER 
        # jog_timer = self.continuous_timers[axis]
        # if jog_timer.isActive():
        #     jog_timer.stop()
        #     jog_timer.disconnect()
        # # CRASHER 
        
        mode = getattr(self, f"combo_Mode{axis}").currentText()
        single  = getattr(self, f"radio_Single{axis}").isChecked()
        # if we’re in single-click jog, *don’t* stop the move here
        if mode == "Jog" and single:
            return
        
        # Gracefully decelerate the motor if it was in “move_velocity” mode:
        iface: interface = getattr(self, f"iface_{axis}")
        if iface.connected_device_name:
            try:
                # read back whatever max-vel is currently set
                threshold_mm_s = 100.0 * UM_TO_MM
                vel, acc = self._read_jog_params(axis)
    
                if vel > threshold_mm_s:
                    # above threshold : smooth, profiled stop
                    iface.instrument.stop_profiled()
                else:
                    # below threshold : zero-vel “sudden” stop
                    # (set max_vel = 0 so motor brakes instantly)
                    iface.set_velocity_profile(0.0, acc)
                    iface.move_velocity_continuous(direction)
                    iface.instrument.stop_profiled()
            except Exception as e:
                print(f"[ERROR] stop failed for {axis} motor: {e}")

            # now catch *all* overshoot by polling position stability
        self._poll_until_settled(axis)

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        """
        Handle keyboard shortcuts in a manner analogous to mouse‐clicks:
        A/D : X ; W/S : Y ; Up/Down : Z ; H/J/K : Home ; Space : Stop
        """
        key = event.key()

        #  X-axis: A / D 
        if key in (QtCore.Qt.Key_A, QtCore.Qt.Key_D) and not event.isAutoRepeat():
            direction = -1 if key == QtCore.Qt.Key_A else +1
            self._arrow_pressed("X", direction)
            return

        #  Y-axis: W / S 
        if key in (QtCore.Qt.Key_W, QtCore.Qt.Key_S) and not event.isAutoRepeat():
            direction = +1 if key == QtCore.Qt.Key_W else -1
            self._arrow_pressed("Y", direction)
            return

        #  Z-axis: Up / Down arrows 
        if key in (QtCore.Qt.Key_Up, QtCore.Qt.Key_Down) and not event.isAutoRepeat():
            direction = +1 if key == QtCore.Qt.Key_Up else -1
            self._arrow_pressed("Z", direction)
            return

        # Home keys: H / J / K
        if key == QtCore.Qt.Key_H:
            self._home_clicked('X')
            return
        if key == QtCore.Qt.Key_J:
            self._home_clicked('Y')
            return
        if key == QtCore.Qt.Key_K:
            self._home_clicked('Z')
            return

        # Spacebar : Stop All
        if key == QtCore.Qt.Key_Space:
            for ax in ("X", "Y", "Z"):
                iface: interface = getattr(self, f"iface_{ax}")
                iface.stop_any_movement()
                self._poll_until_settled(ax)
            return
        super().keyPressEvent(event)
        
    def keyReleaseEvent(self, event: QtGui.QKeyEvent):
        """
        When the user releases A/D, W/S, or Up/Down, stop that axis.
        Ignore auto-repeat releases so we only stop once.
        """
        key = event.key()

        # X-axis: A or D released
        if key in (QtCore.Qt.Key_A, QtCore.Qt.Key_D) and not event.isAutoRepeat():
            direction = -1 if key == QtCore.Qt.Key_A else +1
            self._arrow_released("X", direction)
            return

        # Y-axis: W or S released
        if key in (QtCore.Qt.Key_W, QtCore.Qt.Key_S) and not event.isAutoRepeat():
            direction = +1 if key == QtCore.Qt.Key_W else -1
            self._arrow_released("Y", direction)
            return
        
        # Z-axis: Up or Down released
        if key in (QtCore.Qt.Key_Up, QtCore.Qt.Key_Down) and not event.isAutoRepeat():
            direction = +1 if key == QtCore.Qt.Key_Up else -1
            self._arrow_released("Z", direction)
            return

        super().keyReleaseEvent(event)
        
    def _poll_until_settled(self,
                            axis: str,
                            poll_interval: int = 200,
                            tol_um: float = 0.05,
                            stable_count_needed: int = 3):
        """
        After stopping, read position periodically until it changes by < tol_um
        for stable_count_needed successive polls.
        """
        iface = getattr(self, f"iface_{axis}")
        # one-off timer to watch for “settled” behavior
        last_pos_mm = iface.read_position()
        stable_count = 0

        timer = QtCore.QTimer(self)
        timer.setTimerType(QtCore.Qt.CoarseTimer)
        timer.setInterval(poll_interval)

        def check_stability():
            nonlocal last_pos_mm, stable_count
            pos_mm = iface.read_position()
            # immediately update the UI
            self.on_position_updated(axis, pos_mm)
            moved = abs((pos_mm - last_pos_mm) * MM_TO_UM)
            if moved < tol_um:
                stable_count += 1
            else:
                stable_count = 0
                last_pos_mm = pos_mm
            if stable_count >= stable_count_needed:
                timer.stop()
                timer.deleteLater()

        timer.timeout.connect(check_stability)
        timer.start()

# 
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("pyThorlabsAPT – 3-Axis")

    def closeEvent(self, event):
        # Emergency stop all motors on GUI close
        central = self.centralWidget()
        for axis in ("X", "Y", "Z"):
            try:
                iface: interface = getattr(central, f"iface_{axis}")
                iface.stop_any_movement()
            except Exception as e:
                # If something goes wrong, at least log it before exit
                print(f"[ERROR] Could not stop axis {axis}: {e}")
        try:
            central.poller.stop()
        except Exception as e:
            print(f"[ERROR] Could not stop PollerThread: {e}")
            
        # tear down APT library so next launch starts clean
        try:
            apt_core._cleanup()
        except Exception as e:
            print(f"[ERROR] apt_core._cleanup() failed: {e}")
            
            # Proceed with normal close
        super().closeEvent(event)


def main():
    print(f"[DEBUG] {time.strftime('%H:%M:%S')}  :  In main(), before anything else.")
    parser = argparse.ArgumentParser(description="pyThorlabsAPT – 3-Axis multi-motor GUI")
    parser.add_argument("-s", "--decrease_verbose", action="store_true", help="Decrease verbosity")
    parser.add_argument("-virtual", action="store_true", help="Use virtual driver (no hardware).")
    args = parser.parse_args()
    use_virtual = args.virtual

    app = QtWidgets.QApplication(sys.argv)

    # Instantiate three interfaces, one for X, one for Y, one for Z:
    iface_x = interface(use_virtual=use_virtual)
    iface_y = interface(use_virtual=use_virtual)
    iface_z = interface(use_virtual=use_virtual)

    window = MainWindow()
    gui3 = MultiAxisGui(iface_x, iface_y, iface_z, parent=window)
    window.setCentralWidget(gui3)
    window.resize(700, 500)

    print(f"[DEBUG] {time.strftime('%H:%M:%S')}  :  About to call window.show() and app.exec_().")
    window.show()

    # Ensure interfaces close when the app is quitting:
    app.aboutToQuit.connect(iface_x.close)
    app.aboutToQuit.connect(iface_y.close)
    app.aboutToQuit.connect(iface_z.close)
    # Ensure the window (and its closeEvent cleanup) runs on app.quit
    app.aboutToQuit.connect(window.close)

    sys.exit(app.exec_())


if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("")
        print(" ERROR: Unhandled exception in pyThorlabsAPT.main()")
        print("")
        traceback.print_exc()
        print("")
        input("Press Enter to exit…")

# Alias for the console entrypoint:
gui = MultiAxisGui

# 
