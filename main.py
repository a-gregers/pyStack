import os
import sys
import time
import traceback
import argparse
import PyQt5
from PyQt5 import QtWidgets, QtGui, QtCore

# Ensure Qt finds its platform plugins:
dirname = os.path.dirname(PyQt5.__file__)
plugin_path = os.path.join(dirname, 'plugins', 'platforms')
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = plugin_path

# Import the real & virtual drivers:
import pyThorlabsAPT.driver as driver_real
import pyThorlabsAPT.driver_virtual as driver_virtual

# ───────────────────────────────────────────────────────────────
# Catch any exception inside Qt callbacks & print it to console
def exception_hook(exctype, value, tb):
    print("────────────────────────────────────────────────────────")
    print(" Uncaught exception:")
    traceback.print_exception(exctype, value, tb)
    print("────────────────────────────────────────────────────────")
    input("Press Enter to exit…")

sys._excepthook = sys.excepthook
sys.excepthook = exception_hook
# ───────────────────────────────────────────────────────────────

print(f"[DEBUG] {time.strftime('%H:%M:%S')}  →  Entering pyThorlabsAPT (main module).")

UM_TO_MM = 1e-3      # microns → millimeters
MM_TO_UM = 1e3       # millimeters → microns

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
        QtCore.QTimer.singleShot(0, self.refresh_list_devices)
        
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

        # threshold = 100 µm/s → in mm/s:
        threshold_mm_s = 100.0 * UM_TO_MM
        try:
            # read back whatever max-vel is currently set
            _, current_acc, current_maxv = self.instrument.get_velocity_parameters()
            print(f"[DEBUG] stop_any_movement: max_v = {current_maxv:.4f} mm/s   threshold = {threshold_mm_s:.4f} mm/s")

            if current_maxv > threshold_mm_s:
                # above threshold → smooth, profiled stop
                self.instrument.stop_profiled()
            else:
                # below threshold → zero-vel “sudden” stop
                # (set max_vel = 0 so motor brakes instantly)
                # self.instrument.set_velocity_profile(0.0, current_acc, 0.0)
                self.set_velocity_profile(0.0, current_acc)
                self.move_velocity_continuous(1)
                self.instrument.stop_profiled()
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
        Send the home command. Poll until it finishes, then emit read_position.
        """
        try:
            if self.is_device_moving():
                return
            self.sig_change_homing_status.emit(self.SIG_HOMING_STARTED)
            self.instrument.move_home()
            self._poll_home()
        except Exception:
            self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)

    def _poll_home(self):
        """
        Internal: poll until instrument.is_in_motion becomes False,
        then emit read_position() and "homing ended".
        """
        try:
            if self.instrument.is_in_motion:
                # Emit current position so GUI updates during homing
                self.read_position()
                QtCore.QTimer.singleShot(50, self._poll_home)
            else:
                # Final readout when homing finishes
                self.read_position()
                self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
                self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
        except Exception:
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
        except:
            pass

    def _poll_setpos_finished(self):
        try:
            if self.instrument.is_in_motion:
                QtCore.QTimer.singleShot(50, self._poll_setpos_finished)
            else:
                self.read_position()
                self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
        except:
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


# ───────────────────────────────────────────────────────────────────────────────
class MultiAxisGui(QtWidgets.QWidget):
    def __init__(self, iface_x: interface, iface_y: interface, iface_z: interface, parent=None):
        super().__init__(parent)

        self.iface_x = iface_x
        self.iface_y = iface_y
        self.iface_z = iface_z

        # Keep track of display‐unit multiplier (always mm for simplicity here):
        self.display_unit = "mm"
        
        # Create “position‐poll” timers so we can update the displayed position
        # while the motor is moving continuously. (Also one per axis.)
        self.position_poll_timers = {
            'X': QtCore.QTimer(self),
            'Y': QtCore.QTimer(self),
            'Z': QtCore.QTimer(self)
        }
        
        for poll_t in self.position_poll_timers.values():
            poll_t.setInterval(50)  # poll every 50 ms
            poll_t.timeout.connect(self._poll_position_update)
        
        # Top‐level layout: axis groups stacked vertically
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # Build X‐axis group:
        self.groupX = self._build_axis_group("X", self.iface_x, allow_drive=False)
        layout.addWidget(self.groupX)

        # Build Y‐axis group:
        self.groupY = self._build_axis_group("Y", self.iface_y, allow_drive=False)
        layout.addWidget(self.groupY)

        # Build Z‐axis group (allow drive):
        self.groupZ = self._build_axis_group("Z", self.iface_z, allow_drive=True)
        layout.addWidget(self.groupZ)

        # Tooltip for keyboard shortcuts:
        tips = (
            "Keyboard Shortcuts:\n"
            "  A / D  → Move X left / right  (or hold for continuous)\n"
            "  W / S  → Move Y forward / back  (or hold for continuous)\n"
            "  ↑ / ↓  → Move Z up / down  (or hold for continuous)\n"
            "  H / J / K → Home X / Home Y / Home Z\n"
            "  Space → Stop ALL motors\n"
        )
        self.setToolTip(tips)

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
        btn_refresh = QtWidgets.QPushButton(f"Refresh {axis_label}")
        btn_connect = QtWidgets.QPushButton(f"Connect {axis_label}")

        setattr(self, f"combo_Device{axis_label}", combo_dev)
        setattr(self, f"button_Refresh{axis_label}", btn_refresh)
        setattr(self, f"button_Connect{axis_label}", btn_connect)
        setattr(self, f"iface_{axis_label}", iface)

        for w in [label_dev, combo_dev, btn_refresh, btn_connect]:
            hlay1.addWidget(w)
        hlay1.addStretch(1)
        vlay.addLayout(hlay1)

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
        vlay.addLayout(hlay2)

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
        
        # Whenever they toggle sub-mode, show/hide the “By” container accordingly:
        radio_single.toggled.connect(lambda checked, ax=axis_label: getattr(self, f"container_By{ax}").setVisible(checked))
        radio_cont.toggled.connect(lambda checked, ax=axis_label: getattr(self, f"container_By{ax}").setVisible(False))
        
        setattr(self, f"edit_Velocity{axis_label}", edit_vel)
        setattr(self, f"edit_Accel{axis_label}", edit_acc)

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
        vlay.addWidget(widget_row3)

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

        setattr(self, f"edit_Position{axis_label}", edit_pos)
        setattr(self, f"button_MovePos{axis_label}", btn_move_pos)
        setattr(self, f"edit_StepSize{axis_label}", edit_step)
        setattr(self, f"button_Home{axis_label}", btn_home)
        setattr(self, f"button_Stop{axis_label}", btn_stop)

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
        vlay.addLayout(hlay4)
        
        self.installEventFilter(self)
        QtWidgets.qApp.installEventFilter(self)

        # --- Connect signals → slots for this axis ---
        iface.sig_list_devices_updated.connect(lambda lst, ax=axis_label: self._on_list_devices_updated(ax, lst))
        iface.sig_connected.connect(lambda st, ax=axis_label: self._on_connection_status_change(ax, st))
        iface.sig_update_position.connect(lambda val, ax=axis_label: self._on_position_change(ax, val))
        iface.sig_step_size.connect(lambda val, ax=axis_label: self._on_step_size_change(ax, val))
        iface.sig_change_moving_status.connect(lambda st, ax=axis_label: self._on_moving_state_change(ax, st))
        iface.sig_change_homing_status.connect(lambda st, ax=axis_label: self._on_homing_state_change(ax, st))
        iface.sig_stage_info.connect(lambda info, ax=axis_label: self._on_stage_info_change(ax, info))

        # --- Wire up the buttons and mode changes ---
        btn_refresh.clicked.connect(lambda _, ax=axis_label: self._refresh_clicked(ax))
        btn_connect.clicked.connect(lambda _, ax=axis_label: self._connect_clicked(ax))

        combo_mode.currentTextChanged.connect(lambda text, ax=axis_label: self._on_mode_changed(ax, text))

        # Arrow buttons: we connect pressed/released so we can implement continuous vs single:
        btn_move_neg.pressed.connect(lambda ax=axis_label: self._arrow_pressed(ax, -1))
        btn_move_pos.pressed.connect(lambda ax=axis_label: self._arrow_pressed(ax, +1))
        btn_move_neg.released.connect(lambda ax=axis_label: self._arrow_released(ax))
        btn_move_pos.released.connect(lambda ax=axis_label: self._arrow_released(ax))

        edit_pos.returnPressed.connect(lambda ax=axis_label: self._press_enter_position(ax))
        edit_step.returnPressed.connect(lambda ax=axis_label: self._press_enter_step(ax))
        edit_vel.returnPressed.connect(lambda ax=axis_label: self._press_enter_velocity(ax))
        edit_acc.returnPressed.connect(lambda ax=axis_label: self._press_enter_accel(ax))
        
        edit_pos.returnPressed.disconnect()
        edit_pos.editingFinished.connect(lambda ax=axis_label: self._press_enter_position(ax))
        edit_step.returnPressed.disconnect()
        edit_step.editingFinished.connect(lambda ax=axis_label: self._press_enter_step(ax))
        edit_vel.returnPressed.disconnect()
        edit_vel.editingFinished.connect(lambda ax=axis_label: self._press_enter_velocity(ax))
        edit_acc.returnPressed.disconnect()
        edit_acc.editingFinished.connect(lambda ax=axis_label: self._press_enter_accel(ax))
        
        btn_home.clicked.connect(lambda _, ax=axis_label: self._home_clicked(ax))
        btn_stop.clicked.connect(lambda _, ax=axis_label: self._stop_clicked(ax))

        # Initially hide/show the Jog options row depending on default Mode:
        if combo_mode.currentText() == "Drive":
            widget_row3.hide()
        else:
            widget_row3.show()

        return grp

    def _poll_position_update(self):
        """
        Called by each axis’s position_poll_timer whenever it’s active.
        We read and emit the latest position so the GUI field stays live.
        """
        # We must update X, Y, and Z if they are moving – and stop them if a limit switch is hit:
        for axis in ("X", "Y", "Z"):
            iface = getattr(self, f"iface_{axis}")
            if not (iface.connected_device_name and iface.is_device_moving()):
                continue

            # 1) Check hardware limit switches (reverse, forward):
            try:
                rev_code, fwd_code = iface.instrument.get_hardware_limit_switches()
                # HWLIMSWITCH_BREAKS (3) means active when continuity is broken
                if rev_code == 3 or fwd_code == 3:
                    # Immediately perform a profiled stop rather than overrunning
                    iface.stop_profiled_continuous()
                    continue
            except Exception as e:
                # If the check itself fails, warn but proceed with normal polling
                print(f"[WARN] Limit‐switch check failed on axis {axis}: {e}")

            # 2) Normal position update
            iface.read_position()

    # ───────────────────────────────────────────────────────────────────────────
    # Handlers for list update, connection status, position, etc.

    def _on_list_devices_updated(self, axis: str, lst: list):
        combo: QtWidgets.QComboBox = getattr(self, f"combo_Device{axis}")
        combo.clear()
        combo.addItems(lst)

    def _on_connection_status_change(self, axis: str, connected: bool):
        btn_connect: QtWidgets.QPushButton = getattr(self, f"button_Connect{axis}")
        if connected:
            btn_connect.setText(f"Disconnect {axis}")
        else:
            btn_connect.setText(f"Connect {axis}")

    def _on_position_change(self, axis: str, val_mm: float):
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_Position{axis}")
        val_disp = val_mm * MM_TO_UM
        edit.setText(f"{val_disp:.3f}")

    def _on_step_size_change(self, axis: str, val_mm: float):
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_StepSize{axis}")
        val_disp = val_mm * MM_TO_UM
        edit.setText(f"{val_disp:.3f}")

    def _on_moving_state_change(self, axis: str, status: int):
        pass

    def _on_homing_state_change(self, axis: str, status: int):
        pass

    def _on_stage_info_change(self, axis: str, info: list):
        pass  # no stage info fields shown currently

    def _refresh_clicked(self, axis: str):
        iface: interface = getattr(self, f"iface_{axis}")
        iface.refresh_list_devices()

    def _connect_clicked(self, axis: str):
        combo: QtWidgets.QComboBox = getattr(self, f"combo_Device{axis}")
        iface: interface = getattr(self, f"iface_{axis}")
        text = combo.currentText()
        if iface.connected_device_name == "":
            iface.connect_device(text)
        else:
            iface.disconnect_device()


    def _on_mode_changed(self, axis: str, new_mode: str):
        """
        Called when the Mode combo changes (Jog <-> Drive).
        If new_mode == "Drive", hide the Jog sub-mode row; else show it.
        """
        widget_row3: QtWidgets.QWidget = getattr(self, f"widget_JogOptions{axis}")
        if new_mode == "Drive":
            widget_row3.hide()
        else:
            widget_row3.show()


    def _press_enter_position(self, axis: str):
        """
        User typed a new Position (in µm). Convert to mm and send.
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_Position{axis}")
        iface: interface = getattr(self, f"iface_{axis}")
        if iface.connected_device_name == "":
            return
        try:
            val_disp = float(edit.text())       # µm
            pos_mm   = val_disp / MM_TO_UM      # → mm
            iface.set_position(str(pos_mm))
        except ValueError:
            pass
        self.window().activateWindow()
        self.setFocus(QtCore.Qt.TabFocusReason)

    def _press_enter_step(self, axis: str):
        """
        User typed a new Step‐Size (in µm). Convert to mm and send.
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_StepSize{axis}")
        iface: interface = getattr(self, f"iface_{axis}")
        try:
            val_disp = float(edit.text())        # µm
            step_mm  = val_disp / MM_TO_UM       # → mm
            iface.set_step_size(str(step_mm))
        except ValueError:
            pass
        self.window().activateWindow()
        self.setFocus(QtCore.Qt.TabFocusReason)
        
    def _press_enter_velocity(self, axis: str):
        """User typed a new Velocity (in µm/s). Convert → mm/s and send."""
        try:
            disp_vel = float(getattr(self, f"edit_Velocity{axis}").text())
            vel_mm_s = disp_vel * UM_TO_MM
            # grab the current accel so we don't clobber it:
            _, acc_mm_s2 = self._read_jog_params(axis)
            iface = getattr(self, f"iface_{axis}")
            iface.set_velocity_profile(vel_mm_s, acc_mm_s2)
        except ValueError:
            pass
        self.window().activateWindow()
        self.setFocus(QtCore.Qt.TabFocusReason)

    def _press_enter_accel(self, axis: str):
        """User typed a new Acceleration (in µm/s²). Convert → mm/s² and send."""
        try:
            disp_acc = float(getattr(self, f"edit_Accel{axis}").text())
            acc_mm_s2 = disp_acc * UM_TO_MM
            # grab the current velocity so we don't clobber it:
            vel_mm_s, _ = self._read_jog_params(axis)
            iface = getattr(self, f"iface_{axis}")
            iface.set_velocity_profile(vel_mm_s, acc_mm_s2)
        except ValueError:
            pass
        self.window().activateWindow()
        self.setFocus(QtCore.Qt.TabFocusReason)

    def _home_clicked(self, axis: str):
        iface: interface = getattr(self, f"iface_{axis}")
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
        """
        Reads edit_Velocity{axis} (µm/s) and edit_Accel{axis} (µm/s²),
        converts to mm units, falls back to last-stored or tiny default.
        """
        try:
            disp_vel = float(getattr(self, f"edit_Velocity{axis}").text())
            vel_mm_s = disp_vel * UM_TO_MM
        except Exception:
            vel_mm_s = 0.01
        try:
            disp_acc = float(getattr(self, f"edit_Accel{axis}").text())
            acc_mm_s2 = disp_acc * UM_TO_MM
        except Exception:
            acc_mm_s2 = 0.01
        return vel_mm_s, acc_mm_s2
    
    def _start_continuous(self, axis: str, direction: int):
        """
        Programs the motor ramp (velocity + accel) and starts continuous motion,
        then starts the position-poll timer.
        """
        iface = getattr(self, f"iface_{axis}")
        vel, acc = self._read_jog_params(axis)
        iface.set_velocity_profile(vel, acc)
        iface.move_velocity_continuous(direction)
        poll_t = self.position_poll_timers[axis]
        if not poll_t.isActive():
            poll_t.start()

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
                    poll_t = self.position_poll_timers[axis]
                    if not poll_t.isActive():
                        poll_t.start()
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
        
        # CRASHER ─────────────────────────────────────────────────────
        # jog_timer = self.continuous_timers[axis]
        # if jog_timer.isActive():
        #     jog_timer.stop()
        #     jog_timer.disconnect()
        # CRASHER ─────────────────────────────────────────────────────
        
        mode = getattr(self, f"combo_Mode{axis}").currentText()
        single  = getattr(self, f"radio_Single{axis}").isChecked()
        # if we’re in single-click jog, *don’t* stop the move here
        if mode == "Jog" and single:
            return
        
        # Gracefully decelerate the motor if it was in “move_velocity” mode:
        iface: interface = getattr(self, f"iface_{axis}")
        if iface.connected_device_name and iface.is_device_moving():
            try:
                # read back whatever max-vel is currently set
                threshold_mm_s = 100.0 * UM_TO_MM
                _, current_acc, current_maxv = iface.instrument.get_velocity_parameters()
    
                if current_maxv > threshold_mm_s:
                    # above threshold → smooth, profiled stop
                    iface.instrument.stop_profiled()
                else:
                    # below threshold → zero-vel “sudden” stop
                    # (set max_vel = 0 so motor brakes instantly)
                    iface.set_velocity_profile(0.0, current_acc)
                    iface.move_velocity_continuous(direction)
                    iface.instrument.stop_profiled()
            except Exception as e:
                print(f"[ERROR] stop failed for {axis} motor: {e}")

            # (3) Keep polling until the motor actually stops before we fully clear “moving”:
            #     We do this by starting a short QTimer that watches `is_in_motion` → false.
            stop_poll = QtCore.QTimer(self)
            stop_poll.setInterval(50)  # poll every 50 ms
            def check_stopped():
                if not iface.is_device_moving():
                    stop_poll.stop()
                    stop_poll.deleteLater()
                    iface.sig_change_moving_status.emit(iface.SIG_MOVEMENT_ENDED)
                # else, keep waiting…
            stop_poll.timeout.connect(check_stopped)
            stop_poll.start()

        # (4) Finally, stop the “position poll” timer so we don’t keep updating once it’s done:
        poll_t = self.position_poll_timers[axis]
        if poll_t.isActive():
            poll_t.stop()

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        """
        Handle keyboard shortcuts in a manner analogous to mouse‐clicks:
        A/D → X ; W/S → Y ; Up/Down → Z ; H/J/K → Home ; Space → Stop
        """
        key = event.key()

        # ─── X-axis: A / D ─────────────────────────────────────────────────────
        if key in (QtCore.Qt.Key_A, QtCore.Qt.Key_D) and not event.isAutoRepeat():
            direction = -1 if key == QtCore.Qt.Key_A else +1
            self._arrow_pressed("X", direction)
            return

        # ─── Y-axis: W / S ─────────────────────────────────────────────────────
        if key in (QtCore.Qt.Key_W, QtCore.Qt.Key_S) and not event.isAutoRepeat():
            direction = +1 if key == QtCore.Qt.Key_W else -1
            self._arrow_pressed("Y", direction)
            return

        # ─── Z-axis: Up / Down arrows ───────────────────────────────────────────
        if key in (QtCore.Qt.Key_Up, QtCore.Qt.Key_Down) and not event.isAutoRepeat():
            direction = +1 if key == QtCore.Qt.Key_Up else -1
            self._arrow_pressed("Z", direction)
            return

        # Home keys: H / J / K
        if key == QtCore.Qt.Key_H:
            self.iface_x.home()
            return
        if key == QtCore.Qt.Key_J:
            self.iface_y.home()
            return
        if key == QtCore.Qt.Key_K:
            self.iface_z.home()
            return

        # Spacebar → Stop All
        if key == QtCore.Qt.Key_Space:
            for ax in ("X", "Y", "Z"):
                iface: interface = getattr(self, f"iface_{ax}")
                iface.stop_any_movement()
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


    # ───────────────────────────────────────────────────────────────────────────
    def _on_mode_changed(self, axis: str, new_mode: str):
        """
        Show or hide the Jog sub‐mode row when Mode changes.
        """
        widget_row3: QtWidgets.QWidget = getattr(self, f"widget_JogOptions{axis}")
        if new_mode == "Drive":
            widget_row3.hide()
            # Ensure no jog timer is active
            timer = self.continuous_timers[axis]
            if timer.isActive():
                timer.stop()
                timer.disconnect()
        else:
            widget_row3.show()
            # Ensure no drive timer is active
            timer = self.continuous_timers[axis]
            if timer.isActive():
                timer.stop()
                timer.disconnect()


# ───────────────────────────────────────────────────────────────────────────────
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("pyThorlabsAPT – 3-Axis")

    def closeEvent(self, event):
        # ─── Emergency stop all motors on GUI close ─────────────────────────────
        central = self.centralWidget()
        for axis in ("X", "Y", "Z"):
            try:
                iface: interface = getattr(central, f"iface_{axis}")
                iface.stop_any_movement()
            except Exception as e:
                # If something goes wrong, at least log it before exit
                print(f"[ERROR] Could not stop axis {axis}: {e}")
        # ─── Stop all position-poll timers ────────────────────────────────────
        try:
            for timer in central.position_poll_timers.values():
                if timer.isActive():
                    timer.stop()
        except Exception:
            print("[ERROR] Could not stop timers")
        # Proceed with normal close
        super().closeEvent(event)


def main():
    print(f"[DEBUG] {time.strftime('%H:%M:%S')}  →  In main(), before anything else.")
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

    print(f"[DEBUG] {time.strftime('%H:%M:%S')}  →  About to call window.show() and app.exec_().")
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
        print("───────────────────────────────────────────────────────────────")
        print(" ERROR: Unhandled exception in pyThorlabsAPT.main()")
        print("───────────────────────────────────────────────────────────────")
        traceback.print_exc()
        print("───────────────────────────────────────────────────────────────")
        input("Press Enter to exit…")

# Alias for the console entrypoint:
gui = MultiAxisGui

# ───────────────────────────────────────────────────────────────────────────────
