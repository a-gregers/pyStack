# ───────────────────────────────────────────────────────────────────────────────
# File: …\site-packages\pyThorlabsAPT\main.py
# Overwrite your current main.py with this entire file.
# After saving, restart “Motors” and run: pyThorlabsAPT
# ───────────────────────────────────────────────────────────────────────────────

import os
import sys
import time
import traceback
import argparse
import PyQt5
from PyQt5 import QtWidgets, QtGui, QtCore

# Ensure Qt can find its platform plugins
dirname = os.path.dirname(PyQt5.__file__)
plugin_path = os.path.join(dirname, 'plugins', 'platforms')
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = plugin_path

# Import the real & virtual Thorlabs drivers
import pyThorlabsAPT.driver as driver_real
import pyThorlabsAPT.driver_virtual as driver_virtual

# ───────────────────────────────────────────────────────────────
# Global exception hook so Qt callbacks show full tracebacks:
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



class interface(QtCore.QObject):
    """
    Wraps a single Thorlabs motor. Emits signals for position, motion, etc.
    """

    sig_update_position       = QtCore.pyqtSignal(float)
    sig_step_size             = QtCore.pyqtSignal(float)
    sig_change_moving_status  = QtCore.pyqtSignal(int)
    sig_change_homing_status  = QtCore.pyqtSignal(int)
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
            'step_size': 0.001,  # 0.001 mm (1 µm)
        }
        self.connected_device_name = ''
        self.output = {'Position': 0.0}
        self._units = {'mm': 1, 'deg': 2}

        # Pick driver
        if self.use_virtual:
            self.instrument = driver_virtual.pyThorlabsAPT()
        else:
            self.instrument = driver_real.pyThorlabsAPT()

    def list_serials(self):
        """
        Return a Python list of all connected‐device serial strings.
        """
        try:
            raw = self.instrument.list_devices()
        except Exception:
            return []
        # raw is a list of (serial, name); serial is raw[0]
        return [dev[0] for dev in raw]

    def connect_device_by_serial(self, serial_number: str):
        """
        Connect to a single motor by its serial number.  Emit sig_connected.
        """
        try:
            msg, code = self.instrument.connect_device(serial_number)
            if code == 1:
                self.connected_device_name = serial_number
                self.read_position()
                self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
                self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
                self.sig_connected.emit(True)
            else:
                self.connected_device_name = ''
                self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
                self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
                self.sig_connected.emit(False)
        except Exception:
            self.connected_device_name = ''
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
            self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
            self.sig_connected.emit(False)

    def disconnect_device(self):
        """
        Disconnect if currently connected.
        """
        try:
            _msg, _code = self.instrument.disconnect_device()
        except:
            pass
        self.connected_device_name = ''
        self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
        self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
        self.sig_connected.emit(False)

    def is_device_moving(self):
        """
        Return True if the motor reports it is in motion.
        """
        try:
            return bool(getattr(self.instrument, 'is_in_motion', False))
        except Exception:
            return False

    def stop_any_movement(self):
        """
        If moving, stop and emit movement‐ended.
        """
        if self.is_device_moving():
            try:
                self.instrument.stop_profiled()
            except:
                pass
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)

    def home(self):
        """
        Send home command. Poll and emit position continuously while homing.
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
        While home is in progress, read position every 50 ms.
        """
        try:
            self.read_position()
            if self.instrument.is_in_motion:
                QtCore.QTimer.singleShot(50, self._poll_home)
            else:
                # Final update
                self.read_position()
                self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
                self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
        except Exception:
            self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)

    def move_single_step(self, direction):
        """
        direction = +1 or −1, move by step_size (mm). Emits start/end signals.
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
        Poll until is_in_motion is False, then emit updated position.
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
        User entered step size (in mm). Convert & store.
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
        User entered absolute position (in mm). Move there.
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
        """
        Poll until move ends, then emit updated position.
        """
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
        Read current position (in mm) and emit sig_update_position.
        """
        try:
            pos = float(self.instrument.position)
        except:
            pos = 0.0
        self.output['Position'] = pos
        self.sig_update_position.emit(pos)
        return pos

    def close(self):
        """
        Called when app quits; disconnect if needed.
        """
        try:
            self.instrument.disconnect_device()
        except:
            pass



# ───────────────────────────────────────────────────────────────────────────────
class MultiAxisGui(QtWidgets.QWidget):
    def __init__(self, iface_x: interface, iface_y: interface, iface_z: interface, parent=None):
        super().__init__(parent)

        self.iface_x = iface_x
        self.iface_y = iface_y
        self.iface_z = iface_z

        # Display‐unit: "mm" or "µm"
        self.display_unit = "µm"

        # Internal stores for velocity & accel (always in mm/s and mm/s²)
        self.velocity_mm_s = {'X': 1.0, 'Y': 1.0, 'Z': 1.0}
        self.accel_mm_s2   = {'X': 1.0, 'Y': 1.0, 'Z': 1.0}

        # Timers for continuous jog/drive, one per axis
        self.continuous_timers = {
            'X': QtCore.QTimer(self),
            'Y': QtCore.QTimer(self),
            'Z': QtCore.QTimer(self)
        }
        for t in self.continuous_timers.values():
            t.setInterval(100)  # 100 ms tick

        # Poll positions & homing every 100 ms (so user sees live position during homing)
        self.position_timer = QtCore.QTimer(self)
        self.position_timer.setInterval(100)
        self.position_timer.timeout.connect(self._poll_all_positions)
        self.position_timer.start()

        # Top‐level layout
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # === Row 0: Display‐Units selector ===
        h_units = QtWidgets.QHBoxLayout()
        lbl_units = QtWidgets.QLabel("Display Units:")
        self.combo_display_units = QtWidgets.QComboBox()
        self.combo_display_units.addItems(["mm", "µm"])
        self.combo_display_units.setCurrentText("µm")
        self.combo_display_units.currentTextChanged.connect(self._on_units_changed)

        h_units.addWidget(lbl_units)
        h_units.addWidget(self.combo_display_units)
        h_units.addStretch(1)
        layout.addLayout(h_units)

        # Build axis groups (hard‐wired serials)
        self.groupX = self._build_axis_group("X", self.iface_x, allow_drive=False, serial="26006151")
        layout.addWidget(self.groupX)

        self.groupY = self._build_axis_group("Y", self.iface_y, allow_drive=False, serial="26006139")
        layout.addWidget(self.groupY)

        self.groupZ = self._build_axis_group("Z", self.iface_z, allow_drive=True, serial="26002801")
        layout.addWidget(self.groupZ)

        # Tooltip for keyboard shortcuts
        tips = (
            "Keyboard Shortcuts:\n"
            "  A / D  → Move X left / right\n"
            "  W / S  → Move Y forward / back\n"
            "  ↑ / ↓  → Move Z up / down\n"
            "  In Single‐Click Jog: only one move per press.\n"
            "  In Continuous Jog: hold key to move every 100 ms.\n"
            "  In Drive (Z only): hold key to move at Velocity until release.\n"
            "  H / J / K → Home X / Home Y / Home Z\n"
            "  Space → Stop ALL motors\n"
        )
        self.setToolTip(tips)

        # Ensure this widget can receive key events
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setFocus()

    def _build_axis_group(self, axis_label: str, iface: interface, allow_drive: bool, serial: str) -> QtWidgets.QGroupBox:
        """
        Build controls for one axis. `allow_drive` only for Z. `serial` is hard‐wired.
        """
        grp = QtWidgets.QGroupBox(f"{axis_label}-Axis Controls")
        vlay = QtWidgets.QVBoxLayout(grp)
        vlay.setSpacing(4)
        vlay.setContentsMargins(4, 4, 4, 4)

        # --- Row 1: Serial, Refresh, Connect/Disconnect ---
        hlay1 = QtWidgets.QHBoxLayout()
        label_dev = QtWidgets.QLabel(f"{axis_label} Serial:")
        edit_serial = QtWidgets.QLineEdit(serial)
        edit_serial.setEnabled(False)
        btn_refresh = QtWidgets.QPushButton("Refresh")
        btn_connect = QtWidgets.QPushButton(f"Connect {axis_label}")

        setattr(self, f"edit_Serial{axis_label}", edit_serial)
        setattr(self, f"button_Refresh{axis_label}", btn_refresh)
        setattr(self, f"button_Connect{axis_label}", btn_connect)
        setattr(self, f"iface_{axis_label}", iface)

        for w in [label_dev, edit_serial, btn_refresh, btn_connect]:
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

        # --- Row 3: Jog sub-modes + Velocity + Accel ---
        hlay3 = QtWidgets.QHBoxLayout()
        radio_cont = QtWidgets.QRadioButton("Continuous")
        radio_single = QtWidgets.QRadioButton("Single-Click")
        radio_single.setChecked(True)
        label_vel = QtWidgets.QLabel("Velocity (µm/s):")
        edit_vel = QtWidgets.QLineEdit("1000.0")
        label_acc = QtWidgets.QLabel("Accel (µm/s²):")
        edit_acc = QtWidgets.QLineEdit("1000.0")

        setattr(self, f"radio_Cont{axis_label}", radio_cont)
        setattr(self, f"radio_Single{axis_label}", radio_single)
        setattr(self, f"label_Velocity{axis_label}", label_vel)
        setattr(self, f"edit_Velocity{axis_label}", edit_vel)
        setattr(self, f"label_Accel{axis_label}", label_acc)
        setattr(self, f"edit_Accel{axis_label}", edit_acc)

        for w in [radio_cont, radio_single, label_vel, edit_vel, label_acc, edit_acc]:
            hlay3.addWidget(w)
        hlay3.addStretch(1)

        widget_row3 = QtWidgets.QWidget()
        widget_row3.setLayout(hlay3)
        setattr(self, f"widget_JogOptions{axis_label}", widget_row3)
        vlay.addWidget(widget_row3)

        if not allow_drive:
            combo_mode.setCurrentText("Jog")
            widget_row3.show()

        # --- Row 4: Position display, Move < /> , Step size, Home, Stop ---
        hlay4 = QtWidgets.QHBoxLayout()
        label_pos = QtWidgets.QLabel(f"{axis_label} Position:")
        edit_pos = QtWidgets.QLineEdit("0.0")
        edit_pos.setAlignment(QtCore.Qt.AlignRight)
        btn_move_neg = QtWidgets.QPushButton("<")
        btn_move_pos = QtWidgets.QPushButton(">")
        label_by = QtWidgets.QLabel("By (µm):")
        edit_step = QtWidgets.QLineEdit(str(iface.settings['step_size']*1000.0))
        btn_home = QtWidgets.QPushButton(f"Home {axis_label}")
        btn_stop = QtWidgets.QPushButton(f"Stop {axis_label}")

        setattr(self, f"edit_Position{axis_label}", edit_pos)
        setattr(self, f"button_MoveNeg{axis_label}", btn_move_neg)
        setattr(self, f"button_MovePos{axis_label}", btn_move_pos)
        setattr(self, f"label_By{axis_label}", label_by)
        setattr(self, f"edit_StepSize{axis_label}", edit_step)
        setattr(self, f"button_Home{axis_label}", btn_home)
        setattr(self, f"button_Stop{axis_label}", btn_stop)

        for w in [label_pos, edit_pos, btn_move_neg, btn_move_pos, label_by, edit_step, btn_home, btn_stop]:
            hlay4.addWidget(w)
        hlay4.addStretch(1)
        vlay.addLayout(hlay4)

        # --- Wire up signals & slots ---
        btn_refresh.clicked.connect(lambda _, ax=axis_label: self._refresh_device_availability(ax))
        btn_connect.clicked.connect(lambda _, ax=axis_label: self._connect_clicked(ax))
        combo_mode.currentTextChanged.connect(lambda text, ax=axis_label: self._on_mode_changed(ax, text))

        btn_move_neg.pressed.connect(lambda ax=axis_label: self._arrow_pressed(ax, -1))
        btn_move_pos.pressed.connect(lambda ax=axis_label: self._arrow_pressed(ax, +1))
        btn_move_neg.released.connect(lambda ax=axis_label: self._arrow_released(ax))
        btn_move_pos.released.connect(lambda ax=axis_label: self._arrow_released(ax))

        edit_pos.returnPressed.connect(lambda ax=axis_label: self._press_enter_position(ax))
        edit_step.returnPressed.connect(lambda ax=axis_label: self._press_enter_step(ax))
        edit_vel.returnPressed.connect(lambda ax=axis_label: self._press_enter_velocity(ax))
        edit_acc.returnPressed.connect(lambda ax=axis_label: self._press_enter_accel(ax))

        btn_home.clicked.connect(lambda _, ax=axis_label: self._home_clicked(ax))
        btn_stop.clicked.connect(lambda _, ax=axis_label: self._stop_clicked(ax))

        iface.sig_update_position.connect(lambda val, ax=axis_label: self._on_position_change(ax, val))
        iface.sig_step_size.connect(lambda val, ax=axis_label: self._on_step_size_change(ax, val))
        iface.sig_change_moving_status.connect(lambda st, ax=axis_label: self._on_moving_state_change(ax, st))
        iface.sig_change_homing_status.connect(lambda st, ax=axis_label: self._on_homing_state_change(ax, st))
        iface.sig_connected.connect(lambda st, ax=axis_label: self._on_connection_status_change(ax, st))

        # Initialize Connect button as disabled until Refresh is clicked:
        btn_connect.setEnabled(False)

        # Initially show/hide Jog options based on default Mode
        if combo_mode.currentText() == "Drive":
            widget_row3.hide()
        else:
            widget_row3.show()

        return grp


    # ───────────────────────────────────────────────────────────────────────────
    # Device availability / connect:

    def _refresh_device_availability(self, axis: str):
        """
        Called when the user clicks “Refresh” for <axis>.
        We call iface.list_serials() to see if the assigned serial is present.
        If present, enable Connect; otherwise disable it.
        """
        iface: interface = getattr(self, f"iface_{axis}")
        serial = getattr(self, f"edit_Serial{axis}").text()
        # List all attached device serials
        serials = iface.list_serials()
        btn_connect: QtWidgets.QPushButton = getattr(self, f"button_Connect{axis}")
        if serial in serials:
            btn_connect.setEnabled(True)
        else:
            btn_connect.setEnabled(False)
            # If currently connected to something else, disconnect
            if iface.connected_device_name == serial:
                iface.disconnect_device()

    def _connect_clicked(self, axis: str):
        """
        Connect or disconnect <axis> by its hard-wired serial.
        """
        iface: interface = getattr(self, f"iface_{axis}")
        serial = getattr(self, f"edit_Serial{axis}").text()
        btn_connect: QtWidgets.QPushButton = getattr(self, f"button_Connect{axis}")
        if iface.connected_device_name == "":
            iface.connect_device_by_serial(serial)
        else:
            iface.disconnect_device()

    def _on_connection_status_change(self, axis: str, connected: bool):
        """
        Change Connect button text when connection status changes.
        """
        btn_connect: QtWidgets.QPushButton = getattr(self, f"button_Connect{axis}")
        if connected:
            btn_connect.setText(f"Disconnect {axis}")
        else:
            btn_connect.setText(f"Connect {axis}")

    # ───────────────────────────────────────────────────────────────────────────
    # Position & Step‐Size handlers (with unit conversion):

    def _on_position_change(self, axis: str, val_mm: float):
        """
        Update the position field (in display units).
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_Position{axis}")
        factor = 1.0 if self.display_unit == "mm" else 1000.0
        val_disp = val_mm * factor
        # Format: 4 decimals in mm, 1 decimal in µm
        if self.display_unit == "mm":
            edit.setText(f"{val_disp:.4f}")
        else:
            edit.setText(f"{val_disp:.1f}")

    def _on_step_size_change(self, axis: str, val_mm: float):
        """
        Update the “By” field (in display units).
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_StepSize{axis}")
        factor = 1.0 if self.display_unit == "mm" else 1000.0
        val_disp = val_mm * factor
        if self.display_unit == "mm":
            edit.setText(f"{val_disp:.4f}")
        else:
            edit.setText(f"{val_disp:.1f}")

    # ───────────────────────────────────────────────────────────────────────────
    # Jog / Drive / Arrow handling:

    def _on_mode_changed(self, axis: str, new_mode: str):
        """
        Show/hide Jog options row when Mode changes. Stop any running timer.
        """
        widget_row3: QtWidgets.QWidget = getattr(self, f"widget_JogOptions{axis}")
        timer = self.continuous_timers[axis]

        if new_mode == "Drive":
            widget_row3.hide()
        else:
            widget_row3.show()

        # Stop any active timer on mode switch
        if timer.isActive():
            timer.stop()
            timer.disconnect()

    def _home_clicked(self, axis: str):
        iface: interface = getattr(self, f"iface_{axis}")
        if iface.connected_device_name != "":
            iface.home()

    def _stop_clicked(self, axis: str):
        iface: interface = getattr(self, f"iface_{axis}")
        iface.stop_any_movement()
        timer = self.continuous_timers[axis]
        if timer.isActive():
            timer.stop()
            timer.disconnect()

    def _arrow_pressed(self, axis: str, direction: int):
        """
        Called when < or > is pressed on <axis>. Behavior depends on Mode:
        - Jog/Single-Click: one step (ignored if moving)
        - Jog/Continuous: start QTimer calling move_single_step every 100 ms
        - Drive (Z only): start QTimer calling move_by(velocity × 0.1) every 100 ms
        """
        iface: interface = getattr(self, f"iface_{axis}")
        if iface.connected_device_name == "":
            return

        combo_mode: QtWidgets.QComboBox = getattr(self, f"combo_Mode{axis}")
        mode = combo_mode.currentText()
        timer = self.continuous_timers[axis]

        # --- Jog Mode ---
        if mode == "Jog":
            radio_single: QtWidgets.QRadioButton = getattr(self, f"radio_Single{axis}")
            radio_cont: QtWidgets.QRadioButton   = getattr(self, f"radio_Cont{axis}")

            # Single-Click: one step if not already moving
            if radio_single.isChecked():
                if not iface.is_device_moving():
                    iface.move_single_step(direction)
                return

            # Continuous: start timer if not already active
            if radio_cont.isChecked():
                if not timer.isActive():
                    timer.timeout.connect(lambda d=direction, ax=axis: self._jog_timer_tick(ax, d))
                    timer.start()
                return

        # --- Drive Mode (Z only) ---
        if mode == "Drive" and axis == "Z":
            try:
                edit_vel: QtWidgets.QLineEdit = getattr(self, f"edit_Velocity{axis}")
                val_disp = float(edit_vel.text())
            except:
                val_disp = 1000.0  # default
            factor = 1.0 if self.display_unit == "mm" else 1000.0
            vel_mm_s = val_disp / factor  # convert display (mm/s or µm/s) back to mm/s

            # step per 100 ms = velocity × 0.1 s
            step_mm = vel_mm_s * 0.1
            if not timer.isActive():
                def drive_tick(d=direction, ax=axis, step=step_mm):
                    iface2: interface = getattr(self, f"iface_{ax}")
                    if iface2.connected_device_name != "":
                        try:
                            iface2.instrument.move_by(step * d)
                        except:
                            pass
                        QtCore.QTimer.singleShot(50, iface2.read_position)
                timer.timeout.connect(drive_tick)
                timer.start()
            return

    def _arrow_released(self, axis: str):
        """
        Called when < or > is released. Stop that axis’s QTimer.
        """
        timer = self.continuous_timers[axis]
        if timer.isActive():
            timer.stop()
            timer.disconnect()

    def _jog_timer_tick(self, axis: str, direction: int):
        """
        Every 100 ms, if not moving, issue a single step (Jog‐Continuous).
        """
        iface: interface = getattr(self, f"iface_{axis}")
        if iface.connected_device_name != "" and not iface.is_device_moving():
            iface.move_single_step(direction)

    # ───────────────────────────────────────────────────────────────────────────
    # Keyboard shortcuts

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        key = event.key()

        # X: A / D
        if key == QtCore.Qt.Key_A:
            self._arrow_pressed("X", -1)
            return
        if key == QtCore.Qt.Key_D:
            self._arrow_pressed("X", +1)
            return

        # Y: W / S
        if key == QtCore.Qt.Key_W:
            self._arrow_pressed("Y", +1)
            return
        if key == QtCore.Qt.Key_S:
            self._arrow_pressed("Y", -1)
            return

        # Z: Up / Down
        if key == QtCore.Qt.Key_Up:
            self._arrow_pressed("Z", +1)
            return
        if key == QtCore.Qt.Key_Down:
            self._arrow_pressed("Z", -1)
            return

        # Home: H / J / K
        if key == QtCore.Qt.Key_H:
            self.iface_x.home()
            return
        if key == QtCore.Qt.Key_J:
            self.iface_y.home()
            return
        if key == QtCore.Qt.Key_K:
            self.iface_z.home()
            return

        # Space: Stop all
        if key == QtCore.Qt.Key_Space:
            for ax in ("X", "Y", "Z"):
                iface: interface = getattr(self, f"iface_{ax}")
                iface.stop_any_movement()
                timer = self.continuous_timers[ax]
                if timer.isActive():
                    timer.stop()
                    timer.disconnect()
            return

        super().keyPressEvent(event)

    # ───────────────────────────────────────────────────────────────────────────
    # Poll positions every 100 ms so homing and moves show live updates

    def _poll_all_positions(self):
        for ax in ("X", "Y", "Z"):
            iface: interface = getattr(self, f"iface_{ax}")
            if iface.connected_device_name != "":
                iface.read_position()

    # ───────────────────────────────────────────────────────────────────────────
    # Text‐field handlers with unit conversion:

    def _press_enter_position(self, axis: str):
        """
        User hit Enter in the Position field (display units). Convert to mm & move.
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_Position{axis}")
        iface: interface = getattr(self, f"iface_{axis}")
        if iface.connected_device_name == "":
            return
        try:
            txt = edit.text()
            val_disp = float(txt)
            factor = 1.0 if self.display_unit == "mm" else 1000.0
            pos_mm = val_disp / factor
            iface.set_position(str(pos_mm))
        except ValueError:
            pass

    def _press_enter_step(self, axis: str):
        """
        User hit Enter in “By” field (display units). Convert to mm & set step_size.
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_StepSize{axis}")
        iface: interface = getattr(self, f"iface_{axis}")
        try:
            txt = edit.text()
            val_disp = float(txt)
            factor = 1.0 if self.display_unit == "mm" else 1000.0
            step_mm = val_disp / factor
            iface.set_step_size(str(step_mm))
        except ValueError:
            pass

    def _press_enter_velocity(self, axis: str):
        """
        User hit Enter in Velocity field (display units). Convert to internal mm/s.
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_Velocity{axis}")
        try:
            txt = edit.text()
            val_disp = float(txt)
            factor = 1.0 if self.display_unit == "mm" else 1000.0
            self.velocity_mm_s[axis] = val_disp / factor
        except ValueError:
            pass

    def _press_enter_accel(self, axis: str):
        """
        User hit Enter in Accel field (display units). Convert to mm/s².
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_Accel{axis}")
        try:
            txt = edit.text()
            val_disp = float(txt)
            factor = 1.0 if self.display_unit == "mm" else 1000.0
            self.accel_mm_s2[axis] = val_disp / factor
        except ValueError:
            pass

    # ───────────────────────────────────────────────────────────────────────────
    def _on_units_changed(self, new_unit: str):
        """
        Called when user selects “mm” or “µm”.
        Update all labels and values for Position, Step size, Velocity, Accel.
        """
        old_unit = self.display_unit
        self.display_unit = new_unit
        old_factor = 1.0 if old_unit == "mm" else 1000.0
        new_factor = 1.0 if new_unit == "mm" else 1000.0

        for axis in ("X", "Y", "Z"):
            # --- Update Position ---
            iface: interface = getattr(self, f"iface_{axis}")
            current_mm = iface.output.get('Position', 0.0)
            val_disp = current_mm * new_factor
            edit_pos: QtWidgets.QLineEdit = getattr(self, f"edit_Position{axis}")
            if new_unit == "mm":
                edit_pos.setText(f"{val_disp:.4f}")
            else:
                edit_pos.setText(f"{val_disp:.1f}")

            # --- Update Step size (By) ---
            current_step_mm = iface.settings.get('step_size', 0.001)
            step_disp = current_step_mm * new_factor
            edit_step: QtWidgets.QLineEdit = getattr(self, f"edit_StepSize{axis}")
            if new_unit == "mm":
                edit_step.setText(f"{step_disp:.4f}")
                getattr(self, f"label_By{axis}").setText("By (mm):")
            else:
                edit_step.setText(f"{step_disp:.1f}")
                getattr(self, f"label_By{axis}").setText("By (µm):")

            # --- Update Velocity field ---
            vel_mm_s = self.velocity_mm_s[axis]
            vel_disp = vel_mm_s * new_factor
            edit_vel: QtWidgets.QLineEdit = getattr(self, f"edit_Velocity{axis}")
            if new_unit == "mm":
                edit_vel.setText(f"{vel_disp:.4f}")
                getattr(self, f"label_Velocity{axis}").setText("Velocity (mm/s):")
            else:
                edit_vel.setText(f"{vel_disp:.1f}")
                getattr(self, f"label_Velocity{axis}").setText("Velocity (µm/s):")

            # --- Update Accel field ---
            acc_mm_s2 = self.accel_mm_s2[axis]
            acc_disp = acc_mm_s2 * new_factor
            edit_acc: QtWidgets.QLineEdit = getattr(self, f"edit_Accel{axis}")
            if new_unit == "mm":
                edit_acc.setText(f"{acc_disp:.4f}")
                getattr(self, f"label_Accel{axis}").setText("Accel (mm/s²):")
            else:
                edit_acc.setText(f"{acc_disp:.1f}")
                getattr(self, f"label_Accel{axis}").setText("Accel (µm/s²):")


# ───────────────────────────────────────────────────────────────────────────────
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("pyThorlabsAPT – 3-Axis")

    def closeEvent(self, event):
        super().closeEvent(event)


def main():
    print(f"[DEBUG] {time.strftime('%H:%M:%S')}  →  In main(), before anything else.")
    parser = argparse.ArgumentParser(description="pyThorlabsAPT – 3-Axis GUI")
    parser.add_argument("-s", "--decrease_verbose", action="store_true", help="Decrease verbosity")
    parser.add_argument("-virtual", action="store_true", help="Use virtual driver (no hardware).")
    args = parser.parse_args()
    use_virtual = args.virtual

    app = QtWidgets.QApplication(sys.argv)

    # Instantiate three interfaces, one per axis:
    iface_x = interface(use_virtual=use_virtual)
    iface_y = interface(use_virtual=use_virtual)
    iface_z = interface(use_virtual=use_virtual)

    window = MainWindow()
    gui3 = MultiAxisGui(iface_x, iface_y, iface_z, parent=window)
    window.setCentralWidget(gui3)
    window.resize(750, 540)

    print(f"[DEBUG] {time.strftime('%H:%M:%S')}  →  About to call window.show() and app.exec_().")
    window.show()

    # Disconnect when quitting:
    app.aboutToQuit.connect(iface_x.disconnect_device)
    app.aboutToQuit.connect(iface_y.disconnect_device)
    app.aboutToQuit.connect(iface_z.disconnect_device)

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

# Alias for console entry point:
gui = MultiAxisGui

# ───────────────────────────────────────────────────────────────────────────────
