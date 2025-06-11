# ───────────────────────────────────────────────────────────────────────────────
# File: …\site‐packages\pyThorlabsAPT\main.py
# Replace your entire main.py with the code below.
# After saving, restart “Motors” and run: pyThorlabsAPT
# ───────────────────────────────────────────────────────────────────────────────

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
# Global exception hook so Qt callbacks show tracebacks
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
    A wrapper around a single Thorlabs device. Emits signals when things change.
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
            'step_size': 0.001,  # default 0.001 mm (1 µm) in mm units
        }
        self.connected_device_name = ''
        self.output = {'Position': 0.0}
        self._units = {'mm': 1, 'deg': 2}

        # Pick driver:
        if self.use_virtual:
            self.instrument = driver_virtual.pyThorlabsAPT()
        else:
            self.instrument = driver_real.pyThorlabsAPT()

    def connect_device_by_serial(self, serial_number: str):
        """
        Directly connect to a motor by its serial number (no dropdown).
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
        try:
            return bool(getattr(self.instrument, 'is_in_motion', False))
        except Exception:
            return False

    def stop_any_movement(self):
        if self.is_device_moving():
            try:
                self.instrument.stop_profiled()
            except:
                pass
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)

    def home(self):
        """
        Send the home command.  Poll until it finishes, but also update position continuously.
        """
        try:
            if self.is_device_moving():
                return
            self.sig_change_homing_status.emit(self.SIG_HOMING_STARTED)
            self.instrument.move_home()
            # Start polling
            self._poll_home()
        except Exception:
            # If the home command fails, signal it ended
            self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)

    def _poll_home(self):
        """
        Poll while homing: update position continuously, then once done emit signals.
        """
        try:
            # Always read current (possibly changing) position:
            self.read_position()
            if self.instrument.is_in_motion:
                QtCore.QTimer.singleShot(50, self._poll_home)
            else:
                # Final update:
                self.read_position()
                self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
                self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)
        except Exception:
            self.sig_change_homing_status.emit(self.SIG_HOMING_ENDED)
            self.sig_change_moving_status.emit(self.SIG_MOVEMENT_ENDED)

    def move_single_step(self, direction):
        """
        direction = +1 or -1; uses self.settings['step_size'] (in mm) for Single-Click Jog.
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
        Poll until is_in_motion is False, then emit read_position().
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

    def close(self):
        """
        Called when the app is quitting.
        """
        try:
            self.instrument.disconnect_device()
        except:
            pass
        # No additional signal needed here


# ───────────────────────────────────────────────────────────────────────────────
class MultiAxisGui(QtWidgets.QWidget):
    def __init__(self, iface_x: interface, iface_y: interface, iface_z: interface, parent=None):
        super().__init__(parent)

        self.iface_x = iface_x
        self.iface_y = iface_y
        self.iface_z = iface_z

        # Keep track of display‐unit multiplier:
        #   mm => factor=1;   µm => factor=1000
        self.display_unit = "µm"

        # Create timers for continuous movement (one per axis)
        self.continuous_timers = {
            'X': QtCore.QTimer(self),
            'Y': QtCore.QTimer(self),
            'Z': QtCore.QTimer(self)
        }
        for t in self.continuous_timers.values():
            t.setInterval(100)  # 100 ms tick

        # Poll position & homing continuously every 100 ms for each axis if connected
        self.position_timer = QtCore.QTimer(self)
        self.position_timer.setInterval(100)
        self.position_timer.timeout.connect(self._poll_all_positions)
        self.position_timer.start()

        # Top‐level layout: first row = unit selector, then 3 axis groups
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

        # Build X‐axis group (Jog only, no Drive)
        self.groupX = self._build_axis_group("X", self.iface_x, allow_drive=False, serial="26006151")
        layout.addWidget(self.groupX)

        # Build Y‐axis group (Jog only, no Drive)
        self.groupY = self._build_axis_group("Y", self.iface_y, allow_drive=False, serial="26006139")
        layout.addWidget(self.groupY)

        # Build Z‐axis group (Jog + Drive)
        self.groupZ = self._build_axis_group("Z", self.iface_z, allow_drive=True, serial="26002801")
        layout.addWidget(self.groupZ)

        # Tooltip for keyboard shortcuts
        tips = (
            "Keyboard Shortcuts:\n"
            "  A / D  → X  ;  W / S  → Y  ;  ↑ / ↓ → Z\n"
            "  In Single‐Click Jog: one move per key‐press (ignored if already moving)\n"
            "  In Continuous Jog: hold key to move at step every tick (approx.\n"
            "      speed = step ÷ (timer interval)).\n"
            "  In Drive (Z only): hold key to move at Velocity (mm/s) until release.\n"
            "  H / J / K → Home X / Home Y / Home Z\n"
            "  Space → Stop ALL motors\n"
        )
        self.setToolTip(tips)

        # Ensure this widget gets focus so keyPressEvent works
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setFocus()

    def _build_axis_group(self, axis_label: str, iface: interface, allow_drive: bool, serial: str) -> QtWidgets.QGroupBox:
        """
        Build a QGroupBox containing all controls for one axis (Label e.g. "X").
        If `allow_drive` is True (only for Z), show Drive mode; otherwise only Jog.
        `serial` is the hard‐coded serial number string for this axis.
        """
        grp = QtWidgets.QGroupBox(f"{axis_label}-Axis Controls")
        vlay = QtWidgets.QVBoxLayout(grp)
        vlay.setSpacing(4)
        vlay.setContentsMargins(4, 4, 4, 4)

        # --- Row 1: Serial, Refresh, Connect/Disconnect ---
        hlay1 = QtWidgets.QHBoxLayout()
        label_dev = QtWidgets.QLabel(f"{axis_label} Device:")
        combo_dev = QtWidgets.QComboBox()
        combo_dev.addItem("")            # initially empty; “Refresh” will fill it
        btn_refresh   = QtWidgets.QPushButton(f"Refresh {axis_label}")
        btn_connect   = QtWidgets.QPushButton(f"Connect {axis_label}")
    
        # Save references so callbacks can see them:
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

        # --- Row 3: Jog sub‐mode (Continuous / Single) + Velocity + Accel ---
        hlay3 = QtWidgets.QHBoxLayout()
        radio_cont = QtWidgets.QRadioButton("Continuous")
        radio_single = QtWidgets.QRadioButton("Single‐Click")
        radio_single.setChecked(True)
        label_vel    = QtWidgets.QLabel("")  # we'll set text dynamically
        edit_vel = QtWidgets.QLineEdit("1.0")
        label_acc    = QtWidgets.QLabel("")  # set text dynamically
        edit_acc = QtWidgets.QLineEdit("1.0")

        setattr(self, f"radio_Cont{axis_label}", radio_cont)
        setattr(self, f"radio_Single{axis_label}", radio_single)
        setattr(self, f"label_Vel{axis_label}", label_vel)
        setattr(self, f"edit_Velocity{axis_label}", edit_vel)
        setattr(self, f"label_Acc{axis_label}", label_acc)
        setattr(self, f"edit_Accel{axis_label}", edit_acc)

        for w in [radio_cont, radio_single, label_vel, edit_vel, label_acc, edit_acc]:
            hlay3.addWidget(w)
        hlay3.addStretch(1)

        # Container for row 3 so we can hide/show it easily
        widget_row3 = QtWidgets.QWidget()
        widget_row3.setLayout(hlay3)
        
        setattr(self, f"widget_JogOptions{axis_label}", widget_row3)
        vlay.addWidget(widget_row3)

        # For axes without Drive, force Mode to “Jog”
        if not allow_drive:
            combo_mode.setCurrentText("Jog")
            widget_row3.show()

        # Initialize Velocity/Accel labels according to current display_unit
        # e.g. “Velocity (µm/s):” or “Velocity (mm/s):”
        vel_label: QtWidgets.QLabel = getattr(self, f"label_Vel{axis_label}")
        acc_label: QtWidgets.QLabel = getattr(self, f"label_Acc{axis_label}")
        unit = self.display_unit
        vel_label.setText(f"Velocity ({unit}/s):")
        acc_label.setText(f"Accel ({unit}/s²):")


        # --- Row 4: Position display, Move < / >, Step size, Home, Stop ---
        hlay4 = QtWidgets.QHBoxLayout()
        label_pos = QtWidgets.QLabel(f"{axis_label} Position:")
        edit_pos = QtWidgets.QLineEdit("0.000")
        edit_pos.setAlignment(QtCore.Qt.AlignRight)
        btn_move_neg = QtWidgets.QPushButton("<")
        btn_move_pos = QtWidgets.QPushButton(">")
        label_by = QtWidgets.QLabel("")
        edit_step = QtWidgets.QLineEdit(str( (iface.settings['step_size']*1000) ))
        btn_home = QtWidgets.QPushButton(f"Home {axis_label}")
        btn_stop = QtWidgets.QPushButton(f"Stop {axis_label}")

        setattr(self, f"edit_Position{axis_label}", edit_pos)
        setattr(self, f"button_MoveNeg{axis_label}", btn_move_neg)
        setattr(self, f"button_MovePos{axis_label}", btn_move_pos)
        setattr(self, f"label_By{axis_label}", label_by)
        setattr(self, f"edit_StepSize{axis_label}", edit_step)
        setattr(self, f"button_Home{axis_label}", btn_home)
        setattr(self, f"button_Stop{axis_label}", btn_stop)
        
        by_label: QtWidgets.QLabel = getattr(self, f"label_By{axis_label}")
        unit = self.display_unit
        by_label.setText(f"By ({unit}):")

        for w in [label_pos, edit_pos, btn_move_neg, btn_move_pos, label_by, edit_step, btn_home, btn_stop]:
            hlay4.addWidget(w)
        hlay4.addStretch(1)
        vlay.addLayout(hlay4)

        # --- Wire up signals & slots ---
        combo_mode.currentTextChanged.connect(lambda text, ax=axis_label: self._on_mode_changed(ax, text))

        btn_connect.clicked.connect(lambda _, ax=axis_label: self._connect_clicked(ax))

        # When “Refresh <Axis>” is clicked: scan devices, enable Connect only if our serial is present
        btn_refresh.clicked.connect(lambda _, ax=axis_label: self._refresh_clicked(ax))
        
        btn_move_neg.pressed.connect(lambda ax=axis_label: self._arrow_pressed(ax, -1))
        btn_move_pos.pressed.connect(lambda ax=axis_label: self._arrow_pressed(ax, +1))
        btn_move_neg.released.connect(lambda ax=axis_label: self._arrow_released(ax))
        btn_move_pos.released.connect(lambda ax=axis_label: self._arrow_released(ax))
        edit_pos.returnPressed.connect(lambda ax=axis_label: self._press_enter_position(ax))
        edit_step.returnPressed.connect(lambda ax=axis_label: self._press_enter_step(ax))
        btn_home.clicked.connect(lambda _, ax=axis_label: self._home_clicked(ax))
        btn_stop.clicked.connect(lambda _, ax=axis_label: self._stop_clicked(ax))

        iface.sig_update_position.connect(lambda val, ax=axis_label: self._on_position_change(ax, val))
        iface.sig_step_size.connect(lambda val, ax=axis_label: self._on_step_size_change(ax, val))
        iface.sig_change_moving_status.connect(lambda st, ax=axis_label: self._on_moving_state_change(ax, st))
        iface.sig_change_homing_status.connect(lambda st, ax=axis_label: self._on_homing_state_change(ax, st))
        iface.sig_connected.connect(lambda st, ax=axis_label: self._on_connection_status_change(ax, st))

        # Initially show/hide Jog options based on default Mode:
        if combo_mode.currentText() == "Drive":
            widget_row3.hide()
        else:
            widget_row3.show()

        return grp


    # ───────────────────────────────────────────────────────────────────────────
    # Handlers for signals and buttons:

    def _on_position_change(self, axis: str, val_mm: float):
        """
        Update the position field for <axis>.  val_mm is in mm.
        Convert to display_unit (mm or µm).
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_Position{axis}")
        factor = 1.0 if self.display_unit == "mm" else 1000.0
        val_disp = val_mm * factor
        # Show 4 decimals in mm, 1 decimal in µm
        if self.display_unit == "mm":
            edit.setText(f"{val_disp:.4f}")
        else:
            edit.setText(f"{val_disp:.1f}")

    def _on_step_size_change(self, axis: str, val_mm: float):
        """
        When the interface’s step size changes (in mm), update displayed step
        (in mm or µm).
        """
        edit: QtWidgets.QLineEdit = getattr(self, f"edit_StepSize{axis}")
        factor = 1.0 if self.display_unit == "mm" else 1000.0
        val_disp = val_mm * factor
        if self.display_unit == "mm":
            edit.setText(f"{val_disp:.4f}")
        else:
            edit.setText(f"{val_disp:.1f}")

    def _on_moving_state_change(self, axis: str, status: int):
        """
        Called when a move or jog is starting or ending. We use it to block
        additional single‐clicks until motion ends.
        """
        # If a move just ended, no special action needed here.
        pass

    def _on_homing_state_change(self, axis: str, status: int):
        """
        We poll homing in interface, so position updates already occur.
        """
        pass

    def _on_connection_status_change(self, axis: str, connected: bool):
        """
        Whenever the interface signals a connection change, toggle the button label.
        """
        btn_connect: QtWidgets.QPushButton = getattr(self, f"button_Connect{axis}")
        if connected:
            btn_connect.setText(f"Disconnect {axis}")
        else:
            btn_connect.setText(f"Connect {axis}")



    def _on_mode_changed(self, axis: str, new_mode: str):
        """
        Show/hide Jog sub‐mode row depending on Mode.
        Stop any active continuous timer if switching modes.
        """
        widget_row3: QtWidgets.QWidget = getattr(self, f"widget_JogOptions{axis}")
        timer = self.continuous_timers[axis]
        if new_mode == "Drive":
            widget_row3.hide()
        else:
            widget_row3.show()

        # Stop any running timer when switching mode
        if timer.isActive():
            timer.stop()
            timer.disconnect()

    def _refresh_ui_unit_fields(self, axis: str):
        """
        Re‐emit current position & step so UI fields reformat in new units.
        """
        iface: interface = getattr(self, f"iface_{axis}")
        current_mm = iface.output.get('Position', 0.0)
        self._on_position_change(axis, current_mm)
        current_step_mm = iface.settings.get('step_size', 0.001)
        self._on_step_size_change(axis, current_step_mm)

    def _refresh_all_units(self):
        """
        Called when Display Units changes. Refresh X, Y, Z fields.
        """
        for axis in ("X", "Y", "Z"):
            self._refresh_ui_unit_fields(axis)

    def _refresh_clicked(self, axis: str):
        """
        Called when the user clicks “Refresh <Axis>”.
        Query the driver for all attached devices, then populate the drop‐down.
        """
        iface: interface = getattr(self, f"iface_{axis}")
        combo: QtWidgets.QComboBox = getattr(self, f"combo_Device{axis}")

        # 1) Ask the driver to list all attached devices (serial, name):
        try:
            raw_list = iface.instrument.list_devices()
        except Exception:
            raw_list = []

        # 2) Convert into display strings like "26006151 → MotorName":
        display_list = []
        for dev in raw_list:   # dev = (serial, description)
            # Convert bytes/ints to str if necessary:
            serial_str = str(dev[0])
            name_str = str(dev[1]) if len(dev) > 1 else ""
            display_list.append(f"{serial_str} → {name_str}")

        # 3) Populate the combo box:
        combo.clear()
        if display_list:
            combo.addItems(display_list)
        else:
            combo.addItem("<no devices>")

        # 4) Ensure Connect is disabled until a real device is selected
        btn_connect: QtWidgets.QPushButton = getattr(self, f"button_Connect{axis}")
        btn_connect.setEnabled(False)

        # 5) Connect to a new selection in the dropdown (optional):
        #    If you want the Connect button to become enabled as soon as
        #    someone picks a valid device, you can connect:
        combo.currentIndexChanged.connect(
            lambda idx, ax=axis: btn_connect.setEnabled(idx >= 0 and combo.currentText() != "<no devices>")
        )



    def _connect_clicked(self, axis: str):
        """
        Connect/Disconnect for <axis> using the dropdown’s selected text.
        """
        iface: interface = getattr(self, f"iface_{axis}")
        combo: QtWidgets.QComboBox = getattr(self, f"combo_Device{axis}")
        text = combo.currentText()

        # If no device or placeholder, do nothing:
        if not text or text == "<no devices>":
            return

        if iface.connected_device_name == "":
            # Connect by passing the “serial → name” string exactly
            iface.connect_device(text)
        else:
            iface.disconnect_device()



    def _press_enter_position(self, axis: str):
        """
        User hit Enter in Position field (in display units). Convert to mm and call set_position.
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
        User hit Enter in StepSize field (in display units). Convert to mm and call set_step_size.
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

    # ───────────────────────────────────────────────────────────────────────────
    def _arrow_pressed(self, axis: str, direction: int):
        """
        Called when < or > is pressed. Behavior depends on Mode:
        - Jog / Single‐Click: one move (ignored if already moving)
        - Jog / Continuous: start QTimer that calls step every tick
        - Drive (Z only): start QTimer that calls move_by(velocity*dt) every tick
        """
        iface: interface = getattr(self, f"iface_{axis}")
        if iface.connected_device_name == "":
            return

        combo_mode: QtWidgets.QComboBox = getattr(self, f"combo_Mode{axis}")
        mode = combo_mode.currentText()

        # --- Jog Mode ---
        if mode == "Jog":
            radio_single: QtWidgets.QRadioButton = getattr(self, f"radio_Single{axis}")
            radio_cont: QtWidgets.QRadioButton   = getattr(self, f"radio_Cont{axis}")
            timer = self.continuous_timers[axis]

            # SINGLE‐CLICK: one step only if not already moving
            if radio_single.isChecked():
                if not iface.is_device_moving():
                    iface.move_single_step(direction)
                return

            # CONTINUOUS: start timer that calls move_single_step every 100 ms
            if radio_cont.isChecked():
                if not timer.isActive():
                    timer.timeout.connect(lambda d=direction, ax=axis: self._jog_timer_tick(ax, d))
                    timer.start()
                return

        # --- Drive Mode (Z only) ---
        if mode == "Drive" and axis == "Z":
            try:
                edit_vel: QtWidgets.QLineEdit = getattr(self, f"edit_Velocity{axis}")
                vel_mm_s = float(edit_vel.text())
            except:
                vel_mm_s = 1.0

            # step per 100 ms = velocity × 0.1 s
            step_mm = vel_mm_s * 0.1
            timer = self.continuous_timers[axis]
            if not timer.isActive():
                def drive_tick(d=direction, ax=axis, step=step_mm):
                    iface2: interface = getattr(self, f"iface_{ax}")
                    if iface2.connected_device_name != "":
                        # Directly move_by a small amount each tick
                        try:
                            iface2.instrument.move_by(step * d)
                        except:
                            pass
                        # Poll to update position
                        QtCore.QTimer.singleShot(50, iface2.read_position)

                timer.timeout.connect(drive_tick)
                timer.start()
            return

    def _arrow_released(self, axis: str):
        """
        Called when < or > is released. Stop any QTimer for that axis.
        """
        timer = self.continuous_timers[axis]
        if timer.isActive():
            timer.stop()
            timer.disconnect()

    def _jog_timer_tick(self, axis: str, direction: int):
        """
        Called every 100 ms by the Jog QTimer.  Just calls move_single_step(direction)
        if not already moving.
        """
        iface: interface = getattr(self, f"iface_{axis}")
        if iface.connected_device_name != "" and not iface.is_device_moving():
            iface.move_single_step(direction)

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        """
        Handle keyboard shortcuts: A/D → X, W/S → Y, Up/Down → Z, H/J/K → home.
        Holding a key in Continuous modes acts like mouse‐press. Releasing acts like mouse‐release.
        """
        key = event.key()

        # ─── X‐axis: A / D ─────────────────────────────────────────────────────
        if key == QtCore.Qt.Key_A:
            self._arrow_pressed("X", -1)
            return
        if key == QtCore.Qt.Key_D:
            self._arrow_pressed("X", +1)
            return

        # ─── Y‐axis: W / S ─────────────────────────────────────────────────────
        if key == QtCore.Qt.Key_W:
            self._arrow_pressed("Y", +1)
            return
        if key == QtCore.Qt.Key_S:
            self._arrow_pressed("Y", -1)
            return

        # ─── Z‐axis: Up / Down arrows ───────────────────────────────────────────
        if key == QtCore.Qt.Key_Up:
            self._arrow_pressed("Z", +1)
            return
        if key == QtCore.Qt.Key_Down:
            self._arrow_pressed("Z", -1)
            return

        # ─── Home keys: H / J / K ──────────────────────────────────────────────
        if key == QtCore.Qt.Key_H:
            self.iface_x.home()
            return
        if key == QtCore.Qt.Key_J:
            self.iface_y.home()
            return
        if key == QtCore.Qt.Key_K:
            self.iface_z.home()
            return

        # ─── Spacebar → Stop all ──────────────────────────────────────────────
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

    def _poll_all_positions(self):
        """
        Every 100 ms, if an axis is connected, call read_position() so that
        position updates continuously—this covers both jog/drive and homing.
        """
        for ax in ("X", "Y", "Z"):
            iface: interface = getattr(self, f"iface_{ax}")
            if iface.connected_device_name != "":
                iface.read_position()

    def _on_units_changed(self, new_unit: str):
        """
        Called when “mm” or “µm” is selected. Update display_unit and refresh all fields.
        """
        self.display_unit = new_unit
        self._refresh_all_units()
        
        # Update Velocity/Accel labels for all three axes
        for axis in ("X", "Y", "Z"):
            vel_label: QtWidgets.QLabel = getattr(self, f"label_Vel{axis}")
            acc_label: QtWidgets.QLabel = getattr(self, f"label_Acc{axis}")
            by_label: QtWidgets.QLabel = getattr(self, f"label_By{axis}")
            unit = self.display_unit
            vel_label.setText(f"Velocity ({unit}/s):")
            acc_label.setText(f"Accel ({unit}/s²):")
            by_label.setText(f"By ({unit}):")

# ───────────────────────────────────────────────────────────────────────────────
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("pyThorlabsAPT – 3-Axis")

    def closeEvent(self, event):
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
    window.resize(700, 520)

    print(f"[DEBUG] {time.strftime('%H:%M:%S')}  →  About to call window.show() and app.exec_().")
    window.show()

    # Ensure interfaces close when the app is quitting:
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

# Alias for the console entrypoint:
gui = MultiAxisGui

# ───────────────────────────────────────────────────────────────────────────────
