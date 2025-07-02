# pyStack GUI

A custom Python-based GUI for controlling Thorlabs APT motors used in a van der Waals stacker. This application is intended to serve as a substitute for the Thorlabs Kinesis software, enabling greater fine-tuning and systematic stacking.

**If you would like features to be added/adjusted or you noticed bugs/crashes, please reach out to me with details. Importantly, if the GUI crashes or the motors are out of control, please first open the Kinesis software and connect to ensure the hardware is in a good state before attempting a relaunch.**
> This project builds upon code from [qpit/thorlabs_apt](https://github.com/qpit/thorlabs_apt) and [MicheleCotrufo/pyThorlabsAPT](https://github.com/MicheleCotrufo/pyThorlabsAPT) with extensive GUI and control enhancements. All low-level credit goes to [Tobias Gehring](https://github.com/qpit), and the GUI structure was inspired by [Michele Cotrufo](https://github.com/MicheleCotrufo).

---

## Features

### Multi-Axis Motor Control
- The GUI supports 3 motors to connect to the following axes: **X**, **Y**, and **Z**. Each has a live position readout with continuous polling for updates. The GUI is designed to accept a combination of keyboard and mouse input with various move modes and editable settings, and the distance unit used throughout is µm.

- To launch, single-click the pyStack icon in the Windows Taskbar or double-click on the Desktop icon

- **Note: This GUI and the Kinesis software attempt to use devices at the same time, which can cause issues in detecting and connecting devices for both programs. When using pyStack, ensure that the Kinesis software is fully closed**

### Keybinds, Controls, and Indicators
The motors together define a coordinate system ***FROM THE PERSPECTIVE OF THE CAMERA*** as such: X = left/right movement, Y = forward/back movement, Z = vertical up/down movement. **Importantly, this means that to touch down a transfer slide onto the substrate, the Z motor must be moved up with the ↑ key**

- **Connection:** Before the motors are connected, the GUI will initialize with empty space for the indicators and controls to appear upon connection. The status before connection should display "Not connected." In order to connect motors, please click "Manage connections" and select from the list of detected devices. After this step, the buttons should appear for the selected motors. When stationary, a motor should display a status of "Idle."

- **Movement:** The mappings are A/D for +/- movement in the X direction, W/S for +/- in Y, and ↑/↓ for +/- in Z. There are also arrow buttons that can be pressed to perform the same moves with +/- signs indicating the directions as before. During movement, the motors should indicate a status of "Moving". ***Note**: Due to the physical setup of the motors, maintaining a right-handed (X,Y) coordinate system necessitates that rightward movement in the X direction decreases the position value and vice-versa. The mechanics of the keyboard controls otherwise behave as expected*

- **Homing:** H/J/K will home the X/Y/Z motors, respectively. There is also a button that can be clicked to home each motor individually. While homing, the motors should indicate a status of "Homing" and the "Homing" checkbox should be lit up during the process. The "rev" and "fwd" checkboxes will light up to indicate if the motor has reached its reverse or forward limit switches, which are used to bound movement and carry out homing. Note that it is normal for the "rev" checkbox to light up during homing 

- **Stop:** To stop all motors, press Space on the keyboard, and individual motors can be stopped by clicking their respective "Stop" buttons. For velocities greater than 100 µm/s, the motors will conduct a profiled stop and may significantly overshoot your intended target if the acceleration is set too low. Thus, if operating above 100 µm/s velocity, aim for an acceleration/velocity ratio of 1 or greater to decrease the overshoot of the profiled stop. For low velocities less than 100 µm/s, the motors will stop immediately. **Avoid stopping motors while they are actively homing, as this can lead them to abnormal behavior or unresponsiveness**

- **Reset:** If motors start behaving strangely or become unresponsive, press the "Reset motor" button to reset that specific motor or either Shift key on the keyboard to reset all motors simultaneously

- **Position:** While the motors move, their positions will update in the "Position" display line

- **Text Boxes:** To commit changes to a text box, you can either click away to finish editing or press Enter on the keyboard. Either method will commit the changes you made in that text box

### Motion Modes
#### 1. Go-To Position
- The "Position" display line can also be edited by clicking on it with the mouse. Changing the value in this text box will set a new target position for that motor to go to and cause the motor to move to that new position. Neither the velocity nor the acceleration can be customized for this type of movement
  
#### 2. Jog: Continuous Mode
- Generally used for most adjustments
- Motors move **while a key or button is held** and stop automatically on key/button release. As before, for velocities greater than 100 µm/s, releasing a move key will immediately issue a profiled stop, and lower velocities will issue an immediate stop
- Velocity and acceleration are configurable per axis in their respective text boxes
- You can click the "Slow Preset," "Medium Preset," or "Fast Preset" buttons to automatically set your velocity and acceleration values to ones commonly used for that motor

#### 3. Jog: Single-Click Mode
- Typically used for slow, consistent movements
- Upon selecting the Jog: Single-Click mode, a new text box will appear with "By: " followed by the editable step size. Instead of continuous movement while a key is pressed, one keypress/button click triggers a **fixed-distance step** in which the motor will operate at its provided velocity and acceleration and stop after traveling its step size
- Step size, velocity, and acceleration are configurable as they were in the continuous mode
- As in the Jog: Continuous mode, you can click the "Slow Preset," "Medium Preset," or "Fast Preset" buttons to use common move parameters

#### 4. Drive Mode (Z-axis only)
- Typically used for **high-speed movements**
- In the Drive Mode, the acceleration is preset to 1000 µm/s² and cannot be adjusted. Upon changing the mode to "Drive", the velocity and acceleration text boxes will disappear
- There are four velocity profiles, and each can be altered by clicking the "Edit profiles" button. The preset values are the same used in the Thorlabs Kinesis software:
  - Profile 1: 375 µm/s
  - Profile 2: 750 µm/s
  - Profile 3: 1125 µm/s
  - Profile 4: 1500 µm/s
- Movements are the same as in the Jog: Continuous mode — the motors will move continuously while a key is held, and a stop will be issued once they are let go
---

## Preset Support

- The GUI also supports loading a `.json` file with a customizable target position and motion parameters for **X** and **Y**. This preset target position will appear in vector form, (X target, Y target), in the bottom right of the GUI
- The GUI automatically loads the target.json file stored in C:Users\devar\Documents. Pressing "Load preset" in the bottom left of the GUI will allow you to select a different .json file to load, if you so desire, though it is easiest to edit target.json with your new desired parameters
- Pressing "Go preset" will issue a simultaneous Go-To Position movement for the X and Y motors, which will stop at their target positions

### Example `target.json`

```json
{
  "x": 100.0,
  "y": 200.0,
  "velocity": 750.0,
  "acceleration": 500.0
}
