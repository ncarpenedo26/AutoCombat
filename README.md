# AutoCombat
Autonomous combat robotics platform powered by ROS2

## Connecting to RPI
1. **Find network address.** When directly connected to ethernet, this will be 192.168.1.11. Over wifi, this will vary from network to network. Run `ip -a` on the rpi and look for the 'inet' address for the 'wlan0' interface. If you are connected directly over ethernet, you will also have to configure your local machine's DHCP server to assign the rpi an address properly. set IP Address to `192.168.1.10` and subnet mask to `255.255.255.0`
3. a. **SSH** `ssh autocombat@<ip-address>`
2. b. **RDP** connect to the pi using your preferred RDP client.

## Repository Structure

* **`hardware/`**: Contains all **mechanical and hardware design files**.
    * *Examples*: CAD files (`.step`, `.stl`, `.f3d`), bill of materials (`BOM.xlsx`), assembly drawings, and circuit diagrams.
* **`firmware/`**: Contains the **source code for the ESP32 hardware controller**.
    * *Examples*: ESP32 firmware. This code handles low-level motor control and sensor reading.
* **`ros2_ws/`**: The **ROS 2 workspace** for the Raspberry Pi.
    * *Examples*: Package directories (`src/my_package`), launch files, configuration (`.yaml`), and the robot description (`URDF`). This software handles high-level navigation, perception, and communication.
* **`docs/`**: Project documentation, analysis, and reports.
    * *Examples*: Analysis reports, system architecture diagrams, and final project write-up.

## Contribution Guidelines

### 1. **Branching Strategy (Git Flow)**

* **`main` branch**: **Always production/stable-ready**. Only merge well-tested code in from the `develop` branch. **Never commit directly to `main`**.
* **Feature Branches**: Most work must be done on a dedicated branch named clearly. Use lower case with words seperated by dashes. e.g:
    * `holonomic-kinematics`
    * `path-planning`

### 2. **Commit Messages**

Keep commits atomic (each commit should address one logical change) and use clear, descriptive messages.

* **Format**: `Brief, descriptive summary (max 50 chars)`
* **Examples**:
    * `Add path planner node`
    * `Correct PID constants in ESP32 firmware`
    * `Add new motor mount v2 CAD files`

### 3. **Good Practices**

### 4. **Common Commands**
* `git pull`

## Good Practices & Tooling

### Software (`ros2_ws/` and `firmware/`)
TODO
* **Linter/Formatter**: 
