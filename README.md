# Jetson Robocon 2025

This repository contains the software stack for the Jetson Robocon 2025 robotics project. It includes ROS 2 packages, hardware interfaces, simulation tools, and utility scripts for robot control, sensor integration, and system analysis.

---

## Table of Contents

- [Project Structure](#project-structure)
- [Main Components](#main-components)
- [Installation](#installation)
- [Usage](#usage)
- [Development](#development)
- [Dependencies](#dependencies)
- [License](#license)

---

## Project Structure

```
jetson_robocon_2025/
├── .vscode/                  # VSCode settings
├── Full robot calculation/   # Python simulation and analysis tools
├── gamepad_interface/        # ROS2 package for gamepad input
├── main_controller_pkg/      # Main ROS2 controller package (Python)
├── mcu_interface/            # ROS2 package for MCU UART communication
├── odrive_interface/         # ROS2 package for ODrive motor control
├── robot_bringup/            # ROS2 launch files and bringup scripts
├── robot_interfaces/         # ROS2 custom messages and services
├── ros2_astra_camera/        # ROS2 driver for Orbbec Astra camera
└── README.md                 # This file
```

---

## Main Components

- **Full robot calculation**: Python scripts for simulating robot trajectories, optimizing parameters, and visualizing results.
- **gamepad_interface**: ROS2 node for handling gamepad input.
- **main_controller_pkg**: Main Python-based ROS2 controller logic.
- **mcu_interface**: ROS2 node for communicating with the robot's MCU via UART.
- **odrive_interface**: ROS2 node for controlling ODrive motor controllers.
- **robot_bringup**: Launch files for starting up the robot system.
- **robot_interfaces**: Custom ROS2 messages and services for inter-package communication.
- **ros2_astra_camera**: ROS2 driver and messages for Orbbec Astra 3D camera integration.

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/BK-Robotics/jetson_robocon_2025.git
cd jetson_robocon_2025
```

### 2. Install Python dependencies (for simulation tools)

```bash
cd "Full robot calculation"
pip install numpy matplotlib scipy pandas
cd ..
```

### 3. Install ROS 2 and system dependencies

Follow the [official ROS 2 installation guide](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

Install additional dependencies (replace `$ROS_DISTRO` with your ROS 2 distro, e.g., `galactic` or `humble`):

```bash
sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
```

#### Astra Camera

- Install libuvc:

  ```bash
  git clone https://github.com/libuvc/libuvc.git
  cd libuvc
  mkdir build && cd build
  cmake .. && make -j4
  sudo make install
  sudo ldconfig
  ```

- Install udev rules:

  ```bash
  cd ros2_astra_camera/astra_camera/scripts
  sudo bash install.sh
  sudo udevadm control --reload-rules && sudo udevadm trigger
  cd ../../../..
  ```

---

## Usage

### 1. Build ROS 2 packages

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Run the robot simulation

```bash
cd "Full robot calculation"
python3 main.py
```

### 3. Launch the robot system

```bash
ros2 launch robot_bringup <your_launch_file>.launch.py
```

### 4. Start the Astra camera

```bash
ros2 launch astra_camera astra.launch.xml
```

---

## Development

- Use VSCode for development. The `.vscode/` folder contains recommended settings for Python and C++ ROS 2 development.
- Python and C++ include paths are pre-configured for ROS 2 and custom packages.
- Custom messages are defined in [`robot_interfaces`](robot_interfaces/).

---

## Dependencies

- **Python**: 3.10+
- **ROS 2**: Galactic/Humble (see [ros2_astra_camera/README.MD](ros2_astra_camera/README.MD) for details)
- **Libraries**: numpy, matplotlib, scipy, pandas, ROS 2 core packages, ODrive SDK, Orbbec Astra SDK

---

## License

- Main repository: [MIT](https://choosealicense.com/licenses/mit/)
- Astra camera driver: [Apache 2.0](ros2_astra_camera/README.MD#license)

---

## Credits

- BK Robotics Team
- Orbbec Ltd. (Astra camera driver)