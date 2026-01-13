# microbot_wifi_motor_driver

A ROS 2 package for compiling and uploading firmware to an ESP32-S3 microcontroller that acts as a WiFi-enabled motor driver. The microcontroller connects to a ROS 2 network via micro-ROS over WiFi, receiving velocity commands and controlling motors with PID control.

## Installation

### Prerequisites

- ROS 2 (Humble or later)
- PlatformIO Core CLI

### Installing PlatformIO

Choose one of the following installation methods:

**Option 1: PlatformIO installer script**

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install curl python3-pip python3.10-venv python-is-python3 -y
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
```

**Option 2: pip installation**

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python-is-python3
pip3 install --user -U platformio
```

**Make PATH permanent**

After installing PlatformIO with `--user` flag or the installer script, add PlatformIO to your PATH permanently by adding this line to your shell profile:

```bash
# For bash users
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# For zsh users
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
```

_Note: If you installed PlatformIO with the installer script or as another user, adjust the PATH accordingly (e.g., `/home/ros2/.platformio/penv/bin`)._


### Building the Package

```bash
cd ~/ros2_ws/src
git clone git@github.com:nikosmitrop1995/microbot_wifi_motor_driver.git
cd ~/ros2_ws
colcon build --packages-select microbot_wifi_motor_driver
source install/setup.bash
```

## Configuration

Before uploading, configure WiFi credentials and the micro-ROS agent IP address in `src/main.cpp`:

```cpp
char ssid[] = "SSID_NAME";      // Your WiFi network name
char psk[] = "PASSWORD";        // Your WiFi password
std::stringstream ip_address("IP_ADDRESS");  // IP address of your micro-ROS agent
```

## Uploading Code to the Microcontroller

### Prerequisites

- Connect the ESP32-S3 to your computer via USB
- Verify device detection: `pio device list`

### Using ROS 2 Commands

```bash
# Compile
ros2 run microbot_wifi_motor_driver compile

# Upload
ros2 run microbot_wifi_motor_driver upload
```

### Using PlatformIO Directly

```bash
cd $(ros2 pkg prefix microbot_wifi_motor_driver)/share/microbot_wifi_motor_driver
pio run          # Compile
pio run -t upload  # Upload
```

### Troubleshooting

If upload fails, verify:
- USB connection is secure
- USB permissions: add your user to the `dialout` group: `sudo usermod -a -G dialout $USER` (log out and back in)
- Correct board selected in `platformio.ini` (currently `seeed_xiao_esp32s3`)
