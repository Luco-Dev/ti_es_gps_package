# GPS ROS Node for Serial GPS Module

## Introduction

This ROS 2 node provides live geolocation (latitude and longitude) data from a serial GPS module (e.g., a USB GPS dongle). It publishes this data to a ROS topic, enabling integration with robotic systems for tasks such as navigation, localization, or start-point selection.

## Packages

The node relies on the following packages:

- **ROS 2**: Any distribution such as Foxy, Humble, etc. (Jazzy used)
- **Python Libraries**:
  - ```rclpy```
  - ```pyserial```
  - ```adafruit-circuitpython-gps```

Install the Python libraries using:

```
pip install pyserial adafruit-circuitpython-gps
```


## Hardware that interacts with it

- USB GPS modules (NMEA compatible)
- Serial GPS receivers (connected via ```/dev/ttyUSB0```, ```/dev/ttyAMA0```, etc.)

## Installation guide

1. Clone the package into your ROS 2 workspace:

```
cd ~/your_ros_workspace/src
git clone <repository-url>
```

2. Install required Python libraries:

```
pip install pyserial adafruit-circuitpython-gps
```

3. Build your workspace:

```
cd ~/your_ros_workspace
colcon build
```

## Configuration options

- **Serial Port**: Default is ```/dev/ttyUSB0```. Change this in the code if your GPS device appears on a different port.
- **Baud Rate**: Ensure the baud rate matches your GPS device settings (commonly 9600 or 115200).
- **Topic Name**: Default is ```/ti/es/gps_data```. You can remap this in launch files or at runtime.

## Author

Developed by Luco Berkouwer. Feel free to fork this repository for your own use or to contribute improvements. If you make enhancements or fixes, consider submitting a pull request so the community can benefit too.