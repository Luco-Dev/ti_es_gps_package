# GPS ROS Node for Serial GPS Module

## Overview

This ROS 2 node provides live geolocation (latitude and longitude) from a serial GPS device (such as a USB GPS dongle) to a ROS-based system. It publishes current GPS coordinates received from the GPS module for use by other nodes (e.g., robotic navigation, start position selection, etc).

## Setup

### Dependencies

- **ROS 2** (e.g., Foxy, Humble, etc.)
- **Python Libraries**:
  - `rclpy`
  - `pyserial`
  - `adafruit-circuitpython-gps`  
    > On Ubuntu, install via:  
    ```
    pip install pyserial adafruit-circuitpython-gps
    ```

### Installation

1. Clone the package into your ROS workspace:
   ```
   cd ~/your_ros_workspace/src
   git clone <repository-url>
   ```
2. Install required Python libraries (if needed):
   ```
   pip install pyserial adafruit-circuitpython-gps
   ```
3. Build the workspace:
   ```
   cd ~/your_ros_workspace
   colcon build
   ```

## Node Details

- **Node Name**: `gps_publisher`
- **Publisher**: `/ti/es/gps_data` (`std_msgs/String`)  
  Publishes a JSON string containing live GPS coordinates of the device.

## Communication

- **GPS Device**: Connects to an NMEA GPS receiver (typ. `/dev/ttyUSB0`).
- **Message format**: Publishes a string in the following form:
  ```
  {"type": "gps", "lat": 51.87654, "lon": 4.62345}
  ```

## Functions

- Reads NMEA sentences from the serial GPS.
- Parses latitude and longitude from GPS fix (uses Adafruit_CircuitPython_GPS).
- Publishes the most recent GPS position as a ROS message.

## Usage

1. Connect your GPS receiver (e.g., USB GPS dongle) and ensure it appears as `/dev/ttyUSB0` or similar.
2. Run the ROS node:
   ```
   ros2 run <package_name> gps_publisher
   ```
3. Subscribe to `/ti/es/gps_data` to receive live location updates:
   ```
   ros2 topic echo /ti/es/gps_data
   ```
4. Integrate into your robotic application as required.

## Notes

- The node will only publish data after a valid GPS fix is acquired.
- Edit the serial port name in the code if your device appears elsewhere (e.g., `/dev/ttyAMA0`).
