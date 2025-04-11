import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
import math
import time
import serial
import adafruit_gps
import numpy as np
from datetime import datetime

# Constants for WGS-84
a = 6378137.0            # semi-major axis
e2 = 6.69437999014e-3    # eccentricity squared

def geodetic_to_ecef(lat, lon, alt):
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)

    x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (N * (1 - e2) + alt) * math.sin(lat_rad)

    return np.array([x, y, z])

def greenwich_sidereal_time():
    now = datetime.utcnow()
    year = now.year
    month = now.month
    day = now.day + (now.hour + now.minute / 60 + now.second / 3600) / 24

    if month <= 2:
        year -= 1
        month += 12

    A = int(year / 100)
    B = 2 - A + int(A / 4)

    JD = int(365.25 * (year + 4716)) + int(30.6001 * (month + 1)) + day + B - 1524.5
    T = (JD - 2451545.0) / 36525.0
    GST = 280.46061837 + 360.98564736629 * (JD - 2451545.0) + T*T*(0.000387933 - T / 38710000.0)
    GST = GST % 360.0

    return math.radians(GST)  # in radians

def ecef_to_eci(pos_ecef):
    gst = greenwich_sidereal_time()
    x, y, z = pos_ecef
    x_eci = math.cos(gst) * x + math.sin(gst) * y
    y_eci = -math.sin(gst) * x + math.cos(gst) * y
    z_eci = z
    return np.array([x_eci, y_eci, z_eci])

def eci_to_ra_dec(pos_eci):
    x, y, z = pos_eci
    r = np.linalg.norm(pos_eci)
    ra = math.atan2(y, x)
    dec = math.asin(z / r)

    ra_deg = math.degrees(ra) % 360
    dec_deg = math.degrees(dec)
    return ra_deg, dec_deg, r

class GPSEquatorialPublisher(Node):
    def __init__(self):
        super().__init__('gps_equatorial_publisher')

        self.publisher = self.create_publisher(Float64MultiArray, 'equatorial_coordinates', 10)

        self.uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
        self.gps = adafruit_gps.GPS(self.uart, debug=False)

        self.gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.gps.send_command(b"PMTK220,1000")

        self.get_logger().info("GPS to Equatorial Coordinate Publisher Started")
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.gps.update()
        if not self.gps.has_fix:
            self.get_logger().info("Waiting for GPS fix...")
            return

        lat = self.gps.latitude
        lon = self.gps.longitude
        alt = self.gps.altitude_m or 0.0

        pos_ecef = geodetic_to_ecef(lat, lon, alt)
        pos_eci = ecef_to_eci(pos_ecef)
        ra, dec, _ = eci_to_ra_dec(pos_eci)

        equatorial_msg = Float64MultiArray()
        equatorial_msg.data = [ra, dec]

        self.publisher.publish(equatorial_msg)
        self.get_logger().info(f"Published Equatorial Coordinates: RA={ra:.4f}, Dec={dec:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSEquatorialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
