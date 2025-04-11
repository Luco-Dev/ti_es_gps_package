import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
import math
import time
import serial
import adafruit_gps
import numpy as np

def geodetic_to_ecef(lat, lon, alt):
    a = 6378137.0  # semi-major axis
    e2 = 6.69437999014e-3  # eccentricity squared

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)

    x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (N * (1 - e2) + alt) * math.sin(lat_rad)

    return np.array([x, y, z])

def ecef_to_ra_dec(pos_eq):
    x, y, z = pos_eq
    r = np.linalg.norm(pos_eq)
    ra = math.atan2(y, x)
    dec = math.asin(z / r)

    ra_deg = math.degrees(ra) % 360
    dec_deg = math.degrees(dec)
    return ra_deg, dec_deg, r

def apply_transformation_matrix(pos_ecef, vel_ecef, transformation_matrix):
    state_vector = np.concatenate((pos_ecef, vel_ecef))
    transformed = transformation_matrix @ state_vector
    return transformed[:3], transformed[3:]

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

        self.transformation_matrix = np.identity(6)

    def timer_callback(self):
        self.gps.update()
        if not self.gps.has_fix:
            self.get_logger().info("Waiting for GPS fix...")
            return

        lat = self.gps.latitude
        lon = self.gps.longitude
        alt = self.gps.altitude_m or 0.0

        pos_ecef = geodetic_to_ecef(lat, lon, alt)
        vel_ecef = np.zeros(3)  # Assuming velocity is negligible

        pos_eq, _ = apply_transformation_matrix(pos_ecef, vel_ecef, self.transformation_matrix)
        ra, dec, _ = ecef_to_ra_dec(pos_eq)

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
