import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
import math
import time
import serial
import adafruit_gps

def gps_to_equatorial(lat, lon):
    """
    Convert GPS latitude and longitude to equatorial coordinates.
    Assumes altitude is negligible.
    """
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    
    right_ascension = lon_rad
    declination = lat_rad
    
    return right_ascension, declination

class GPSEquatorialPublisher(Node):
    def __init__(self):
        super().__init__('gps_equatorial_publisher')
        
        self.publisher = self.create_publisher(Float64MultiArray, 'equatorial_coordinates', 10)
        
        # GPS Setup
        self.uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
        self.gps = adafruit_gps.GPS(self.uart, debug=False)
        
        self.gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.gps.send_command(b"PMTK220,1000")
        
        self.get_logger().info("GPS to Equatorial Coordinate Publisher Started")
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.gps.update()
        if not self.gps.has_fix:
            self.get_logger().info("Waiting for fix...")
            return
        
        ra, dec = gps_to_equatorial(self.gps.latitude, self.gps.longitude)
        
        equatorial_msg = Float64MultiArray()
        equatorial_msg.data = [ra, dec]
        
        self.publisher.publish(equatorial_msg)
        self.get_logger().info(f"Published Equatorial Coordinates: RA={ra}, Dec={dec}")


def main(args=None):
    rclpy.init(args=args)
    node = GPSEquatorialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
