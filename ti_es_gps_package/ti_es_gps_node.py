import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import adafruit_gps
import json

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher = self.create_publisher(String, '/ti/es/gps_data', 10)
        self.uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
        self.gps = adafruit_gps.GPS(self.uart, debug=False)
        self.gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.gps.send_command(b"PMTK220,1000")
        self.timer = self.create_timer(1.0, self.publish_gps)

    def publish_gps(self):
        self.gps.update()
        if not self.gps.has_fix:
            self.get_logger().info("GPS: waiting for fix...")
            return
        lat = self.gps.latitude
        lon = self.gps.longitude
        if lat is not None and lon is not None:
            payload = {"type": "gps", "lat": lat, "lon": lon}
            msg = String()
            msg.data = json.dumps(payload)
            self.publisher.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()