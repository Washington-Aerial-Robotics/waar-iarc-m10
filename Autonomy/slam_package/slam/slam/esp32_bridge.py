import rclpy
from rclpy.node import Node
import serial
import math
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # Declare parameters for easy reconfiguration
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        # Open the serial port
        try:
            self.serial_conn = serial.Serial(port, baudrate, timeout=1.0)
            self.get_logger().info(f"Connected to ESP32 on {port} at {baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {port}: {e}")
            return
            
        # Initialize publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.baro_pub = self.create_publisher(PoseWithCovarianceStamped, '/baro/pose', 10)
        
        # Check the serial port at 50Hz
        self.timer = self.create_timer(0.02, self.read_serial_data) 
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to a Quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr # x
        q[1] = sy * cp * sr + cy * sp * cr # y
        q[2] = sy * cp * cr - cy * sp * sr # z
        q[3] = cy * cp * cr + sy * sp * sr # w
        return q

    def read_serial_data(self):
        if self.serial_conn.in_waiting > 0:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                data = line.split(',')
                
                # Verify we received exactly 6 values before parsing
                if len(data) == 6:
                    yaw_deg, roll_deg, pitch_deg, lat_decideg, lon_decideg, alt_m = map(float, data)
                    now = self.get_clock().now().to_msg()
                    
                    # 1. Format IMU Message
                    imu_msg = Imu()
                    imu_msg.header.stamp = now
                    imu_msg.header.frame_id = 'imu_link'
                    
                    # Convert degrees to radians for standard ROS2 compliance
                    roll = math.radians(roll_deg)
                    pitch = math.radians(pitch_deg)
                    yaw = math.radians(yaw_deg)
                    
                    q = self.euler_to_quaternion(roll, pitch, yaw)
                    imu_msg.orientation.x = q[0]
                    imu_msg.orientation.y = q[1]
                    imu_msg.orientation.z = q[2]
                    imu_msg.orientation.w = q[3]
                    
                    # Mark missing data (acceleration/velocity) as invalid (-1.0 at element 0)
                    imu_msg.angular_velocity_covariance[0] = -1.0 
                    imu_msg.linear_acceleration_covariance[0] = -1.0 
                    
                    self.imu_pub.publish(imu_msg)
                    
                    # 2. Format GPS Message
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = now
                    gps_msg.header.frame_id = 'gps_link'
                    
                    # Convert decidegrees to standard decimal degrees
                    gps_msg.latitude = lat_decideg / 10.0
                    gps_msg.longitude = lon_decideg / 10.0
                    gps_msg.altitude = alt_m
                    
                    self.gps_pub.publish(gps_msg)
                    
                    # 3. Format Barometer Pose Message
                    baro_msg = PoseWithCovarianceStamped()
                    baro_msg.header.stamp = now
                    baro_msg.header.frame_id = 'odom'
                    baro_msg.pose.pose.position.z = alt_m
                    
                    self.baro_pub.publish(baro_msg)
                    
            except Exception as e:
                self.get_logger().warn(f"Error parsing serial data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
