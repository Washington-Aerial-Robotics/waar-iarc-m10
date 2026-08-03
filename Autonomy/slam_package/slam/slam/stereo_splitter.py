import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml

class StereoSplitter(Node):
    def __init__(self):
        super().__init__('stereo_splitter')
        
        self.bridge = CvBridge()
        self.declare_parameter('left_calib', '')
        self.declare_parameter('right_calib', '')

        left_calib_path = self.get_parameter('left_calib').value
        right_calib_path = self.get_parameter('right_calib').value

        # Load, scale, and permanently set frame IDs
        self.info_left = self.load_and_scale_camera_info(left_calib_path, 'camera_left_frame')
        self.info_right = self.load_and_scale_camera_info(right_calib_path, 'camera_right_frame')
        
        self.sub_image = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        self.pub_left = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, '/camera/right/image_raw', 10)
        self.pub_left_info = self.create_publisher(CameraInfo, '/camera/left/camera_info', 10)
        self.pub_right_info = self.create_publisher(CameraInfo, '/camera/right/camera_info', 10)

    def load_and_scale_camera_info(self, yaml_file, frame_id):
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        if not yaml_file:
            self.get_logger().warn("Calibration file path is empty!")
            return msg

        try:
            with open(yaml_file, 'r') as f:
                calib_data = yaml.safe_load(f)

            msg.width = calib_data['image_width'] // 2
            msg.height = calib_data['image_height'] // 2
            msg.distortion_model = calib_data['distortion_model']
            msg.d = calib_data['distortion_coefficients']['data']
            
            K = calib_data['camera_matrix']['data']
            msg.k = [K[0]/2, K[1],   K[2]/2, 
                     K[3],   K[4]/2, K[5]/2, 
                     K[6],   K[7],   K[8]]
            
            msg.r = calib_data['rectification_matrix']['data']
            
            P = calib_data['projection_matrix']['data']
            msg.p = [P[0]/2, P[1],   P[2]/2, P[3]/2, 
                     P[4],   P[5]/2, P[6]/2, P[7]/2, 
                     P[8],   P[9],   P[10],  P[11]]
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration from {yaml_file}: {e}")
            
        return msg

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        
        cv_img_small = cv2.resize(cv_img, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
        
        height, width = cv_img_small.shape
        midpoint = width // 2
        
        left_img = cv_img_small[:, :midpoint]
        right_img = cv_img_small[:, midpoint:]
        
        left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding='mono8')
        right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding='mono8')
        
        # Use primitive assignments to prevent reference overwrites and save CPU
        left_msg.header.stamp.sec = msg.header.stamp.sec
        left_msg.header.stamp.nanosec = msg.header.stamp.nanosec
        left_msg.header.frame_id = 'camera_left_frame'
        
        right_msg.header.stamp.sec = msg.header.stamp.sec
        right_msg.header.stamp.nanosec = msg.header.stamp.nanosec
        right_msg.header.frame_id = 'camera_right_frame'
        
        self.info_left.header.stamp.sec = msg.header.stamp.sec
        self.info_left.header.stamp.nanosec = msg.header.stamp.nanosec
        
        self.info_right.header.stamp.sec = msg.header.stamp.sec
        self.info_right.header.stamp.nanosec = msg.header.stamp.nanosec
        
        self.pub_left_info.publish(self.info_left)
        self.pub_right_info.publish(self.info_right)
        
        self.pub_left.publish(left_msg)
        self.pub_right.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StereoSplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
