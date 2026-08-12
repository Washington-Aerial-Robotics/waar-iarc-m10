import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('slam')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_gps.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'drone.urdf')
    left_calib_file = os.path.join(pkg_share, 'config', 'left.yaml')
    right_calib_file = os.path.join(pkg_share, 'config', 'right.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Hardware Interface Nodes
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 2560,
                'image_height': 960,
                'pixel_format': 'mjpeg2rgb',
                'framerate': 10.0
            }]
        ),
        
        Node(
            package='slam',
            executable='esp32_bridge',
            name='esp32_bridge',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0', 'baudrate': 115200}]
        ),

        Node(
            package='slam',
            executable='stereo_splitter',
            name='stereo_splitter',
            output='screen',
            parameters=[{
                'left_calib': left_calib_file,
                'right_calib': right_calib_file
            }]
        ),

        # Image Rectification Nodes
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_left',
            namespace='camera/left',
            output='screen',
            parameters=[{'queue_size': 20}],
            remappings=[('image', 'image_raw')]
        ),

        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_right',
            namespace='camera/right',
            output='screen',
            parameters=[{'queue_size': 20}],
            remappings=[('image', 'image_raw')]
        ),

        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),

        # NavSat Transform Node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_config],
            remappings=[
                ('imu/data', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odometry/filtered')
            ]
        ),

        # RTAB-Map Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'subscribe_stereo': True,
                'subscribe_gps': True,
                'approx_sync': False,
                'topic_queue_size': 20, 
                'sync_queue_size': 20,  
                'Vis/MaxFeatures': '400',
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'Grid/FromDepth': 'true',
                'Grid/MaxObstacleHeight': '2.0',
                'Rtabmap/DetectionRate': '2.0'
            }],
            remappings=[
                ('left/image_rect', '/camera/left/image_rect'), 
                ('right/image_rect', '/camera/right/image_rect'),
                ('left/camera_info', '/camera/left/camera_info'),
                ('right/camera_info', '/camera/right/camera_info'),
                ('gps/fix', '/gps/fix'),
                ('odom', '/odometry/filtered'),
                ('grid_map', '/map')
            ]
        ),

        # Stereo Visual Odometry Node
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_stereo': True,
                'approx_sync': False,
                'topic_queue_size': 20, 
                'sync_queue_size': 20,  
                'Vis/MaxFeatures': '400'
            }],
            remappings=[
                ('left/image_rect', '/camera/left/image_rect'),
                ('right/image_rect', '/camera/right/image_rect'),
                ('left/camera_info', '/camera/left/camera_info'),
                ('right/camera_info', '/camera/right/camera_info'),
                ('odom', '/stereo/odom')
            ]
        )
    ])
