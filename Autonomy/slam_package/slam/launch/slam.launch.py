import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('slam')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_gps.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'drone.urdf')
    left_calib_file = os.path.join(pkg_share, 'config', 'left.yaml')
    right_calib_file = os.path.join(pkg_share, 'config', 'right.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    video_device = LaunchConfiguration('video_device')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')
    use_legacy_esp32_bridge = LaunchConfiguration('use_legacy_esp32_bridge')
    enable_navsat = LaunchConfiguration('enable_navsat')

    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('image_width', default_value='2560'),
        DeclareLaunchArgument('image_height', default_value='960'),
        DeclareLaunchArgument('framerate', default_value='10.0'),
        DeclareLaunchArgument(
            'use_legacy_esp32_bridge',
            default_value='false',
            description=(
                'Enable the deprecated six-value CSV serial bridge. Keep false '
                'with current binary KAF firmware and drone_hardware_bridge.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_navsat',
            default_value='false',
            description=(
                'Enable navsat_transform and RTAB-Map GPS input only when a '
                'real /gps/fix publisher is present. Current binary autonomy '
                'telemetry provides local metric position, not latitude/longitude.'
            ),
        ),
    ]

    nodes = [
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
                'video_device': video_device,
                'image_width': ParameterValue(image_width, value_type=int),
                'image_height': ParameterValue(image_height, value_type=int),
                'pixel_format': 'mjpeg2rgb',
                'framerate': ParameterValue(framerate, value_type=float),
            }]
        ),
        
        Node(
            package='slam',
            executable='esp32_bridge',
            name='esp32_bridge',
            output='screen',
            condition=IfCondition(use_legacy_esp32_bridge),
            parameters=[{
                'port': serial_port,
                'baudrate': ParameterValue(serial_baudrate, value_type=int),
            }]
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
            condition=IfCondition(enable_navsat),
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
                'subscribe_gps': ParameterValue(enable_navsat, value_type=bool),
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
                'Vis/MaxFeatures': '400',
                # EKF is the sole odom -> base_link authority.
                'publish_tf': False,
            }],
            remappings=[
                ('left/image_rect', '/camera/left/image_rect'),
                ('right/image_rect', '/camera/right/image_rect'),
                ('left/camera_info', '/camera/left/camera_info'),
                ('right/camera_info', '/camera/right/camera_info'),
                ('odom', '/stereo/odom')
            ]
        )
    ]
    return LaunchDescription(arguments + nodes)
