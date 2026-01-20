import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # 0. Fetch dynamic ports from environment (mapped by fixusb.py)
    # Defaulting to common paths if variables are missing
    gps_port = os.environ.get('GPS_PORT', '/dev/ttyACM0')
    mcu_port = os.environ.get('MCU_PORT', '/dev/ttyACM1')

    # 1. Paths to configs
    basekit_launch_dir = get_package_share_directory('basekit_launch')
    config_file = os.path.join(basekit_launch_dir, 'config', 'basekit.yaml')

    # 2. Declare Launch Arguments
    declare_use_camera = DeclareLaunchArgument(
        'use_camera', default_value='false',
        description='Whether to launch the Axis IP camera node'
    )

    declare_hw_platform = DeclareLaunchArgument(
        'hw_platform', default_value='laptop',
        description='Hardware platform: orin or laptop'
    )

    # 3. Basekit Driver Node (The ESP32/Lizard controller)
    # Added 'port' parameter to use detected MCU_PORT
    basekit_driver_node = Node(
        package='basekit_driver',
        executable='basekit_driver_node',
        name='controller',
        output='screen',
        parameters=[config_file, {
            'hw_platform': LaunchConfiguration('hw_platform'),
            'port': mcu_port  # Dynamically assigned
        }],
        remappings=[('/cmd_vel', '/cmd_vel_nav')]
    )


    # Explicitly using the zed_f9p.yaml for your hardware
    ublox_config = os.path.join(
        get_package_share_directory('ublox_gps'), 
        'config', 
        'zed_f9p.yaml'
    )

    ublox_gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ublox_gps'), 'launch', 'ublox_gps_node-launch.py')
        ),
        launch_arguments={
            'params_file': ublox_config,
            'device': gps_port,
            'respawn': 'true'
        }.items()
    )
    
    
    
     
    
    

    # 5. Static Transforms
    static_tf_nodes = [
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_base_link_imu',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_imu_gnss',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'imu', 'gnss']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_imu_vsm',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'imu', 'vsm']),
    ]

    # 6. Optional Axis Camera Node
    axis_camera_node = Node(
        package='axis_camera',
        executable='axis_camera_node',
        name='camera',
        condition=IfCondition(LaunchConfiguration('use_camera')),
        parameters=[config_file]
    )

    # 7. UI Node (NiceGUI)
    ui_node = Node(
        package='basekit_ui',
        executable='basekit_ui_node',
        name='web_ui',
        output='screen'
    )

    return LaunchDescription([
        declare_use_camera,
        declare_hw_platform,
        basekit_driver_node,
        ublox_gps_launch,
        axis_camera_node,
        ui_node,
        *static_tf_nodes
    ])
