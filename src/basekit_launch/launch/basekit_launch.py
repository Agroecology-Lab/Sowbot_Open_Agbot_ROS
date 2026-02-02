import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Force ROS to find its libraries
    os.environ['LD_LIBRARY_PATH'] = os.environ.get('LD_LIBRARY_PATH', '') + ':/opt/ros/humble/lib'

    return LaunchDescription([
        # 1. ROSBRIDGE (Required for UI to talk to ROS)
        Node(
            package='rosbridge_server', 
            executable='rosbridge_websocket', 
            name='rosbridge_websocket',
            output='screen'
        ),

        # 2. GPS DRIVER (Pure hardware access)
        Node(
            package='ublox_gps', 
            executable='ublox_gps_node', 
            name='ublox_gps_node',
            parameters=[
                os.path.join(get_package_share_directory('ublox_gps'), 'config', 'zed_f9p.yaml'),
                {'device': '/dev/ttyACM0', 'tmode3': 0, 'nav_rate': 1}
            ],
            output='screen'
        ),

        # 3. MAP MANAGER (The simplified way)
        Node(
            package="topological_navigation", 
            executable="map_manager2.py",
            name="map_manager", 
            arguments=['/workspace/maps/test_map.yaml'],
            parameters=[{'no_db': True}],
            output='screen'
        ),

        # 4. WEB UI
        Node(
            package='basekit_ui', 
            executable='basekit_ui_node', 
            name='web_ui',
            output='screen'
        ),

        # 5. MOTOR DRIVER
        Node(
            package='basekit_driver', 
            executable='basekit_driver_node', 
            name='basekit_driver_node',
            parameters=[{'port': 'virtual'}],
            output='screen'
        )
    ])
