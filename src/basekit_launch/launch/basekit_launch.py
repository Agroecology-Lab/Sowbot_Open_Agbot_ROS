import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Paths for configuration
    nav2_params = os.path.join(get_package_share_directory('basekit_launch'), 'config', 'nav2_params.yaml')
    map_path = '/workspace/maps/test_map.yaml'
    
    return LaunchDescription([
        # 1. INFRASTRUCTURE: Rosbridge & Static TF (The "Ghost" Foundation)
        Node(
            package='rosbridge_server', 
            executable='rosbridge_websocket', 
            name='rosbridge_websocket',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),

        # 2. TOPOLOGICAL STACK (Dev Branch V2 naming)
        Node(
            package="topological_navigation", 
            executable="map_manager2.py",
            name="map_manager", 
            arguments=[map_path],
            parameters=[{'no_db': True}],
            output='screen'
        ),
        Node(
            package="topological_navigation",
            executable="localisation2.py",
            name="topological_localisation",
            output='screen'
        ),
        Node(
            package="topological_navigation",
            executable="navigation2.py",
            name="navigation_executor",
            output='screen'
        ),

        # 3. NAV2 BACK-END (Core Servers)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params],
            output='screen'
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_params],
            output='screen'
        ),
        # Behavior Server provides 'spin', 'backup', and 'wait' for Humble
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_params],
            output='screen'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True, 
                'node_names': [
                    'planner_server', 
                    'controller_server', 
                    'behavior_server', 
                    'bt_navigator'
                ]
            }],
            output='screen'
        ),
        
        # 4. HARDWARE DRIVERS & UI
        Node(
            package='ublox_gps', 
            executable='ublox_gps_node', 
            name='ublox_gps_node',
            parameters=[{'device': 'virtual', 'nav_rate': 1}],
            output='screen'
        ),
        Node(
            package='basekit_driver', 
            executable='basekit_driver_node', 
            name='basekit_driver_node',
            parameters=[{'port': 'virtual'}],
            output='screen'
        ),
        Node(
            package='basekit_ui', 
            executable='basekit_ui_node', 
            name='web_ui',
            output='screen'
        )
    ])
