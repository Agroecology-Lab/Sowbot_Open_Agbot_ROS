import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    # 1. Pull from Environment (Host -> manage.py -> Docker Env)
    # This is our Single Source of Truth
    mcu_port = os.environ.get('MCU_PORT', '/dev/ttyACM0')
    gps_port = os.environ.get('GPS_PORT', '/dev/ttyACM2')

    # 2. Paths to YAML configs
    driver_config = os.path.join(get_package_share_directory('basekit_driver'), 'config', 'basekit_driver.yaml')
    ublox_config = os.path.join(get_package_share_directory('basekit_launch'), 'config', 'ublox.yaml')

    return LaunchDescription([
        LogInfo(msg=f'--- Launching AgBot with MCU:{mcu_port} GPS:{gps_port} ---'),

        # MCU Driver Node
        Node(
            package='basekit_driver',
            executable='basekit_driver_node',
            name='basekit_driver_node',
            output='screen',
            parameters=[driver_config, {'port': mcu_port}] # Force the port override
        ),

        # GPS Node
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            output='screen',
            # Force the device override
            parameters=[ublox_config, {'device': gps_port}] 
        )
    ])
