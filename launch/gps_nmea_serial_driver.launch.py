import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('mvp_gui_2')
    
    # Define the path to the parameter file
    params_file = os.path.join(pkg_share_dir, 'config', 'gps_nmea_serial_driver.yaml')

    # Node for the primary GPS device
    gps_topside_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_topside',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
        remappings=[
            ("/fix", "gps_topside/fix"),
            ("/heading", "gps_topside/heading"),
            ("/vel", "gps_topside/vel"),
            ("/time_reference", "gps_topside/time_reference"),
            ("/gga", "gps_topside/gga"),
        ]
    )

    return LaunchDescription([
        gps_topside_node
    ])