import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    missions_pkg_dir = get_package_share_directory('missions_pkg')
    laser_params = os.path.join(missions_pkg_dir, 'params', 'hokuyo_serial.yaml')

    laser = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='hokuyo_serial',
        output='screen',
        prefix="xterm -e",
        parameters=[{'angle_max': 3.14},
            {'angle_min': -3.14},
            {'ip_port': 1853723},
            {'serial_port': '/dev/sensors/hokuyo'},
            {'serial_baud': 115200},
            {'laser_frame_id': 'hokuyo_link'},
            {'calibrate_time': False},
            {'default_user_latency': -0.2},
            {'diagnostics_tolerance': 0.05},
            {'diagnostics_window_time': 5.0},
            {'error_limit': 4},
            {'get_detailed_status': False},
            {'publish_intensity': False},
            {'publish_multiecho': False},
            {'cluster': 1},
            {'skip':0}],)

    return LaunchDescription([
        laser  
    ])
