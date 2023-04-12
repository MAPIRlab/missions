import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    missions_pkg_dir = get_package_share_directory('missions_pkg')
    urdf = os.path.join(missions_pkg_dir, 'params', 'rhodon_urdf.xml')
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # URDF model (state publisher)
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf]
    )
        
    return LaunchDescription([
        start_robot_state_publisher_cmd        
    ])
