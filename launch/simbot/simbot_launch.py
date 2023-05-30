import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get the pkg directory
    pkg_dir = get_package_share_directory('missions_pkg')
    coppelia_ros2_pkg = get_package_share_directory('coppelia_ros2_pkg')

    # variables/files
    use_sim_time = True
    remappings = []
    params_yaml_file = os.path.join(pkg_dir, 'launch', 'simbot', 'simbot_params.yaml')
    nav2_launch_file = os.path.join(pkg_dir, 'launch', 'simbot', 'nav2_launch.py')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'coppelia_sim.rviz')
    urdf = os.path.join(pkg_dir, 'launch', 'rhodon', 'rhodon_urdf.xml')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    logger = LaunchConfiguration("log_level")
    
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        # Declare Arguments 
        # ==================
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),
        DeclareLaunchArgument(
            "pkg_dir",
            default_value = pkg_dir,
            description = "this pkg dir",
            ),
        DeclareLaunchArgument(
            "coppelia_ros2_pkg",
            default_value = coppelia_ros2_pkg,
            description = "Coppelia_pkg path",
            ),

        # RVIZ2
        #============
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[],
            arguments = ['-d' + rviz_file]
            ),

        # HW_DRIVERS
        #============
        # Coppelia Simulator
        Node(
            package='coppelia_ros2_pkg',
            executable='coppelia_simulator',
            name='coppelia_simulator',
            output='screen',
            prefix="xterm -hold -e",
            parameters = [ParameterFile(params_yaml_file, allow_substs=True)]
            ),
                        
        # Keyboard Control
        Node(
            package='keyboard_control',
            executable='keyboard_control_plus',
            name='keyboard_control',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file]
            ),

        # MANAGEMENT
        #===========
        # Task Manager
        Node(
            package='task_manager',
            executable='bt_manager',
            name='task_manager',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file]
            ),
        # Task From Topic
        Node(
            package='task_manager',
            executable='task_from_topic',
            name='task_from_topic',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file]
            ),
        
        # UTILS
        #=========
        # NAV2 Autonomous Navigation (see nav2_launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file)
        ),

        # MQTT_bridge
        Node(
            package='mqtt_bridge',
            executable='mqtt_bridge_node',
            name='mqtt_bridge',
            output='screen',
            prefix='xterm -hold -e',
            parameters=[params_yaml_file]            
            ),

        # HRI
        Node(
            package='web_hri',
            executable='web_hri',
            name='web_hri',
            output='screen',
            prefix='xterm -hold -e',
            parameters=[params_yaml_file]
            ),

        # PATROL
        Node(
            package='patrol',
            executable='patrol_times',
            name='patrol_times',
            output='screen',
            parameters=[params_yaml_file]
            ),
        
        # Battery_Manager
        Node(
            package='battery_manager',
            executable='battery_manager',
            name='battery_manager',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file]            
            ),
        
        # Status Publisher
        Node(
            package='robot_status_publisher',
            executable='robot_status_publisher_node',
            name='status_publisher',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file]
            )
    ]) #end LaunchDescription
