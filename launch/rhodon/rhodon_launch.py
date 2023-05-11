import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    missions_pkg_dir = get_package_share_directory('missions_pkg')

    # variables/files
    use_sim_time = False
    remappings=[]
    params_yaml_file = os.path.join(missions_pkg_dir, 'launch', 'rhodon', 'rhodon_params.yaml')
    nav2_launch_file = os.path.join(missions_pkg_dir, 'launch', 'rhodon', 'nav2_launch.py')
    rviz_file = os.path.join(missions_pkg_dir, 'rviz', 'rhodon.rviz')
    urdf = os.path.join(missions_pkg_dir, 'launch', 'rhodon', 'rhodon_urdf.xml')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    logger = LaunchConfiguration("log_level")
    
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),

        # HW_DRIVERS
        #===========    
        # ARIA
        Node(
            package='ros2aria',
            executable='ros2aria',
            name='ros2aria',
            output='screen',
            prefix='xterm -e',
            parameters=[params_yaml_file]
            ),

        # Hokuyo Front SICK LMS-200 || Hokuyo URG-04LX
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='hokuyo_front',
            output='screen',
            prefix="xterm -e",
            parameters=[params_yaml_file]
            ),        
        
        # Hokuyo Back UTM-30LX
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='hokuyo_back',
            output='screen',
            prefix="xterm -e",
            parameters=[params_yaml_file]
            ),

        # URDF model (TFs)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]
            ),
                
        # Keyboard Control
        Node(
            package='keyboard_control',
            executable='keyboard_control_plus',
            name='keyboard_control',
            output='screen',
            prefix="xterm -e",
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
            prefix="xterm -e",
            parameters=[params_yaml_file]
            ),
        # Task From Topic
        Node(
            package='task_manager',
            executable='task_from_topic',
            name='task_from_topic',
            output='screen',
            prefix="xterm -e",
            parameters=[params_yaml_file]
            ),
        
        # UTILS
        #=========
        # RVIZ
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            prefix="xterm -e",
            arguments=['-d' + rviz_file]
            ),

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
            prefix='xterm -e',
            parameters=[params_yaml_file]            
            ),

        # HRI
        Node(
            package='web_hri',
            executable='web_hri',
            name='web_hri',
            output='screen',
            prefix='xterm -e',
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
            prefix="xterm -e",
            parameters=[params_yaml_file]            
            ),
        
        # Status Publisher
        Node(
            package='robot_status_publisher',
            executable='robot_status_publisher_node',
            name='status_publisher',
            output='screen',
            parameters=[params_yaml_file]
            )
    ]) #end LaunchDescription
