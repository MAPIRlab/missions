import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import xacro

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    pkg_dir = get_package_share_directory("missions_pkg")
    

    # variables/files
    use_sim_time = True
    remappings = []
    params_yaml_file = ParameterFile( os.path.join(pkg_dir, 'launch', 'simbot', 'simbot_params.yaml'), allow_substs=True)
    nav2_launch_file = os.path.join(pkg_dir, 'launch', 'simbot', 'nav2_launch.py')
    apriltags_launch_file = os.path.join(pkg_dir, 'launch', 'simbot', 'apriltags_launch.py')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'coppelia_sim.rviz')    
    urdf = os.path.join(pkg_dir, 'launch', 'rhodon', 'rhodon_urdf.xml')
    namespace = LaunchConfiguration('namespace').perform(context)

    
    # RVIZ2
    #============
    rviz=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            prefix="xterm -hold -e",
            arguments=['-d' + rviz_file],
            remappings=[
                ("/initialpose", "/simbot/initialpose"),
                ("/goal_pose", "/simbot/goal_pose")
            ]    
        ),
    ]

    # SIMULATOR
    #============
    sim_nodes = [
        Node(
            package='coppelia_ros2_pkg',
            executable='coppelia_simulator',
            name='coppelia_simulator',
            output='screen',
            prefix="xterm -hold -e",
            parameters = [params_yaml_file]
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
    ]

    # URDF model (TFs)
    robot_desc = xacro.process_file(os.path.join(pkg_dir, 'launch', 'rhodon', 'rhodon.xacro'), mappings={'frame_ns': namespace})
    robot_desc = robot_desc.toprettyxml(indent='  ')
    robot_state_publisher = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf],
            prefix="xterm -hold -e",
            ),
    ]

    # MANAGEMENT
    #===========
    task_manager = [
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
    ]    


    # NAV2 Autonomous Navigation (see nav2_launch.py)
    navigation = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file)
        ),
    ]


    # MQTT_bridge
    mqtt = [
        Node(
            package='mqtt_bridge',
            executable='mqtt_bridge_node',
            name='mqtt_bridge',
            output='screen',
            prefix='xterm -hold -e',
            parameters=[params_yaml_file]            
            ),
    ]

    # HRI
    HRI = [
        Node(
            package='web_hri',
            executable='web_hri',
            name='web_hri',
            output='screen',
            prefix='xterm -hold -e',
            parameters=[params_yaml_file]
            ),
    ]

    # PATROL
    patrol= [
        Node(
            package='patrol',
            executable='patrol_times',
            name='patrol_times',
            output='screen',
            parameters=[params_yaml_file]
            ),
    ]
    
    # Battery_Manager
    battery_manager = [
        Node(
            package='battery_manager',
            executable='battery_manager',
            name='battery_manager',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file]            
            ),
    ] 
    
    # Status Publisher
    status_publisher= [
        Node(
            package='robot_status_publisher',
            executable='robot_status_publisher_node',
            name='status_publisher',
            output='screen',
            parameters=[params_yaml_file]
            ), 
    ]

    # SET what to launch
    actions=[PushRosNamespace(namespace)]
    #actions.extend(sim_nodes)
    actions.extend(robot_state_publisher)
    #actions.extend(task_manager)
    actions.extend(rviz)
    #actions.extend(navigation)
    #actions.extend(mqtt)
    #actions.extend(HRI)
    #actions.extend(patrol)
    #actions.extend(battery_manager)
    #actions.extend(status_publisher)
    return[
        GroupAction
        (
            actions=actions
        )
    ]


def generate_launch_description():

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),

        DeclareLaunchArgument(
            "coppelia_ros2_pkg",
            default_value = get_package_share_directory('coppelia_ros2_pkg'),
            description = "Coppelia_pkg path",
            ),

        DeclareLaunchArgument('namespace', default_value="simbot"),
        OpaqueFunction(function = launch_setup)
    ])