import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import xacro

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    pkg_dir = get_package_share_directory("missions_pkg")
    params_yaml_file = ParameterFile( os.path.join(pkg_dir, 'launch', 'rhodon', 'rhodon_params.yaml'), allow_substs=True)
    nav2_launch_file = os.path.join(pkg_dir, 'launch', 'rhodon', 'nav2_launch.py')
    apriltags_launch_file = os.path.join(pkg_dir, 'launch', 'rhodon', 'apriltags_launch.py')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'rhodon.rviz')
    namespace = LaunchConfiguration('namespace').perform(context)
    
    # HW_DRIVERS
    #===========    
    driver_nodes =[
        # ARIA
        Node(
            package='ros2aria',
            executable='ros2aria',
            name='ros2aria',
            output='screen',
            prefix='xterm -hold -e',
            parameters=[params_yaml_file]
            ),

        # Hokuyo Front SICK LMS-200 || Hokuyo URG-04LX
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='hokuyo_front',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file]
            ),        
        
        # Hokuyo Back UTM-30LX
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='hokuyo_back',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file]
            ),

    ]

    falcon_tdlas = [
        Node(
            package='falcon_tdlas',
            executable='falcon_tdlas',
            name='falcon_tdlas',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[
                {"port" : "/dev/ttyUSB1"},
                {"topic" : "/falcon/reading"}
            ]
            ),
    ]    

    # URDF model (TFs)
    robot_state_publisher = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'rhodon', 'rhodon_urdf_launch.py')
            )
        )
    ]
            
    # Keyboard Control
    keyboard_control=[
        Node(
            package='keyboard_control',
            executable='keyboard_control_plus',
            name='keyboard_control',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file]
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
    
    # UTILS
    #=========
    # RVIZ
    rviz=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            prefix="xterm -hold -e",
            arguments=['-d' + rviz_file],
            remappings=[
                ("/initialpose", "/rhodon/initialpose"),
                ("/goal_pose", "/rhodon/goal_pose")
            ]    
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

        
        Node(
            package='nav2_over_mqtt',
            executable='mqtt2Nav2',
            name='mqtt2Nav2',
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
            prefix='xterm -hold -e',
            parameters=[params_yaml_file]
            ), 
    ]  
    
    # Camera and AprilTags
    apriltags = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(apriltags_launch_file)
        ),  
    ]

    reactive_robot2023 = [
        Node(
            package='robot2023',
            executable='reactive_master',
            name='reactive_master',
            output='screen',
            prefix='xterm -hold -e',
            parameters=[
                {"/master/linearSpeed" : 0.1},
                {"/master/stoppingDistance" : 0.3},
                {"/master/directionTolerance" : 0.1},
                {"/master/local_frame" : "rhodon_base_link"},
                {"/master/directionTolerance" : 0.1},
                {"/master/directionTolerance" : 0.1},
                {"/master/directionTolerance" : 0.1},
                {"/master/directionTolerance" : 0.1},
            ]  
        ),
    ]
    actions=[PushRosNamespace(namespace)]
    actions.extend(driver_nodes)
    actions.extend(robot_state_publisher)
    #actions.extend(keyboard_control)
    #actions.extend(task_manager)
    actions.extend(rviz)
    actions.extend(navigation)
    actions.extend(mqtt)
    #actions.extend(HRI)
    #actions.extend(patrol)
    #actions.extend(battery_manager)
    actions.extend(status_publisher)
    actions.extend(reactive_robot2023)
    #actions.extend(falcon_tdlas)
    #actions.extend(apriltags)
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
        DeclareLaunchArgument('namespace', default_value="rhodon"),
        OpaqueFunction(function = launch_setup)
    ])
