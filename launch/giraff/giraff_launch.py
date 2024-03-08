# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    my_dir = os.path.join(get_package_share_directory('missions_pkg'), 'launch', 'giraff')
    namespace = LaunchConfiguration('namespace').perform(context)

     # common variables

    params_yaml_file = ParameterFile( os.path.join(my_dir, 'giraff_params.yaml'), allow_substs=True)
    

    giraff_driver = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('giraff_ros2_driver'), 'launch', 'giraff_launch.py')
            ),
            launch_arguments={
                'publish_odom': 'True' # using RF2O instead (in nav2_launch.py)
            }.items()
        )
    ]

    hokuyo_node = [
        Node(
        package='urg_node',
        executable='urg_node_driver',
        name='hokuyo_front',
        #prefix='xterm -hold -e',
        output='screen',
        parameters=[params_yaml_file]
        ),  
    ]

    navigation_nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_dir, 'nav2_launch.py')
            )
        )
    ]

    rviz = [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d' +  os.path.join(get_package_share_directory('missions_pkg'), 'rviz', 'giraff.rviz')],
            prefix="xterm -hold -e",
            remappings=[
                ("/initialpose", "/giraff/initialpose"),
                ("/goal_pose", "/giraff/goal_pose")
            ]
        ),
    ]

    mqtt = [
        Node(
            package='mqtt_bridge',
            executable='mqtt_bridge_node',
            name='mqtt_bridge',
            output='screen',
            #prefix='xterm -hold -e',
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

    reactive_robot2023 = [
        Node(
            package='robot2023',
            executable='reactive_follower',
            name='reactive_follower',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[
                {"/follower/offsetDistance" : 1.5},
                {"/follower/linearSpeed" : 0.3},
                {"/follower/directionTolerance" : 0.1},
                {"/follower/local_frame_id" : "giraff_base_link"},
                {"/follower/master_loc_topic" : "/rhodon/status"},
            ]  
        ),
    ]

    start_async_slam_toolbox_node = [
        Node(
            parameters=[
            params_yaml_file,
            {'use_sim_time': False}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            prefix="xterm -hold -e",
            output='screen'
        )
        # To save the map run in a different terminal:
        #   ros2 run nav2_map_server map_saver_cli -f my_map
        # Does not need the map server to be running
    ]

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

    PID  =[
        Node(
            package="minirae_lite",
            executable="minirae_lite_node",
            name="minirae_lite",
            prefix="xterm -hold -e",
            parameters=[
                {"port":"/dev/ttyUSB0"}
            ]
        )
    ] 

    anemometer  =[
        Node(
            package="windsonic",
            executable="windsonic_node",
            name="windsonic",
            prefix="xterm -hold -e",
            parameters=[
                {"port":"/dev/ttyUSB1"},
                {"frame_id":"giraff_base_link"}
            ]
        )
    ] 

    actions=[PushRosNamespace(namespace)]
    #actions.extend(robot_state_publisher)
    #actions.extend(rviz)
    #actions.extend(mqtt)
    #actions.extend(status_publisher)
    #actions.extend(reactive_robot2023)
    #actions.extend(start_async_slam_toolbox_node)
    
    #actions.extend(giraff_driver)
    #actions.extend(hokuyo_node)
    #actions.extend(PID)
    #actions.extend(anemometer)
    actions.extend(navigation_nodes)
    actions.extend(keyboard_control)
    return[
        GroupAction
        (
            actions=actions
        ),
    ]

def generate_launch_description():

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),
        DeclareLaunchArgument('namespace', default_value="giraff"),
        OpaqueFunction(function = launch_setup)
    ])
