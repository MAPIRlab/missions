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
import xacro

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
            )
        )
    ]

    hokuyo_node = [
        Node(
        package='urg_node',
        executable='urg_node_driver',
        name='hokuyo_front',
        prefix='xterm -hold -e',
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

    #robot description for state_pùblisher
    robot_desc = xacro.process_file(os.path.join(my_dir, 'giraff.xacro'), mappings={'frame_ns': namespace})
    robot_desc = robot_desc.toprettyxml(indent='  ')

    robot_state_publisher = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'use_sim_time': True, 
                    'robot_description': robot_desc
                }
            ],
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

    actions=[PushRosNamespace(namespace)]
    actions.extend(giraff_driver)
    actions.extend(robot_state_publisher)
    actions.extend(rviz)
    actions.extend(navigation_nodes)
    actions.extend(hokuyo_node)
    actions.extend(mqtt)
    actions.extend(status_publisher)
    #actions.extend(start_async_slam_toolbox_node)
    #actions.extend(keyboard_control)
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
        
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),
        DeclareLaunchArgument('namespace', default_value="giraff"),
        OpaqueFunction(function = launch_setup)
    ])
