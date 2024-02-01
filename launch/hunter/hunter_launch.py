import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    pkg_dir = get_package_share_directory("missions_pkg")
    params_yaml_file = ParameterFile( os.path.join(pkg_dir, 'launch', 'hunter', 'hunter_params.yaml'), allow_substs=True)

    # NameSpace
    namespace = LaunchConfiguration('namespace').perform(context)
    
    
    # OUSTER 3D LIDAR (10Hz)
    ouster_pkg_dir = get_package_share_directory("ros2_ouster")
    ouster_launch_file = os.path.join(ouster_pkg_dir, 'launch', 'driver_launch.py')
    ouster3DLidar = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ouster_launch_file)
        ),
    ] 
            
    
    # ASTRA RGB-D (10Hz)
    astra_pkg_dir = get_package_share_directory("astra_camera")
    astra_launch_file = os.path.join(astra_pkg_dir, 'launch', 'multi_astra_mini.launch.py')
    astraRGBDcamera = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(astra_launch_file)
        ),
    ]

    # NMEA GPS (1Hz)
    nmea_gps_pkg_dir = get_package_share_directory("nmea_navsat_driver")
    nmea_gps_launch_file = os.path.join(nmea_gps_pkg_dir, 'launch', 'nmea_serial_driver.launch.py')
    nmeaGPSnavsat = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nmea_gps_launch_file)
        ),
    ]

    # RVIZ
    rviz_file = os.path.join(pkg_dir, 'rviz', 'hunter.rviz')    
    rviz=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            prefix="xterm -hold -e",
            arguments=['-d' + rviz_file],
            remappings=[
                ("/initialpose", "/hunter/initialpose"),
                ("/goal_pose", "/hunter/goal_pose")
            ]    
        ),
    ]
    
    
    

    actions=[PushRosNamespace(namespace)]
    actions.extend(ouster3DLidar)
    actions.extend(astraRGBDcamera)
    actions.extend(nmeaGPSnavsat)
    actions.extend(rviz)
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
        DeclareLaunchArgument('namespace', default_value="hunter"),
        OpaqueFunction(function = launch_setup)
    ])