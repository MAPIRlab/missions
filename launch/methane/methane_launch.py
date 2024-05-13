import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
import xacro

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    pkg_dir = get_package_share_directory("missions_pkg")

    # params file
    params_yaml_file = ParameterFile( os.path.join(pkg_dir, 'launch', 'methane', 'methane_params.yaml'), allow_substs=True)

    # set Namespace
    namespace = LaunchConfiguration('namespace').perform(context)
    
    # URDF model (TFs)
    setup_desc = xacro.process_file(os.path.join(pkg_dir, 'launch', 'methane', 'tdlas.xacro'), mappings={'frame_ns': namespace})
    setup_desc = setup_desc.toprettyxml(indent='  ')
    robot_state_publisher = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': setup_desc}],
        )
    ]
    
        
    # PTU Interbotix (USB0)
    interbotix_xsturret_control_path = get_package_share_directory("interbotix_xsturret_control")
    DeclareLaunchArgument('interbotix_xsturret_control_path', default_value=interbotix_xsturret_control_path)
    ptu_interbotix = [
        Node(
            package='interbotix_xs_sdk',
            executable='xs_sdk',
            name='interbotix_xs_sdk',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[{
                'motor_configs': os.path.join(interbotix_xsturret_control_path, 'config', 'wxxms.yaml'),
                'mode_configs': os.path.join(interbotix_xsturret_control_path, 'config', 'modes.yaml'),
                'load_configs': False
            }],
            ),
    ]   


    # FALCON Methane Detector (USB1)
    falcon_tdlas = [
        Node(
            package='falcon_tdlas',
            executable='falcon_tdlas',
            name='falcon_tdlas',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[{
                "port": "/dev/ttyUSB1",
                "topic": "/falcon/reading",
                "frequency": 10.0,
                "verbose": True                
            }]
        ),

        # RQT plot
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='falcon_plot',
            output='screen',
            prefix="xterm -hold -e",
            arguments=[
                #"/falcon/reading/average_ppmxm", "/falcon/reading/average_reflection_strength", "/falcon/reading/average_absorption_strength"
                "/falcon/reading/average_ppmxm"
            ],
        ),
    ]

    # RGB Camera (Video0)
    usb_cam = [
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='camera',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file],
            arguments=[],
            remappings=[]    
        ),
    ]


    # GPS DELUO NMEA (USB3)
    GPSdriver = [
        Node(
            package='gps2cartesian',
            executable='fakeGPSpub',
            name='fakeGPS_tdlas',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file]
        ),

         Node(
            package='gps2cartesian',
            executable='fakeGPSpub',
            name='fakeGPS_hunter',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file]
        ),

        #Node(
        #    package='nmea_navsat_driver',
        #    executable='nmea_serial_driver',
        #    name='nmea_serial_driver',
        #    namespace= "deluo",
        #    output='screen',
        #    prefix="xterm -hold -e",
        #    parameters=[params_yaml_file],
        #)
    ]

    # Find Aruco
    aruco= [
        Node(
            package="aruco",
            executable="findAruco",
            prefix="xterm -hold -e",
            parameters=[
                {"markerLength": 0.3},
                {"imageTopic":"camera/image_raw"},
                {"cameraInfoTopic":"camera/camera_info"},
            ],
        )
    ]

    # Track Aruco with PTU
    ptu_tracking = [
        Node(
            package='ptu_tracking',
            executable='ptu_tracking',
            name='ptu_tracking',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file],
            ),
    ]
    
    

    # Data Logger (for GDM with TDLAS)
    measurement_logger = [
        Node(
            package='robot2023',
            executable='log_measurements',
            name='log_measurements',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[
                {"file_path" : "/home/mapir/mapir_ws/measurement_log1"},
            ]
        )
    ]

    #gps2cartesian
    gps2cartesian = [
        Node(
            package='gps2cartesian',
            executable='gps2cartesian',
            name='gps2cartesian',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file]
        )        
    ]

    # RVIZ
    rviz_file = os.path.join(pkg_dir, 'rviz', 'tdlas.rviz')
    rviz=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            prefix="xterm -hold -e",
            arguments=['-d' + rviz_file],
            remappings=[]    
        ),
    ]

   
    # SELECT COMPONENTS TO LAUNCH
    #=============================
    actions=[PushRosNamespace(namespace)]
    # HW
    actions.extend(robot_state_publisher)
    actions.extend(ptu_interbotix)
    actions.extend(usb_cam)
    actions.extend(falcon_tdlas)
    actions.extend(GPSdriver)
    # SW
    actions.extend(aruco)
    actions.extend(ptu_tracking)
    actions.extend(gps2cartesian)
    #actions.extend(measurement_logger)
    actions.extend(rviz)
    
    
    return[
        GroupAction
        (
            actions=actions
        ),
    ]


# LAUNCH DESCRIPTION
def generate_launch_description():

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),
        # MAIN NAMESPACE
        DeclareLaunchArgument('namespace', default_value="methane"),
        # SETUP NODES
        OpaqueFunction(function = launch_setup)
    ])