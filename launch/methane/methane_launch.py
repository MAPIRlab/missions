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
    
        
    # PTU Interbotix
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


    # Camera and AprilTags
    compos_usbcam_apriltags = [
        # sudo apt install ros-humble-apriltag
        # sudo apt install ros-humble-apriltag-msgs
        # sudo apt install ros-humble-apriltag-ros
        #===========
        # CONTAINER
        #===========
        ComposableNodeContainer(        
            package = "rclcpp_components",
            executable = "component_container",
            name = "tag_container",
            namespace = "",
            composable_node_descriptions=[
                # v4l2_camera
                ComposableNode(        
                    package = "v4l2_camera",
                    plugin = "v4l2_camera::V4L2Camera",
                    name = "v4l2_camera",
                    namespace = "v4l2",
                    parameters = [{ 'video_device': '/dev/video0',
                                    'camera_name': 'camera',
                                    'pixel_format': 'YUYV',
                                    'output_encoding': 'rgb8',
                                    'image_size': [1920, 1080],
                                    'camera_info_url': 'file:///home/mapir/.ros/camera_info/owlotech_camera.yaml',
                                    'publish_rate': 30,
                                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                
                # Image Rect
                ComposableNode(        
                    package = "image_proc",
                    plugin = "image_proc::RectifyNode", 
                    name = "rectify",
                    namespace = "v4l2",        
                    remappings=[("image", "image_raw"), ("camera_info", "camera_info")],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                
                # AprilTags comes as a component
                ComposableNode(        
                    package = "apriltag_ros",
                    plugin = "AprilTagNode",
                    name = "apriltag",
                    namespace = "apriltag",
                    parameters=[params_yaml_file],
                    remappings=[("/methane/apriltag/image_rect", "/methane/v4l2/image_rect"), ("/methane/apriltag/camera_info", "/methane/v4l2/camera_info")],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output = "both",
            prefix = "xterm -hold -e"
        ),
    ]    


    # FALCON Methane Detector
    falcon_tdlas = [
        Node(
            package='falcon_tdlas',
            executable='falcon_tdlas',
            name='falcon_tdlas',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[{
                "port": "/dev/ttyUSB0",
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


    # Track AprilTags with PTU
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
    
    # GPS NMEA
    nmeaGPSnavsat = [
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file],
        )
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


    # Status Publisher (for MQTT)
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

    # Camera alone
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

        # Image Rect
        Node(        
            package = "image_proc",
            executable = "rectify_node", 
            name = "rectify_node",
            namespace = "camera",
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file],
            remappings=[
                ("image", "image_raw"),
                ("camera_info", "camera_info")
            ],
        ),

        # Apriltags
        Node(        
            package = "apriltag_ros",
            executable = "apriltag_node",
            name = "apriltag_node",
            namespace = "camera",
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file],
            remappings=[("image_rect", "image_rect"), ("camera_info", "camera_info")],
        ),
    ]

    # SELECT COMPONENTS TO LAUNCH
    #=============================
    actions=[PushRosNamespace(namespace)]
    actions.extend(robot_state_publisher)
    actions.extend(ptu_interbotix)
    actions.extend(compos_usbcam_apriltags)
    actions.extend(falcon_tdlas)
    actions.extend(ptu_tracking)
    actions.extend(nmeaGPSnavsat)
    #actions.extend(measurement_logger)
    actions.extend(rviz)
    #actions.extend(usb_cam)
    #actions.extend(mqtt)
    #actions.extend(status_publisher)
    
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