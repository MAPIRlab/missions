import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import xacro

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    pkg_dir = get_package_share_directory("missions_pkg")
    params_yaml_file = ParameterFile( os.path.join(pkg_dir, 'launch', 'simbot', 'apriltags_params.yaml'), allow_substs=True)
    urdf = os.path.join(pkg_dir, 'launch', 'rhodon', 'rhodon_urdf.xml')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'coppelia_sim.rviz')
    namespace = LaunchConfiguration('namespace').perform(context)

    interbotix_xsturret_control_path = get_package_share_directory("interbotix_xsturret_control")
    DeclareLaunchArgument('interbotix_xsturret_control_path', default_value=interbotix_xsturret_control_path)

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

    apriltag = [
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
                                    'camera_name': 'owlotech_camera',
                                    'pixel_format': 'YUYV',
                                    'output_encoding': 'rgb8',
                                    'image_size': [640, 480],
                                    'camera_info_url': 'file:///home/jgmonroy/.ros/camera_info/owlotech_camera.yaml',
                                    'publish_rate': 30
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
                    namespace =  "apriltag",
                    parameters=[params_yaml_file],
                    remappings=[("/simbot/apriltag/image_rect", "/simbot/v4l2/image_rect"), ("/simbot/apriltag/camera_info", "/simbot/v4l2/camera_info")],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output = "both",
            prefix = "xterm -hold -e"
        ),
    ]    

    ptu_d46 = [
        Node(
            package='hal_flir_d46',
            executable='hal_flir_d46',
            name='hal_flir_d46',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[params_yaml_file],
            ),
    ]
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

    # ===================
    # SET what to launch
    # ===================
    actions=[PushRosNamespace(namespace)]
    #actions.extend(ptu_d46)
    actions.extend(ptu_interbotix)
    actions.extend(rviz)
    actions.extend(apriltag)
    actions.extend(robot_state_publisher)
    actions.extend(ptu_tracking);
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

        DeclareLaunchArgument('namespace', default_value="simbot"),
        OpaqueFunction(function = launch_setup)
    ])



    