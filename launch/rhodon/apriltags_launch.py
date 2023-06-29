import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


pkg_dir = get_package_share_directory('missions_pkg')
params_yaml_file = ParameterFile(os.path.join(pkg_dir, 'launch', 'rhodon', 'apriltags_params.yaml'), allow_substs=True)


def generate_launch_description():

    # v4l2_camera (sudo apt install ros-humble-v4l2-camera)
    composable_v4l2_camera = ComposableNode(        
        package = "v4l2_camera",
        plugin = "v4l2_camera::V4L2Camera",
        name = "v4l2_camera",
        namespace = "v4l2",
        parameters = [{ 'video_device': '/dev/video0',
                        'camera_name': 'owlotech_camera',
                        'pixel_format': 'YUYV',
                        'output_encoding': 'rgb8',
                        'image_size': [640, 480],
                        'camera_info_url': 'file:///home/mapir/.ros/camera_info/owlotech_camera.yaml',
                        'publish_rate': 30
                        }],
        extra_arguments=[{'use_intra_process_comms': True}]
        )
    
    # Image Rect (sudo apt install ros-humble-image-proc)
    composable_image_proc = ComposableNode(        
        package = "image_proc",
        plugin = "image_proc::RectifyNode",
        name = "rectify",
        namespace = "v4l2",        
        remappings=[("image", "image_raw"), ("camera_info", "camera_info")],
        extra_arguments=[{'use_intra_process_comms': True}]
        )
    
    # AprilTags comes as a component
    composable_apriltag = ComposableNode(        
        package = "apriltag_ros",
        plugin = "AprilTagNode",
        name = "apriltag",
        namespace =  "apriltag",
        parameters=[params_yaml_file],
        remappings=[("/apriltag/image_rect", "/v4l2/image_rect"), ("/apriltag/camera_info", "/v4l2/camera_info")],
        extra_arguments=[{'use_intra_process_comms': True}]
        )
    
    #===========
    # CONTAINER
    #===========
    container = ComposableNodeContainer(        
        package = "rclcpp_components",
        executable = "component_container",
        name = "tag_container",
        namespace = "",
        composable_node_descriptions=[composable_v4l2_camera, composable_image_proc, composable_apriltag],
        output = "both",
        prefix = "xterm -hold -e"
        )
    
    return launch.LaunchDescription([container])