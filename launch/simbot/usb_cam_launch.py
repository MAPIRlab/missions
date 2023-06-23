import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get the pkg directory
    pkg_dir = get_package_share_directory('missions_pkg')
    
    # variables/files
    use_sim_time = True    
    params_yaml_file = os.path.join(pkg_dir, 'launch', 'simbot', 'apriltags_params.yaml')
    logger = LaunchConfiguration("log_level")
    image_topic = ["/image_raw"]
    info_topic = ["/camera_info"]
    
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        # Declare Arguments 
        # ==================
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),
        DeclareLaunchArgument(
            "pkg_dir",
            default_value = pkg_dir,
            description = "this pkg dir",
            ),
        
        # HW_DRIVERS
        #============
        # USB-CAM
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            prefix="xterm -hold -e",
            parameters = [ParameterFile(params_yaml_file, allow_substs=True)]
            ),

        # USB-CAM (viewer)
        Node(
            package='usb_cam',
            executable='show_image.py',
            name='usb_cam_viewer',
            output='screen',
            prefix="xterm -hold -e",
            parameters = [ParameterFile(params_yaml_file, allow_substs=True)]
            )

        # For calibration
        #  ros2 run camera_calibration cameracalibrator --size 7x7 --square 0.03 --ros-args -r image:=/image_raw -p camera:=/laptop_camera        

    ]) #end LaunchDescription
