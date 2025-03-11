import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution

#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument("", default_value=""),
   ]
#==========================

def launch_setup(context, *args, **kwargs):

    serverIP = "150.214.109.133"

    tf = Node(
        package="tf_transfer",
        executable="tf_client",
        prefix ="xterm -hold -T tf -e",
        parameters=[
            {"protocol": "UDP"},
            {"serverPort": 15760},
            {"serverIP": serverIP},
            {"topic": "/tf"},
            {"isServerSocket": False},
        ],
    )

    map = Node(
        package="nav2_transfer",
        executable="mapClient",
        prefix ="xterm -hold -e",
        parameters=[
            {"protocol": "TCP"},
            {"serverPort": 15770},
            {"serverIP": serverIP},
            {"topic": "/giraff/map"},
            {"isServerSocket": False},
        ],
    )


    nav2 = Node(
        package="nav2_transfer",
        executable="navToPoseServer",
        prefix ="xterm -hold -e",
        parameters=[
            {"protocol": "TCP"},
            {"serverPort": 15780},
            {"serverIP": serverIP},
            {"actionServer": "/giraff/navigate_to_pose"},
            {"isServerSocket": False},
        ],
    )

    initialPose = Node(
        package="tf_transfer",
        executable="poseWithCovarianceStamped_server",
        prefix ="xterm -hold -e",
        parameters=[
            {"protocol": "TCP"},
            {"serverPort": 15790},
            {"serverIP": serverIP},
            {"topic": "/giraff/initialpose"},
            {"isServerSocket": False},
        ],
    )

    laser = Node(
        package="laser_scan_transfer",
        executable="client",
        prefix ="xterm -hold -T laser -e",
        parameters=[
            {"protocol": "UDP"},
            {"serverPort": 15800},
            {"serverIP": serverIP},
            {"topic": "/giraff/laser_scan"},
            {"isServerSocket": False},
            {"maxFrequency" : 2}
        ],
    )

    camera = [
        Node(
            package="image_transfer",
            executable="client_info",
            prefix="xterm -hold -T cameraInfo -e",
            parameters=[
                {"protocol": "UDP"},
                {"serverIP": serverIP},
                {"serverPort": 15801},
                {"topic": "/giraff/camera/color/camera_info"},
                {"isServerSocket": False},
                {"maxFrequency" : 10}
            ],
        ),
        Node(
            package="image_transfer",
            executable="client_compressed",
            prefix="xterm -hold -T rgb -e",
            parameters=[
                {"protocol": "UDP"},
                {"serverIP": serverIP},
                {"serverPort": 15802},
                {"topic": "/giraff/camera/color/image_compressed"},
                {"isServerSocket": False},
                {"maxFrequency" : 10}
            ],
        ),
        Node(
            package="image_transfer",
            executable="client_compressed",
            prefix="xterm -hold -T depth -e",
            parameters=[
                {"protocol": "UDP"},
                {"serverIP": serverIP},
                {"serverPort": 15803},
                {"topic": "/giraff/camera/depth/image_compressed"},
                {"isServerSocket": False},
                {"maxFrequency" : 10}
            ],
        )
    ]

    amcl = Node(
        package="tf_transfer",
        executable="poseWithCovarianceStamped_client",
        prefix="xterm -hold -T amcl -e ",
        parameters=[
            {"protocol": "TCP"},
            {"serverIP": serverIP},
            {"serverPort": 15791},
            {"topic": "/giraff/amcl_pose"},
            {"isServerSocket": False},
        ],
    )

    cmd_vel = Node(
        package="nav2_transfer",
        executable="twistServer",
        prefix="xterm -hold -e ",
        parameters=[
            {"protocol": "UDP"},
            {"serverIP": serverIP},
            {"serverPort": 15792},
            {"topic": "/giraff/cmd_vel"},
            {"isServerSocket": False},
        ],
    )

    nodes = []
    nodes.append(tf)
    nodes.append(map)
    nodes.append(nav2)
    nodes.append(initialPose)
    nodes.append(laser)
    nodes.extend(camera)
    nodes.append(amcl)
    nodes.append(cmd_vel)
    return nodes



def generate_launch_description():

    launch_description = [
       # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
   ]
   
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
   
    return  LaunchDescription(launch_description)