from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    model_package_dir = get_package_share_directory("bumper_bot_description")

    ros_distro =  os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
 
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("bumper_bot_description"), "urdf", "bumperbot.urdf.xacro"),
        description="Path to the URDF Description"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model"),
                                                " is_ignition:=", 
                                                is_ignition
                                                ]), 
                                                value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    gazebo_resource_path =  SetEnvironmentVariable(
        name = "GZ_SIM_RESOURCE_PATH",  #REQUIRES A FOLDER, SO USE PATH() .PARENT.RESOLVE() TO FIND A FOLDER, OS.PATH.JOIN(.......BUMPERBOT.URDF.XACRO) LEADS TO A SINGLE FILE
        value = [
                str(Path(model_package_dir).parent.resolve())
                ]
    )
    
    gazebo =  IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
                ]),
                launch_arguments= [
                    ("gz_args", [" -v 4", " -r", " empty.sdf"])
                ]
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output = "screen",
        arguments=["-name", "bumperbot",
                   "-topic", "robot_description"]
    )

    # FIXED: Added the explicit Ros-Gazebo Clock Bridge Node
    # This maps the internal Gazebo clock over to the native ROS 2 /clock topic
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen"
    )

    '''
    controller_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("bumperbot_controllers"), "launch", "controller.launch.py")
    ]),
    launch_arguments={'use_sim_time': 'True'}.items(),
    )

    delay_controller_launch = TimerAction(
        period=5.0,
        actions=[controller_launch]
    )

    
    delay_controller_after_spawn = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=gz_spawn_entity,
        on_exit=[controller_launch],
        )
    )
    '''
    return LaunchDescription([ 
         model_arg,
         robot_state_publisher,
         gazebo_resource_path,
         gazebo,
         gz_spawn_entity,
         ros_gz_bridge,
         #delay_controller_after_spawn
    ])