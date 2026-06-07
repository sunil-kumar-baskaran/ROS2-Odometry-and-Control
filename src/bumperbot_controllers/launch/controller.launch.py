from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def noisy_controller(context, *args, **kargs):
    wheel_radius_conf = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation_conf = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error_conf = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error_conf = float(LaunchConfiguration("wheel_separation_error").perform(context))

    noisy_controller_py = Node(
        package="bumperbot_controllers",
        executable="noisy_controller.py",
        parameters=[
            {"wheel_radius": wheel_radius_conf + wheel_radius_error_conf,
            "wheel_separation": wheel_separation_conf + wheel_separation_error_conf}
            ]
    )
    return [noisy_controller_py]


def generate_launch_description():
   
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    use_python_arg = DeclareLaunchArgument(
        name="use_python",
        default_value = "True"
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        name="wheel_radius",
        default_value="0.033"
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        name="wheel_separation",
        default_value="0.17"
    )

    use_simple_vel_cntrlrs_arg = DeclareLaunchArgument(   
        name="use_simple_vel_controllers",
        default_value="True"
    )
    
    wheel_radius_error_arg = DeclareLaunchArgument(
        name="wheel_radius_error",
        default_value="0.005"
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        name="wheel_separation_error",
        default_value="0.02"
    )
    
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_python_conf = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_vel_cntrlrs_conf = LaunchConfiguration("use_simple_vel_controllers")


    
    joint_state_broadcaster_spawner =  Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager"
        ],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    bumperbots_diff_drive_controller_spawner =  Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
                    "bumperbots_diff_drive_controller",
                    "--controller-manager",
                    "/controller_manager"
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_simple_vel_cntrlrs_conf)
    )
    
    use_simple_vel_cntrlrs = GroupAction(
        condition= IfCondition(use_simple_vel_cntrlrs_conf),
        actions=[
            Node(   # USES PREWRITTEN ROS2_CONTROLLERS SIMPLE_VELOCITY_CONTROLLER PKG
                package="controller_manager",
                executable="spawner",
                arguments=[
                            "simple_velocity_controller",
                            "--controller-manager",
                            "/controller_manager"
                            ],
                parameters=[{"use_sim_time": use_sim_time}]
            ),

            Node(      # USES OUR OWN CONTROLLER
                package="bumperbot_controllers",
                executable="simple_controller.py",
                parameters=[{"wheel_radius":wheel_radius, 
                            "wheel_separation":wheel_separation,
                            "use_sim_time": use_sim_time}], #"use_sim_time": use_sim_time
                condition=IfCondition(use_python_conf)
            )
        ]
    )

    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    
    
    return LaunchDescription([
        # 1. ALWAYS declare arguments first
            use_sim_time_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            use_python_arg,
            use_simple_vel_cntrlrs_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
        # 2. Then list the nodes that use them
            joint_state_broadcaster_spawner,
            bumperbots_diff_drive_controller_spawner,
            use_simple_vel_cntrlrs,
            noisy_controller_launch
    ])
       
# In ROS 2, the LaunchDescription processes items in the order they appear in the list. 
# If a Node (which uses a LaunchConfiguration) [here simple_velocity controller uses use_pyhton_conf] is listed before the  
# DeclareLaunchArgument that creates it, the launch system panics because it hasn't "registered" that variable name yet.