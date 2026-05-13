from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
    
def generate_launch_description():
    
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
    
    use_python_conf = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    
    joint_state_broadcaster_spawner =  Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager"
        ]
    )
    
    velocity_controller_spawner =  Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
        ]
    )
    
    simple_controller = Node(
        package="bumperbot_controllers",
        executable="simple_controller.py",
        parameters=[{"wheel_radius":wheel_radius, 
                     "wheel_separation":wheel_separation}],
        condition=IfCondition(use_python_conf)
    )
    
    return LaunchDescription([
        # 1. ALWAYS declare arguments first
            wheel_radius_arg,
            wheel_separation_arg,
            use_python_arg,
        # 2. Then list the nodes that use them
            joint_state_broadcaster_spawner,
            velocity_controller_spawner,
            simple_controller
    ])
       
# In ROS 2, the LaunchDescription processes items in the order they appear in the list. 
# If a Node (which uses a LaunchConfiguration) [here simple_velocity controller uses use_pyhton_conf] is listed before the  
# DeclareLaunchArgument that creates it, the launch system panics because it hasn't "registered" that variable name yet.