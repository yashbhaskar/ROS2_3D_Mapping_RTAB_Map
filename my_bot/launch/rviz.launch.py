from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot_description_val= ParameterValue(Command(["xacro ",os.path.join(get_package_share_directory("my_bot"),"models","urdf","my_bot.xacro")]),value_type=str)
 
    a = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='screen',
        parameters=[{"robot_description": robot_description_val}]

    )
    b = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    c = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )
    return LaunchDescription([
        a,
        b,
        c
    
    ])