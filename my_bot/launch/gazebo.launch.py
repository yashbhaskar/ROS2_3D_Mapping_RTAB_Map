from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    robot_description_val= ParameterValue(Command(["xacro ",os.path.join(get_package_share_directory("my_bot"),"models","urdf","my_bot.xacro")]),value_type=str)
    sdf_path=os.path.join(get_package_share_directory("my_bot"),"worlds", "short.sdf")
    
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_ign_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',          # off when using ekf
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            'lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            ]
    )

    odom=Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="odom_bridge",
        output="screen",
        arguments=['/model/my_bot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        remappings=[('/model/my_bot/odometry','/odom')]                 # Gazebo Odometry
    )

    st_pub=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_val}],
    )
    
    rviz_node= Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )
    
    gazebo=IncludeLaunchDescription(
        PathJoinSubstitution([
         get_package_share_directory("ros_gz_sim"),"launch","gz_sim.launch.py"   
        ]),
        launch_arguments=[("gz_args",[" -r " + sdf_path])]
    )
    
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "my_bot",
            "-x", "0.0",   # X position
            "-y", "0.0",   # Y position
            "-z", "0.0",   # Z position
            "-R", "0.0",   # Roll
            "-P", "0.0",   # Pitch
            "-Y", "0.0"    # Yaw (in radians)
    ],
    output="screen"
    )

    return LaunchDescription([
        gazebo,
        bridge_node,
        odom,
        rviz_node,
        st_pub,
        spawn_entity,
        
    ])
