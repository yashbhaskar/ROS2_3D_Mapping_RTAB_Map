# 3D Lidar Only Mapping

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Base frame of the robot (updated to match your TF tree)
    frame_id = LaunchConfiguration('frame_id')

    # IMU topic (disabled as not used)
    imu_topic = LaunchConfiguration('imu_topic')
    imu_used = imu_topic.perform(context) != ''

    # RGB-D topics (disabled as not used)
    rgbd_image_topic = LaunchConfiguration('rgbd_image_topic')
    rgbd_images_topic = LaunchConfiguration('rgbd_images_topic')
    rgbd_image_used = (
        rgbd_image_topic.perform(context) != '' or rgbd_images_topic.perform(context) != ''
    )
    rgbd_cameras = 0 if rgbd_images_topic.perform(context) != '' else 1

    # Voxel size for downsampling (adjusted for better indoor resolution)
    voxel_size = LaunchConfiguration('voxel_size')
    voxel_size_value = float(voxel_size.perform(context))

    # Use real-world time (no simulation)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # LiDAR topic (updated to match your 3D LiDAR)
    lidar_topic = LaunchConfiguration('lidar_topic')
    lidar_topic_value = lidar_topic.perform(context)
    lidar_topic_deskewed = lidar_topic_value + "/deskewed"

    # Localization mode (default off for mapping)
    localization = LaunchConfiguration('localization').perform(context)
    localization = localization == 'true' or localization == 'True'

    # Deskewing settings (enabled by default, can be disabled if TF issues persist)
    deskewing = LaunchConfiguration('deskewing').perform(context)
    deskewing = deskewing == 'true' or deskewing == 'True'

    deskewing_slerp = LaunchConfiguration('deskewing_slerp').perform(context)
    deskewing_slerp = deskewing_slerp == 'true' or deskewing_slerp == 'True'

    # Fixed frame from IMU (disabled since IMU is not used)
    fixed_frame_from_imu = False
    fixed_frame_id = LaunchConfiguration('fixed_frame_id').perform(context)
    if not fixed_frame_id and imu_used:
        fixed_frame_from_imu = True
        fixed_frame_id = frame_id.perform(context) + "_stabilized"

    # Disable deskewing if no fixed frame or deskewing is off
    if not fixed_frame_id or not deskewing:
        lidar_topic_deskewed = lidar_topic

    # Rule of thumb: max correspondence distance is 10x voxel size
    max_correspondence_distance = voxel_size_value * 10.0

    # Shared parameters for all nodes
    shared_parameters = {
        'use_sim_time': use_sim_time,
        'frame_id': frame_id,
        'qos': LaunchConfiguration('qos'),
        'approx_sync': rgbd_image_used,  # Disabled since rgbd_image_used is false
        'wait_for_transform': 0.2,  # Adjust if TF delays (e.g., 180-200s) occur

        # RTAB-Map internal parameters
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '50',  # Increased from 10 for better accuracy
        'Icp/VoxelSize': str(voxel_size_value),
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/MaxTranslation': '3',
        'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.7',
    }

    # ICP odometry parameters
    icp_odometry_parameters = {
        'expected_update_rate': LaunchConfiguration('expected_update_rate'),
        'deskewing': not fixed_frame_id and deskewing,  # Disabled without IMU
        # 'odom_frame_id': 'icp_odom',
        'odom_frame_id': 'odom',
        'guess_frame_id': 'odom',  # Updated to use your /odom topic via TF
        'deskewing_slerp': deskewing_slerp,

        # RTAB-Map internal parameters
        'Odom/ScanKeyFrameThr': '0.4',
        'OdomF2M/ScanSubtractRadius': str(voxel_size_value),
        'OdomF2M/ScanMaxSize': '15000',
        'OdomF2M/BundleAdjustment': 'false',
        'Icp/CorrespondenceRatio': '0.01'
    }

    # RTAB-Map parameters
    rtabmap_parameters = {
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_odom_info': True,  # Enabled to use odometry info
        'subscribe_scan_cloud': True,
        'map_frame_id': 'map',  # Changed to standard 'map' frame
        'odom_sensor_sync': False,  # Disabled since no camera

        # RTAB-Map internal parameters
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/CreateOccupancyGrid': 'false',
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
        'Reg/Strategy': '1',
        'Icp/CorrespondenceRatio': str(
            LaunchConfiguration('min_loop_closure_overlap').perform(context)
        ),
    }

    # Arguments for RTAB-Map (delete database if not localization)
    arguments = []
    if localization:
        rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
        rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
    else:
        arguments.append('-d')  # Deletes previous database (~/.ros/rtabmap.db)

    # Remappings
    remappings = [('odom', 'icp_odom')]  # Uses icp_odom internally, TF handles /odom
    if imu_used:
        remappings.append(('imu', LaunchConfiguration('imu_topic')))
    else:
        remappings.append(('imu', 'imu_not_used'))  # IMU disabled

    if rgbd_image_used:
        if rgbd_cameras == 1:
            remappings.append(('rgbd_image', LaunchConfiguration('rgbd_image_topic')))
        else:
            remappings.append(('rgbd_images', LaunchConfiguration('rgbd_images_topic')))

    # Nodes
    nodes = [
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            output='screen',
            parameters=[shared_parameters, icp_odometry_parameters],
            remappings=remappings + [('scan_cloud', lidar_topic_deskewed)],
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[
                shared_parameters,
                rtabmap_parameters,
                {'subscribe_rgbd': rgbd_image_used, 'rgbd_cameras': rgbd_cameras},
            ],
            remappings=remappings + [('scan_cloud', lidar_topic_deskewed)],
            arguments=arguments,
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[shared_parameters, rtabmap_parameters],
            remappings=remappings + [('scan_cloud', 'odom_filtered_input_scan')],
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',   # true
            description='Use simulated clock.'
        ),
        DeclareLaunchArgument(
            'deskewing', default_value='false', # true
            description='Enable lidar deskewing.'
        ),
        DeclareLaunchArgument(
            'frame_id', default_value='base_footprint',
            description='Base frame of the robot.'
        ),
        DeclareLaunchArgument(
            'fixed_frame_id', default_value='',
            description='Fixed frame used for lidar deskewing. If not set, generated from IMU.'
        ),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Localization mode.'
        ),
        DeclareLaunchArgument(
            'lidar_topic', default_value='/lidar/points',
            description='Name of the lidar PointCloud2 topic.'
        ),
        DeclareLaunchArgument(
            'imu_topic', default_value='',
            description='IMU topic (ignored if empty).'
        ),
        DeclareLaunchArgument(
            'rgbd_image_topic', default_value='',
            description='RGBD image topic (ignored if empty).'
        ),
        DeclareLaunchArgument(
            'rgbd_images_topic', default_value='',
            description='RGBD images topic (ignored if empty).'
        ),
        DeclareLaunchArgument(
            'expected_update_rate', default_value='15.0',       # 15.0
            description='Expected lidar frame rate. Set slightly higher than actual (e.g., 15 Hz for 10 Hz).'
        ),
        DeclareLaunchArgument(
            'voxel_size', default_value='0.05',
            description='Voxel size (m) of downsampled lidar cloud. Adjusted for better indoor resolution.'
        ),
        DeclareLaunchArgument(
            'min_loop_closure_overlap', default_value='0.2',
            description='Minimum scan overlap percentage to accept a loop closure.'
        ),
        DeclareLaunchArgument(
            'deskewing_slerp', default_value='true',
            description='Use slerp interpolation between first and last scan timestamps.'
        ),
        DeclareLaunchArgument(
            'qos', default_value='1',
            description='QoS setting: 0=default, 1=reliable, 2=best effort.'
        ),
        OpaqueFunction(function=launch_setup),
    ])

