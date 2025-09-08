import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    parameters = [{
        'frame_id': 'camera_link',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': False,          # RGB + Depth are hardware synced
        'wait_imu_to_init': True,
        'database_path': LaunchConfiguration('database_path')
    }]

    remappings = [
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw')
    ]

    return LaunchDescription([

        # Choose unite_imu_method
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0=none, 1=copy, 2=linear_interpolation.'
        ),

        # Choose database path
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.expanduser('~/berisha_ws/maps/default_map.db'), #change path if you want to load a specific map
            description='Path to existing RTAB-Map database for localization'
        ),

        # Ensure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # 1. Camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
            launch_arguments={
                'camera_namespace': '',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                'align_depth.enable': 'true',
                'enable_sync': 'true',
                'rgb_camera.profile': '640x360x30'
            }.items(),
        ),

        # 2. RGB-D odometry
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=parameters,
            remappings=remappings
        ),

        # 3. RTAB-Map SLAM (mapping mode)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['--rtabmao-args', '--Mem/IncrementalMemory false']  #remove if you dont want to delete memory
        ),

        # 4. RTAB-Map visualization (graph, features, loop closures)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings
        ),
        
        # RViz for clean visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('my_rtab_bringup'),
                'rviz',
                'localization_view.rviz')],
            output='screen'
        ),


        # 5. IMU filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False
            }],
            remappings=[('imu/data_raw', '/camera/imu')]
        ),
    ])

