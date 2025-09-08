import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    default_bag_path = os.path.expanduser('~/berisha_ws/rosbags/localization_test1')

    return LaunchDescription([

        # Choose rosbag file to replay
        DeclareLaunchArgument(
            'bag_path',
            default_value=default_bag_path,
            description='Path to the rosbag to play'
        ),

        # Replay rosbag
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path')],
            output='screen'
        ),

        # Run localization launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('my_rtab_bringup'),
                'launch',
                'localization.launch.py'
            )),
            launch_arguments={
                # Use same DB as mapping
                'database_path': os.path.expanduser('~/berisha_ws/maps/rviz_map.db')
            }.items(),
        )
    ])

