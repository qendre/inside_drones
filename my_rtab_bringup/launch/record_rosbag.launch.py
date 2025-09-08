from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Folder for saving rosbags
    default_bag_path = os.path.expanduser('~/berisha_ws/rosbags/test_bag')

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_path',
            default_value=default_bag_path,
            description='Folder path where rosbag will be saved'
        ),

        # ros2 bag record command
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', LaunchConfiguration('bag_path'),
                '/camera/color/image_raw',
                '/camera/aligned_depth_to_color/image_raw',
                '/camera/color/camera_info',
                '/camera/imu',
                '/odom',
                '/odom_info',
                '/tf',
                '/tf_static'
            ],
            output='screen'
        )
    ])

