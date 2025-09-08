from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )
    
    rviz_config = os.path.join(
    get_package_share_directory('my_rtab_bringup'),
    'rviz',
    'camera_view.rviz'
)


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'align_depth.enable': 'true',
                'enable_sync': 'true',
                'unite_imu_method': '2',
                'rgb_camera.profile': '640x480x30',
            }.items()
        ),
        Node(
    		package='rviz2',
    		executable='rviz2',
    		name='rviz2',
    		arguments=['-d', rviz_config],
    		output='screen'
		)

    ])

