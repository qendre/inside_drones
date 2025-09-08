from setuptools import setup
import os
from glob import glob

package_name = 'my_rtab_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Register package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install rviz configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jw',
    maintainer_email='jw@todo.todo',
    description='Bringup package for RTAB-Map experiments',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

