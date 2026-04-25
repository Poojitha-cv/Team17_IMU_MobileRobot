from setuptools import setup
from glob import glob
import os

package_name = 'imu_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@todo.todo',
    description='IMU tilt detector demo with diff-drive robot',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'tilt_detector     = imu_robot.tilt_detector:main',
            'motion_classifier = imu_robot.motion_classifier:main',
            'imu_reader        = imu_robot.imu_reader:main',
            'teleop            = imu_robot.teleop:main',
            'imu_graph         = imu_robot.imu_graph:main',   # ← ADDED
        ],
    },
)
