from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'unity'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jared',
    maintainer_email='jared@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "move_publisher = unity.move_publisher:main",
            "drive_publisher = unity.drive_publisher:main",
            "mask_subscriber = unity.mask_subscriber:main",
            "image_subscriber = unity.image_subscriber:main",
            "imu_odometry = unity.imu_odometry:main",
            "pointcloud_to_laser = unity.pointcloud_to_laser:main",
            "differential_odom = unity.differential_odom:main",
            "camera_to_laser = unity.camera_to_laser:main",
        ],
    },
)
