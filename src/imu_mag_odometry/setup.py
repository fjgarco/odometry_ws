from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'imu_mag_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='franchu',
    maintainer_email='franchu@todo.todo',
    description='IMU and Magnetometer odometry estimation for orientation and optional position/velocity integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_mag_odometry_node = imu_mag_odometry.imu_mag_odometry_node:main',
        ],
    },
)
