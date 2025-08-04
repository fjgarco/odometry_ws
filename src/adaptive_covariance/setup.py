from setuptools import setup

package_name = 'adaptive_covariance'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/adaptive_covariance.launch.py']),
        ('share/' + package_name + '/config', ['config/adaptive_covariance_params.yaml']),
        ('share/' + package_name + '/config', ['config/sensor_health_thresholds.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fjgarco',
    maintainer_email='fjgarco@users.noreply.github.com',
    description='Adaptive covariance adjustment for multi-sensor fusion based on real-time sensor health monitoring',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adaptive_covariance_node = adaptive_covariance.adaptive_covariance_node:main',
            'sensor_health_monitor = adaptive_covariance.sensor_health_monitor:main',
            'ekf_covariance_adapter = adaptive_covariance.ekf_covariance_adapter:main',
            'sync_monitor = adaptive_covariance.sync_monitor:main',
        ],
    },
)
