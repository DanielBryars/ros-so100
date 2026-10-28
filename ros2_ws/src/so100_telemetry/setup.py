import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'so100_telemetry'

setup(
    name=package_name,
    version='0.0.1',
    # find_packages picks up both so100_telemetry and the vendored
    # so100_telemetry.scservo_sdk subpackage (each has an __init__.py).
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files so `ros2 launch so100_telemetry ...` can find them.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danb',
    maintainer_email='daniel@bryars.com',
    description='Publish SO100 (Feetech STS3250) joint positions as sensor_msgs/JointState.',
    license='MIT',
    entry_points={
        # This is what makes `ros2 run so100_telemetry joint_state_publisher` work:
        # it maps a command name to the main() function in our module.
        'console_scripts': [
            'joint_state_publisher = so100_telemetry.joint_state_publisher:main',
        ],
    },
)
