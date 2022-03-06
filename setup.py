from setuptools import setup
import os
from glob import glob

package_name = 'pet_ros2_joystick_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seniorKullken',
    maintainer_email='stefan.kull@gmail.com',
    description='ROS2 Python Joystick controller for Raspberry Pi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pet_joystick_node=pet_ros2_joystick_pkg.pet_joystick_node:main"
        ],
    },
)
