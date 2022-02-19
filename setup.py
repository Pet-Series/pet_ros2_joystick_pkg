from setuptools import setup

package_name = 'pet_joystick'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seniorKullken',
    maintainer_email='stefan.kull@gmail.com',
    description='My first ROS2 package...',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pet_joystick_node=pet_joystick.pet_joystick_node:main"
        ],
    },
)
