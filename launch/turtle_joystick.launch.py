# ROS2 lanchfile
# 1) Remember to add '<exec_depend>....</exec_depend>' in package.xml
#    - In this case '<exec_depend>turtlesim</exec_depend>'
# 2) Remember to add '(os.path.join('share', package_name), glob('launch/*.launch.py'))' in setup.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Create node One in launch-file
    turtle_node = Node( 
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[
            {"background_b": 200},
            {"background_g": 200},
            {"background_r": 200}
        ]
    )

    # Create node Two in launch-file
    joystick_node = Node(
        package="pet_ros2_joystick_pkg",
        executable="pet_joystick_node",
        parameters=[
            {"ros_topic_twist": 'turtle1/cmd_vel'}
        ]
    )

    # Expose nodes
    ld.add_action(turtle_node)
    ld.add_action(joystick_node)

    return ld