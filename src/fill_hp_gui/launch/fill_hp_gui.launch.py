from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="fill_hp_gui",
            executable="fill_hp_gui",
            name="fill_hp_gui",
            output="screen",
            emulate_tty=True,
        ),
    ])
