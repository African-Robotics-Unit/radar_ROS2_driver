from launch import LaunchDescription
from launch_ros.actions import Node

package_name = 'radar_dca1000_py_pkg'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package=package_name,
            namespace='dca1000',
            executable='radar_control',
            name='control'
        ),
        Node(
            package=package_name,
            namespace='dca1000',
            executable='radar_data',
            name='data',
        )
    ])