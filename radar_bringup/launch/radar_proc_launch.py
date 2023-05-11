import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from m2s2_pydevclass.m2s2_device import M2S2Device

device = M2S2Device("RADAR")

def generate_launch_description():

    config = os.path.join(
      get_package_share_directory('radar_bringup'),
      'config',
      'radar_config.yaml'
      )
    
    return LaunchDescription([
        Node(
            package='radar_dca1000_py_pkg',
            executable='radar_ctrl',
            name=("%s_node" % device.devNAME),
            parameters = [config]),

        Node(
            package='radar_dca1000_cpp_pkg',
            executable='radar_data',
            name=("%s_data_node" % device.devNAME),
            parameters = [config]),

        Node(
            package='radar_gui_py_pkg',
            executable='radar_proc',
            name=("%s_proc_node" % device.devNAME),
            parameters = [config]),

        Node(
            package='radar_gui_py_pkg',
            executable='radar_ctrl_gui',
            name=("%s_client_node" % device.devNAME),
            parameters = [config]),
  ])