import launch
import launch_ros.actions

from m2s2_pydevclass.m2s2_device import M2S2Device

device = M2S2Device("RADAR")

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='radar_dca1000_py_pkg',
            executable='radar_ctrl',
            name=("%s_node" % device.devNAME)),

        launch_ros.actions.Node(
            package='radar_gui_py_pkg',
            executable='radar_client',
            name=("%s_client_node" % device.devNAME)),

        launch_ros.actions.Node(
            package='radar_dca1000_cpp_pkg',
            executable='radar_data',
            name=("%s_data_node" % device.devNAME)),

        launch_ros.actions.Node(
            package='radar_recorders_py_pkg',
            executable='radar_hdf5',
            name=("%s_hdf5_node" % device.devNAME)),
  ])