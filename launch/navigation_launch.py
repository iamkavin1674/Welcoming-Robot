# -*- coding: utf-8 -*-
"""
launch/navigation_launch.py -- ROS 2 launch file that brings up:

  1. slam_toolbox  (online_async_launch.py with custom params)
  2. sensor_fusion_node
  3. navigation_node
  4. recovery_node
  5. cmd_vel_mux_node
  6. interaction_camera_node  (OV2710)
  7. nav_camera_node          (FIT0701)
  8. receptionist_node

Usage
-----
    ros2 launch launch/navigation_launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # -- Paths -----------------------------------------
    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    slam_params_file = os.path.join(project_dir, "config", "slam_toolbox_params.yaml")
    nodes_dir = os.path.join(project_dir, "nodes")

    # -- Launch arguments ------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock (set true for Gazebo).",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # -- 1. SLAM Toolbox --------------------------------
    slam_toolbox_launch = None
    try:
        slam_share = get_package_share_directory("slam_toolbox")
        slam_toolbox_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "slam_params_file": slam_params_file,
                "use_sim_time": use_sim_time,
            }.items(),
        )
    except Exception:
        pass

    # -- Helper: build an ExecuteProcess for a node -----
    def _node_proc(script_name: str, name: str) -> ExecuteProcess:
        return ExecuteProcess(
            cmd=["python3", os.path.join(nodes_dir, script_name)],
            name=name,
            output="screen",
        )

    # -- 2-8. All project nodes -------------------------
    sensor_fusion_proc = _node_proc("sensor_fusion_node.py", "sensor_fusion_node")
    navigation_proc = _node_proc("navigation_node.py", "navigation_node")
    recovery_proc = _node_proc("recovery_node.py", "recovery_node")
    cmd_vel_mux_proc = _node_proc("cmd_vel_mux_node.py", "cmd_vel_mux_node")
    interaction_cam_proc = _node_proc("interaction_camera_node.py", "interaction_camera_node")
    nav_cam_proc = _node_proc("nav_camera_node.py", "nav_camera_node")
    receptionist_proc = _node_proc("receptionist_node.py", "receptionist_node")

    # -- Assemble --------------------------------------
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)

    if slam_toolbox_launch is not None:
        ld.add_action(slam_toolbox_launch)

    ld.add_action(sensor_fusion_proc)
    ld.add_action(navigation_proc)
    ld.add_action(recovery_proc)
    ld.add_action(cmd_vel_mux_proc)
    ld.add_action(interaction_cam_proc)
    ld.add_action(nav_cam_proc)
    ld.add_action(receptionist_proc)

    return ld

