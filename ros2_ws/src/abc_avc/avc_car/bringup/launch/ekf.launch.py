# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # creating the path to find the ekf.yaml files (robot_localization -> config -> ekf.yaml)
    robot_localization_dir = get_package_share_directory('avc_car') 
    parameters_file_dir = os.path.join(robot_localization_dir, 'config') 
    parameters_file_path = os.path.join(parameters_file_dir, 'ekf_test.yaml')

    os.environ['FILE_PATH'] = str(parameters_file_dir)
    return LaunchDescription([
    launch_ros.actions.Node(
           package='robot_localization', # declaring that we want to use the prebuilt robot_localization package
           executable='ekf_node', # extended kalman filter
           name='ekf_filter_node_local', # differentiates between the headings in ekf.yaml
	       output='screen', # all output (e.g. print, log) should be sent to the terminal screen where the launch file was started
           parameters=[parameters_file_path],
           remappings=[('odometry/filtered', 'odometry/local')] # instead of both ekf_local and ekf_global publishing to the same topic, 
                                                                # now they're remapped to publish to different ones
           ),
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_global',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odometry/global')] 
           )    
])