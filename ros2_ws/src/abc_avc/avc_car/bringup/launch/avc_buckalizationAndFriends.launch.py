import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# includes buckalization, ekf, pose to pose with covariance, and extended kalman filter

def generate_launch_description():
    buckalization = Node(
        package='buckalization',
        executable='buckalization',
        name='buckalization'
    )
    pose_to_pwc = Node(
        package = 'pose_to_pose_with_covariance',
        executable = 'pose_to_pose_with_covariance',
        name= 'pose_to_pose_with_covariance'
    )
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('avc_car'),'launch','ekf.launch.py'
        )]), 
    )

    declared_arguments = [
        buckalization,
        pose_to_pwc,
        ekf
    ]

    return LaunchDescription(declared_arguments)