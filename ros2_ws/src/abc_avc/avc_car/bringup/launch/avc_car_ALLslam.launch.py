import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (EmitEvent, LogInfo,
                            RegisterEventHandler, DeclareLaunchArgument, GroupAction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, PythonExpression, AndSubstitution,
                                  NotSubstitution)
from launch_ros.actions import LoadComposableNodes
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from lifecycle_msgs.msg import Transition
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import LifecycleNode
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name='avc_car'

    # finicky so just alunching separately so it can get reset seperately too
    # roscontrolrviz = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory(package_name),'launch','avc_car.testlaunch.py'
    #     )]), 
    # )

    # navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory(package_name),'launch','avc_navigation.launch.py'
    #     )]), 
    # )
    # start_nav_after_slam_activates = RegisterEventHandler(
    #     OnStateTransition(
    #         target_lifecycle_node=start_async_slam_toolbox_node,
    #         goal_state='active',
    #         entities=[
    #             LogInfo(msg="async_slam_toolbox_node is active: starting nav2 stack"),
    #             navigation
    #         ]
    #     ),
    # )


    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sllidar_ros2'),'launch','sllidar_c1_launch.py'
        )]), 
    )
    
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory(package_name), 'config', 'mappertest.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # Perform substitution `$find-pkg-share`
    slam_params_file_w_subst = ParameterFile(
        slam_params_file,
        allow_substs=True,
    )
    
    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
          slam_params_file_w_subst,
          {
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(
            get_package_share_directory(package_name),'config','mappertest.yaml')
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    declared_arguments = [
        lidar,
        declare_autostart_cmd, declare_use_lifecycle_manager, declare_use_sim_time_argument, declare_slam_params_file_cmd, start_async_slam_toolbox_node, configure_event, activate_event
        #, navigation, start_nav_after_slam_activates
    ] # just do nav by itself. It creates too many controller servers otherwise. ts pmo. - Annabeth
        #navigation,

    return LaunchDescription(declared_arguments)
