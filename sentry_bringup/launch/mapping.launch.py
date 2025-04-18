import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():

  config_path = os.path.join(
      get_package_share_directory('sentry_bringup'), 'params') 
  
  twist2chassis_cmd_node=Node(
    package='cmd_chassis',
    executable='twist2chassis_cmd',
    output='screen'
  )
  
  fake_joint_node=Node(
    package='cmd_chassis',
    executable='fake_joint',
    output='screen'
  )
  
  twist_transformer_node=Node(
    package='cmd_chassis',
    executable='twist_transformer',
    output='screen'
  )

  rot_imu=Node(
    package='cmd_chassis',
    executable='rot_imu',
    output='screen'
  )

  sentry_description = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sentry_description'), 'launch', 'view_model.launch.py')])
  )

  # mid360
  mid360_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
          get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')])
  )
  
  # fast-lio localization   
  fast_lio_param = os.path.join(
      config_path, 'fast_lio_mapping_param.yaml')
  fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
          fast_lio_param
        ],
        output='screen',
        remappings=[('/Odometry','/state_estimation')]
    )

  start_octomap_server = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sentry_bringup'), 'launch', 'octomap_server_intensity.launch.py')])
  )
        
  rviz_config_file = os.path.join(
    get_package_share_directory('sentry_bringup'), 'rviz', 'loam_livox.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file,'--ros-args', '--log-level', 'warn'],
    output='screen'
  )

  delayed_start_mapping = TimerAction(
    period=8.0,
    actions=[
      fast_lio_node,
      start_octomap_server
    ]
  )

  ld = LaunchDescription()

  ld.add_action(twist2chassis_cmd_node)
  ld.add_action(fake_joint_node)
  ld.add_action(twist_transformer_node)
  ld.add_action(rot_imu)
  ld.add_action(sentry_description)
  ld.add_action(mid360_node)
  ld.add_action(start_rviz)
  ld.add_action(delayed_start_mapping)

  return ld