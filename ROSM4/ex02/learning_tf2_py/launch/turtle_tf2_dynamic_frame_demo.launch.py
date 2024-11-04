from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  radius_arg = DeclareLaunchArgument('radius', default_value='3',)
  direction_of_rotation_arg = DeclareLaunchArgument(
    'direction_of_rotation', default_value='1', 
  )

  demo_nodes = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory('learning_tf2_py'), 'launch'),
      '/turtle_tf2_demo.launch.py']),
    launch_arguments={'target_frame': 'carrot1'}.items(),
  )

  dynamic_broadcaster_node = Node(
    package='learning_tf2_py',
    executable='dynamic_frame_tf2_broadcaster',
    name='dynamic_broadcaster',
    parameters=[{
      'radius': LaunchConfiguration('radius'),
      'direction_of_rotation': LaunchConfiguration('direction_of_rotation'),
    }],
  )

  return LaunchDescription([
    radius_arg,
    direction_of_rotation_arg,
    demo_nodes,
    dynamic_broadcaster_node,
  ])

