import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  # launch parameter  
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  pkgsPath = FindPackageShare([
      "person_models",
      "aws_common",
      "gazebo_ros",
      "rosbot_description",])
  gazebo_ros = pkgsPath.find("gazebo_ros")
  aws_common = pkgsPath.find("aws_common")

  # world
  world_file_name =  'person_empty.world'
  world = os.path.join(
    pkgsPath.find("person_models"), 'worlds', world_file_name)

  # gazebo
  gazebo_client = launch.actions.IncludeLaunchDescription(
  launch.launch_description_sources.PythonLaunchDescriptionSource(
    os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
      condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
  )
  
  gazebo_server = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
      os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'world',
      default_value=[world, ''],
      description='SDF world file'),
    DeclareLaunchArgument(
      name='gui',
      default_value='true'
    ),
    DeclareLaunchArgument(
      name='use_sim_time',
      default_value= "true"
    ),
    DeclareLaunchArgument('verbose', default_value='true',
      description='Set "true" to increase messages written to terminal.'),
    DeclareLaunchArgument('gdb', default_value='false',
      description='Set "true" to run gzserver with gdb'),
    DeclareLaunchArgument('state', default_value='true',
      description='Set "false" not to load "libgazebo_ros_state.so"'),
    gazebo_server,
    gazebo_client,
  ])

if __name__ == '__main__':
    generate_launch_description()
