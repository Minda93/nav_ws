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
      "aws_robomaker_bookstore_world",
      "aws_common",
      "gazebo_ros",
      "rosbot_description",])
  gazebo_ros = pkgsPath.find("gazebo_ros")
  aws_common = pkgsPath.find("aws_common")

  # world
  world_file_name =  'bookstore.world'
  world = os.path.join(
    pkgsPath.find("aws_robomaker_bookstore_world"), 'worlds', world_file_name)

  # spawn robot (insert model to gazebo)
  # xacro_path = pkgsPath.find("rosbot_description")+"/urdf/rosbot2.xacro"
  
  spawn_robot = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    name="spawn_entity",
    arguments=[
      '-spawn_service_timeout', '60',
      '-entity', 'rosbot', 
      '-x', '0', 
      '-y', '0', 
      '-z', '0.03',
      '-file', pkgsPath.find("rosbot_description") + '/models/rosbot.sdf'],
      # '-topic', "/robot_description"],
    parameters=[
        {"use_sim_time": use_sim_time,},],
    output='screen',
  )

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

  rosbot_tf = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
      os.path.join(aws_common, 'launch', 'robot_tf.launch.py')),
    launch_arguments={'use_sim_time': use_sim_time, 'use_map_2_odom' : 'true'}.items(),
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
    spawn_robot,
    rosbot_tf,
  ])

if __name__ == '__main__':
    generate_launch_description()
