import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml

def generate_launch_description():
  # launch parameter
  use_sim_time = LaunchConfiguration("use_sim_time", default='true')
  namespace = LaunchConfiguration('namespace')
  slam_params_file = LaunchConfiguration("slam_params_file")

  pkgsPath = FindPackageShare(["aws_common"])
  aws_common_dir = pkgsPath.find('aws_common')


  # Create our own temporary YAML files that include substitutions
  param_substitutions = {
      'use_sim_time': use_sim_time}

  configured_params = RewrittenYaml(
      source_file=slam_params_file,
      root_key=namespace,
      param_rewrites=param_substitutions,
      convert_types=True)
  
  # Node
  slam_toolbox_node = Node(
    parameters=[
        configured_params],
    package="slam_toolbox",
    executable="sync_slam_toolbox_node",
    name="slam_toolbox",
    output="screen"
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      name="use_sim_time",
      default_value="true",
      description="use simulation time"
    ),
    DeclareLaunchArgument(
      'namespace', default_value='',
      description='Top-level namespace'
    ),
    DeclareLaunchArgument(
      name="slam_params_file",
      default_value= os.path.join(pkgsPath.find("aws_common"), 
          "config","rosbot_slam_bookstore_sim.yaml"),
      description="Full path to the ROS2 parameters file to use for the slam_toolbox node"
    ),

    slam_toolbox_node,
  ])

if __name__ == '__main__':
  generate_launch_description()