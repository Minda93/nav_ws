import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  # launch parameter
  use_sim_time = LaunchConfiguration("use_sim_time", default='true')
  slam_params_file = LaunchConfiguration("slam_params_file")
  pkgsPath = FindPackageShare(["aws_common"])
  
  # Node
  slam_toolbox_node = Node(
    parameters=[
        slam_params_file,
        {"use_sim_time", use_sim_time}],
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
      name="slam_params_file",
      default_value= os.path.join(pkgsPath.find("aws_common"), 
          "config","slam_toolbox_sim.yaml"),
      description="Full path to the ROS2 parameters file to use for the slam_toolbox node"
    ),
    slam_toolbox_node,
  ])

if __name__ == '__main__':
  generate_launch_description()