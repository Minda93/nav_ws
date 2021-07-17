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
  use_sim_time = LaunchConfiguration("use_sim_time")
  rviz_file = LaunchConfiguration("rviz_file")
  pkgsPath = FindPackageShare(["aws_common"])
  
  # Node
  rviz2_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    parameters=[
        {"use_sim_time", use_sim_time}],
    arguments=['-d', rviz_file],
    output="screen"
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      name="use_sim_time",
      default_value="false",
      description="use simulation time"
    ),
    DeclareLaunchArgument(
      name="rviz_file",
      default_value= os.path.join(pkgsPath.find("aws_common"), 
          "rviz","rosbot.rviz"),
      description="Full path to the ROS2 parameters file to use for the rviz node"
    ),
    rviz2_node,
  ])

if __name__ == '__main__':
  generate_launch_description()