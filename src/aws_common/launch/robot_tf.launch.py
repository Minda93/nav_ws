import os

import launch
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command

import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  # launch parameter
  use_sim_time = LaunchConfiguration("use_sim_time")
  use_map_2_odom = LaunchConfiguration("use_map_2_odom")
  pkgsPath = FindPackageShare([
      "aws_common", 
      "rosbot_description",])
  xacro_path = pkgsPath.find("rosbot_description")+"/urdf/rosbot2.xacro"
  # urdf = os.path.join(pkgsPath.find('rosbot_description'), 'urdf', "rosbot2.urdf")
  
  # map related
  map_2_odom = Node( 
    condition=IfCondition(use_map_2_odom),
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_map_2_odom",
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    parameters=[
        {"use_sim_time", use_sim_time},],
    output='screen',
  )

  odom_2_base_link = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_odom_2_base_link",
    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    parameters=[
        {"use_sim_time", use_sim_time},],
    output='screen',
  )

  # robot sensor related
  base_link_2_laser = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_base_link_2_laser",
    arguments=['0.02', '0', '0.058', '0', '0', '0', 'base_link', 'laser'],
    parameters=[
       {"use_sim_time", use_sim_time},],
    output='screen',
  )

  base_link_2_camera_link = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_base_link_2_camera_link",
    arguments=['-0.03', '0', '0.18', '0', '0', '0', 'base_link', 'camera_link'],
    parameters=[
       {"use_sim_time", use_sim_time},],
    output='screen',
  )

  camera_link_2_camera_rgb_frame = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_camera_link_2_camera_rgb_frame",
    arguments=['0', '0', '0', '-1.5707', '0', '-1.5707', 'camera_link', 'camera_rgb_frame'],
    parameters=[
       {"use_sim_time", use_sim_time},],
    output='screen',
  )

  camera_link_2_camera_depth_frame = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_camera_link_2_camera_depth_frame",
    arguments=['0', '0', '0', '-1.5707', '0', '-1.5707', 'camera_link', 'camera_depth_frame'],
    parameters=[
       {"use_sim_time", use_sim_time},],
    output='screen',
  )

  # imu
  base_link_2_top = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_base_link_2_top",
    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'top'],
    parameters=[
        {"use_sim_time", use_sim_time},],
    output='screen',
  )

  # range sensor
  base_link_2_range_fl = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_base_link_2_range_fl",
    arguments=['0.1', '0.05', '0.01', '0', '0', '0.13', 'base_link', 'range_fl'],
    parameters=[
      {"use_sim_time", use_sim_time},],
    output='screen',
  )

  base_link_2_range_fr = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_base_link_2_range_fr",
    arguments=['0.1', '-0.05', '0.01', '0', '0', '-0.13', 'base_link', 'range_fr'],
    parameters=[
        {"use_sim_time", use_sim_time},],
    output='screen',
  )

  base_link_2_range_rl = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_base_link_2_range_rl",
    arguments=['-0.1', '0.05', '0.01', '0', '0', '2.01', 'base_link', 'range_rl'],
    parameters=[
       {"use_sim_time", use_sim_time},],
    output='screen',
  )

  base_link_2_range_rr = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_base_link_2_range_rr",
    arguments=['-0.1', '-0.05', '0.01', '0', '0', '3.27', 'base_link', 'range_rr'],
    parameters=[
        {"use_sim_time", use_sim_time},],
    output='screen',
  )

  # robot related
  base_link_2_front_left_wheel = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_base_link_2_front_left_wheel",
    arguments=['0.05', '0.11', '0', '0', '0', '0', 'base_link', 'front_left_wheel'],
    parameters=[
        {"use_sim_time", use_sim_time},],
    output='screen',
  )

  base_link_2_front_right_wheel= Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_base_link_2_front_right_wheel",
    arguments=['0.05', '-0.11', '0', '0', '0', '0', 'base_link', 'front_right_wheel'],
    parameters=[
        {"use_sim_time", use_sim_time},],
    output='screen',
  )

  base_link_2_rear_left_wheel = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_base_link_2_rear_left_wheel",
    arguments=['-0.05', '0.11', '0', '0', '0', '0', 'base_link', 'rear_left_wheel'],
    parameters=[
        {"use_sim_time", use_sim_time},],
    output='screen',
  )

  base_link_2_rear_right_wheel = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="tf_base_link_2_rear_right_wheel",
    arguments=['-0.05', '-0.11', '0', '0', '0', '0', 'base_link', 'rear_right_wheel'],
    parameters=[
        {"use_sim_time", use_sim_time},],
    output='screen',
  )

  # robot_state model (slam not working)
  # xacro method
  robot_state = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    parameters=[{
        'use_sim_time': use_sim_time,
        'robot_description':Command(['xacro',' ', xacro_path])
    }],
    output='screen',
  )
  
  # urdf method
  # robot_state = Node(
  #   package='robot_state_publisher',
  #   executable='robot_state_publisher',
  #   name='robot_state_publisher',
  #   output='screen',
  #   parameters=[{'use_sim_time': use_sim_time}],
  #   arguments=[urdf]
  # )
  
  return LaunchDescription([
    DeclareLaunchArgument(
      name="use_sim_time",
      default_value="false",
      description="use simulation time"
    ),
    DeclareLaunchArgument(
      name="use_map_2_odom",
      default_value="false",
      description="if pub map_2_odom tf"
    ),
    # map_2_odom,
    # odom_2_base_link,
    base_link_2_laser,
    base_link_2_camera_link,
    camera_link_2_camera_rgb_frame,
    camera_link_2_camera_depth_frame,
    base_link_2_top,
    base_link_2_range_fl,
    base_link_2_range_fr,
    base_link_2_range_rl,
    base_link_2_range_rr,
    # base_link_2_front_left_wheel,
    # base_link_2_front_right_wheel,
    # base_link_2_rear_left_wheel,
    # base_link_2_rear_right_wheel,
    robot_state,
  ])

if __name__ == '__main__':
  generate_launch_description()