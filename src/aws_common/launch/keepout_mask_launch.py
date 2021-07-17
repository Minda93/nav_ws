import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
  # Get the launch directory
  pkgsPath = FindPackageShare('aws_common')
  aws_common_dir = pkgsPath.find('aws_common')

  namespace = LaunchConfiguration('namespace')
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  map_yaml_file = LaunchConfiguration('map_mask')
  params_file = LaunchConfiguration('params_file')
  lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

  # Create our own temporary YAML files that include substitutions
  param_substitutions = {
    'use_sim_time': use_sim_time,
    'yaml_filename': map_yaml_file}

  configured_params = RewrittenYaml(
    source_file=params_file,
    root_key=namespace,
    param_rewrites=param_substitutions,
    convert_types=True)
  
  # Nodes launching commands
  lifecycle_manager_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_costmap_filters',
    namespace=namespace,
    output='screen',
    parameters=[{'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': lifecycle_nodes}])

  map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='filter_mask_server',
    namespace=namespace,
    output='screen',
    parameters=[configured_params])

  costmap_filter_info_server_node = Node(
    package='nav2_map_server',
    executable='costmap_filter_info_server',
    name='costmap_filter_info_server',
    namespace=namespace,
    output='screen',
    parameters=[configured_params])
  
  return LaunchDescription([
    # Set env var to print messages to stdout immediately
    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

    DeclareLaunchArgument(
      'namespace', default_value='',
      description='Top-level namespace'),
    
    DeclareLaunchArgument(
      'use_sim_time', default_value='false',
      description='Use simulation (Gazebo) clock if true'),
    
    DeclareLaunchArgument(
      'autostart', default_value='true',
      description='Automatically startup the nav2 stack'),
    
    DeclareLaunchArgument(
      'map_mask',
      default_value=os.path.join(aws_common_dir, 'map', 'bookstore_keepout_mask.yaml'),
      description='Full path to map yaml file to load'),
    
    DeclareLaunchArgument(
      'params_file',
      default_value=os.path.join(aws_common_dir, 'config', 'keepout_params.yaml'),
      description='Full path to the ROS2 parameters file to use'),
    
    lifecycle_manager_node,
    map_server_node,
    costmap_filter_info_server_node,
  ])