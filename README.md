<!-- TOC -->

- [nav_ws](#nav_ws)
- [1. Environment](#1-environment)
- [2. Setup Env](#2-setup-env)
- [3. Flow](#3-flow)
  - [3.1 slam_toolbox -> map](#31-slam_toolbox---map)
  - [3.2 nav2](#32-nav2)
- [4. Base Demo](#4-base-demo)
  - [4.1 SLAM demo](#41-slam-demo)
  - [4.2 Localization and Navigation demo](#42-localization-and-navigation-demo)
  - [4.3 Keepout region demo](#43-keepout-region-demo)
- [5. Tools](#5-tools)
- [Bug](#bug)
  - [1. clang compiler : (-Wall -Wextra -Wpedantic)](#1-clang-compiler---wall--wextra--wpedantic)
  - [2. rosdep install error](#2-rosdep-install-error)
  - [3. robot_state_publisher is incorrectly working](#3-robot_state_publisher-is-incorrectly-working)
  - [4. Some plugins of gazebo_ros_pkgs is not support for ros2](#4-some-plugins-of-gazebo_ros_pkgs-is-not-support-for-ros2)
- [TODO](#todo)
- [Reference](#reference)

<!-- /TOC -->

# nav_ws
ros2 practice 3

# 1. Environment

| names                         | version             |
| ---                           | ---                 |
| gcc                           | 9.3.0               |
| ~~clang~~                     | ~~10.0.0-4ubuntu1~~ |
| CMake                         | 3.18.4              |
| ros2                          | galactic (release20210716)        |
| gazebo                        | 11.9.1              |
| gazebo_ros_pkgs               | galactic <br> (205755685f77300a27d16c9739b1e397e30811d9)         |
| aws-robomaker-bookstore-world | ros2                |
| rosbot_description            | foxy                |
| navigation2                   | galactic <br> Galactic sync 5 (4a6878c2f8fb95e5bc3be8e6f0b531d17d9bebb2) |
|  BehaviorTree.CPP             | master <br>(0bff68a051a05e0b60a6908750d91ce6079c8685) |
| Groot | master <br> (05fe640172e3cd447ab5db31f71355789f6a48b3) |

# 2. Setup Env

```bash
  # download navigation2
  $ cd <ws>/src
  $ git clone https://github.com/ros-planning/navigation2.git -b galactic
  
  # build
  $ cd <ws>
  $ rosdep install -y -r -q --from-paths src --ignore-src --rosdistro galactic
  $ colcon build --symlink-install
```

# 3. Flow 
## 3.1 slam_toolbox -> map
## 3.2 nav2

# 4. Base Demo
## 4.1 SLAM demo
```bash
  # run gazebo and tf 
  $ ros2 launch aws_common nav_bookstore.launch.py

  # run slam_toolbox
  $ ros2 launch aws_common slam_toolbox_sim.launch.py

  # run rviz2
  $ ros2 launch aws_common rosbot_rviz.launch.py use_sim_time:=true

  # manual control
  $ ros2 run aws_teleop teleop_keyboard
```

## 4.2 Localization and Navigation demo
* behavior tree
  * navigate_to_pose_w_replanning_and_recovery.xml
  
```bash
  # run gazebo and tf 
  $ ros2 launch aws_common nav_bookstore.launch.py

  # run rviz2
  $ ros2 launch aws_common rosbot_rviz.launch.py use_sim_time:=true

  # run map_server and amcl
  $ ros2 launch aws_common localization_launch.py use_sim_time:=true
  
  # set init_pose
  # method 1: use rviz 2D pose estimate button
  # method 2: use command -> pose(0, 0, 0) orientation(0, 0, 0, 1) 
  $ ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 0, y: 0}, orientation: {w: 1}}}}" --once

  # run nav2
  $ ros2 launch aws_common navigation_launch.py use_sim_time:=true

  # check behavior tree
  ## 1. click monitor -> click start
  ## 2. default localhost and port 
  ## 3. click connect 
  $ ros2 run groot Groot
```

## 4.3 Keepout region demo
```bash
  # run gazebo and tf 
  $ ros2 launch aws_common nav_bookstore.launch.py

  # run rviz2
  $ ros2 launch aws_common rosbot_rviz.launch.py use_sim_time:=true

  # run map_server and amcl
  $ ros2 launch aws_common localization_launch.py use_sim_time:=true params_file:=<path_of_nav2_keepout_filter_params>
  
  # keepout mask
  # run filter_mask_server(another map_server) and costmap_filter_info_server
  $ ros2 launch aws_common keepout_mask_launch.py use_sim_time:=true
  
  # set init_pose
  # method 1: use rviz 2D pose estimate button
  # method 2: use command -> pose(0, 0, 0) orientation(0, 0, 0, 1) 
  $ ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 0, y: 0}, orientation: {w: 1}}}}" --once

  # run nav2
  $ ros2 launch aws_common navigation_launch.py use_sim_time:=true params_file:=<path_of_nav2_keepout_filter_params>
```

# 5. Tools
* check tf tree to pdf file
  ```bash
    # default frames.gv and frames.pdf
    $ ros2 run tf2_tools view_frames
  ```
* check urdf format
  ```bash
    $ check_urdf <path_of_urdf_file>

    # success output
    # robot name
    # ---------- Successfully Parsed XML ---------------
    # robot link
  ```
* check or edit behavior tree
  ```bash
    $ ros2 run groot Groot
  ```
    
# Bug 
## 1. clang compiler : (-Wall -Wextra -Wpedantic)
  * nav2_behavior_tree : BT::Tree getTree() const error
    ```c++
      // BehaviorTree_cpp_v3 cannot use copy constructor
      
      // non-copyable. Only movable
      Tree(const Tree& ) = delete;
    ```
      
  * nav2_system_tests/nav2_behavior_tree 
    ```text
      memory library error
    ```
    
  * some functions do not use override
  
  * __ros2 galactic__ change in rclcpp’s logging macros but galactic branch __does not modified__
    ```c++
      // const char *my_const_char_string format = "Foo";
      // RCLCPP_DEBUG(get_logger(), my_const_char_string);

      const char *my_const_char_string format = "Foo";
      RCLCPP_DEBUG(get_logger(), "%s", my_const_char_string);
    ```
  * ros/geometry2 plugin for __ros2 branch__ add .hpp file, but galactic branch do not.
    ```bash
      # use navigation2 for main branch
      # have to update ros/geometry2 to ros2 version
    ```

## 2. rosdep install error
* aws_common: Cannot locate rosdep definition for [rviz]
  * reason
    * maybe source environment is not complete.
  * solution
    * Temporarily use COLCON_IGNORE file ignore aws_common packages
    
## 3. robot_state_publisher is incorrectly working
* output `/tf` is not correct.
  * robot model and tf is not stable.
  * reason
    * Different from `sdf model` and `xacro model`
    * tf_tree connection is incorrect.
  * solution
    * Temporarily use custom tf tree -> `robot_tf.launch.py`
      * But, there is not robot model in rviz.
    * ~~custom odom transform~~

## 4. Some plugins of gazebo_ros_pkgs is not support for ros2
* xacro model is too old version to use.
  * reason
    * plugins too old
      * gazebo_ros_skid_steer_drive
      * gazebo_ros_openni_kinect
      * gazebo_ros_laser
      * gazebo_ros_range
      * gazebo_ros_control
  * solution
    * use sdf model to build xacro model (rosbot2.xacro)
      * change tf tree
      * change plugin 
        * gazebo_ros_skid_steer_drive -> gazebo_ros_diff_drive
        * gazebo_ros_openni_kinect -> gazebo_ros_depth_camera
        * gazebo_ros_laser -> gazebo_ray_sensor
        * gazebo_ros_range -> gazebo_ray_sensor
        * gazebo_ros_control -> x

# TODO
* [x] 1. fix xacro model bug
  * [x] rviz
  * [ ] gazebo -> plugin cannot use, not found mesh file and model frame error
* [x] ~~2. set rviz file and launch rviz~~
  * [x] ~~rviz file~~
  * [x] ~~launch~~
* [ ] 3. test `slam_toolbox` parameters
* [x] 4. test `nav2`
  * [x] amcl
  * [x] map_server
  * [x] nav2_bt_navigator
* [x] 5. Add Application
  * [ ] Inspection task
  * [ ] Dynamic Object Following

# Reference 
* slam_toolbox
  * [slam_toolbox github](https://github.com/SteveMacenski/slam_toolbox)
  * [ROS2 Navigation2 \~slam-toolboxでSLAMする\~](https://qiita.com/porizou1/items/152ad3829e84a9ba0355)
* navigation2
  * [nav2 index](https://navigation.ros.org/index.html)
  * [navigation2](https://github.com/ros-planning/navigation2)
* turtlebot3
  * [turtlebot3 github](https://github.com/ROBOTIS-GIT/turtlebot3)
* gazebo
  * [How to Simulate a Robot Using Gazebo and ROS 2](https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/)
  * [ROS講座40 車輪ロボットを作る4(gazeboの位置をrvizに取り出す)](https://qiita.com/srs/items/5848c6b05e5f8a0827f9)
  * [ROS 2 Migration: Spawn and delete](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Spawn-and-delete)
* rviz
  * [ROS2:簡単な箱の表示 rviz ](https://memodays.jp/400/)
  * [Using URDF with robot_state_publisher](https://docs.ros.org/en/galactic/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher.html)
  * [robot_state_publisher + xacro + Python launch](https://answers.ros.org/question/361623/ros2-robot_state_publisher-xacro-python-launch/)
* behavior tree
  * [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
  * [Groot](https://github.com/BehaviorTree/Groot)
* issue
  * [gazebo_ros_pkgs #512](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/512)