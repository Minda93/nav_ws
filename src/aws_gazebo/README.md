
<!-- TOC -->

- [AWS GAZEBO](#aws-gazebo)
- [1. Environment](#1-environment)
- [2. Setup Env](#2-setup-env)
- [3. Test Model on gazebo](#3-test-model-on-gazebo)
- [Bug](#bug)
- [Reference](#reference)

<!-- /TOC -->

# AWS GAZEBO

# 1. Environment

| names                         | version           |
| ---                           | ---               |
| aws-robomaker-bookstore-world | ros2              |
| rosbot_description            | foxy              |

# 2. Setup Env
  ```bash
    $ cd <ws>/src/aws_gazebo

    # download packages
    $ git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-bookstore-world.git
    $ git clone -b foxy https://github.com/husarion/rosbot_description.git

    
    $ cd <ws>

    # download dependencies
    # $ sudo apt update
    # $ rosdep update
    # $ rosdep install -i -r -y --from-paths .

    $ colcon build --symlink-install --packages-select aws-robomaker-bookstore-world rosbot_description 
  ```

# 3. Test Model on gazebo
  ```bash
    # source ros and ws env
    $ . /opt/ros2_ws/install/setup.bash
    $ . install/local_setup.bash
    
    # source gazebo env
    $ . /usr/share/gazebo/setup.sh
    # or
    $ . /usr/share/gazebo-<version>/setup.sh

    # run 
    $ ros2 launch aws_robomaker_bookstore_world bookstore.launch.py gui:=true
    $ ros2 launch rosbot_description rosbot_sim.launch.py
  ```

# Bug
* aws_robotmaker_bookstore_world
  * 1. build warning
    ```bash
      # error
      find_packages(gazebo_ros) does not match.

      # solution 
      # maybe use gazebo_ros_pkgs

      # CMakeLists.txt
      find_packages(gazebo_ros_pkgs)
      ament_export_dependencies(gazebo_ros_pkgs)
    ```
  * 2. run gazebo with ros error
    * reference 
      * [shared_ptr assertion error](https://answers.ros.org/question/358847/cannot-launch-gzclient-on-a-launch-file-results-in-shared_ptr-assertion-error/ )
    ```bash
      # error
      # gzclient boost error
      # gazebo pkgs not found

      # solution
      # source gazebo env
    ```

# Reference
* [aws-robomaker-bookstore-world](https://github.com/aws-robotics/aws-robomaker-bookstore-world)
* [rosbot_description](https://github.com/husarion/rosbot_description)