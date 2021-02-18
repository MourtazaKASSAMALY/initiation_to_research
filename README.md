## Requirements

ROS Eloquent : https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/

Colcon: 

``` bash
sudo apt install python3-colcon-common-extensions
```

ROS2 packages: 

``` bash
sudo apt install ros-eloquent-rviz2 ros-eloquent-urdf ros-eloquent-xacro ros-eloquent-robot-state-publisher ros-eloquent-joint-state-publisher-gui
```

ROS2 gazebo packages: 

``` bash
sudo apt install ros-eloquent-gazebo-dev ros-eloquent-gazebo-plugins ros-eloquent-gazebo-ros ros-eloquent-rqt-robot-steering
```

Create a ROS2 Workspace : 

``` bash
$ cd ~
$ mkdir -p ws_ros2/src
$ cd ws_ros2
$ colcon build --symlink-install
$ . install/setup.bash
$ export LC_NUMERIC="en_US.UTF-8"
```

## Get the package

``` bash
$ cd ~/ws_ros2/src
$ git clone https://github.com/MourtazaKASSAMALY/initiation_to_research.git
$ cd ~/ws_ros2
$ colcon build --symlink-install
$ . install/setup.bash
```

## About the package


## Package usage

