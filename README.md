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

# initiation_to_research folder:

ACOClass.py: Python class implementing Ants Colony Optimization algorithm

acoplanner.py: ROS2 Node sending targets computed by ACOClass one by one to the controller node

MCTSClass.py: Python class implementing a Monte Carlo Tree

MCTSPlannerClass.py: Python class implementing Monte Carlo Tree Search algorithm

mctsplanner.py: ROS2 Node sending targets computed by MCTSPLannerClass one by one to the controller node

controller.py: ROS2 Node sending heading to the car to reach target point using artificial potential field

car.py: ROS2 Node simulating a Dubin's vehicle

display.py: ROS2 Node displaying the car, the targets and the the potential field

logger.py: Launch simulations using ACO or MCTS Algorithm and log results

# launch folder:

launch_aco_simulations.launch.py : launch an ACO algorithm simulation
launch_aco_simulations.sh: script launching the ACO simulation

launch_aco_mcts.launch.py : launch a MCTS algorithm simulation
launch_aco_mcts.sh: script launching the MCTS simulation

launch_logger.launch.py: Launch alternatively an ACO or MCTS simulation by sending a scenario index to load

# data: 

generate_scenarios.py: generate scenarios to be loaded in later simulations
plot_results.py: plot results

## Package usage

