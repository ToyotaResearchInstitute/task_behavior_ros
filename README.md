# Behavior Tree ROS Integration #

[Behavior trees](https://github.com/ToyotaResearchInstitute/task_behavior_engine) provide the structure for execution of a finite set of tasks in a modular fashion.  This project provides a ROS integration to the Behavior Tree Engine.

## System Requirements ##

The following [ROS distributions](http://wiki.ros.org/Distributions) are currently supported:

* Indigo

## Building & Installation ##
We recommend using [wstool](http://wiki.ros.org/wstool) and [rosdep](http://wiki.ros.org/rosdep).

``` bash
# Install wstool and rosdep.
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep

# Createa a new workspace in 'catkin_ws'.
mkdir catkin_ws
cd catkin_ws
wstool init src

# Merge the task_behavior_ros.rosinstall file and download code dependancies.
wstool merge -t src https://raw.githubusercontent.com/toyotaresearchinstitute/task_behavior_engine/master/task_behavior_ros.rosinstall
wstool update -t src

# Install deb dependencies.
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build and install.
catkin_make --install
source install_isolated/setup.bash
```

## Running the demos ##
Now that Task Behavior Engine and Task Behavior Engine's ROS integration are insalled, we can run the examples.

```bash
# run the talker demo
rosrun task_behavior_ros talker.py
```
