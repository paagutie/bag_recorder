# bag_recorder 
Recording ROS2 bags from a node dynamically.
## Description
With this plugin it is possible to use ROS2 bags dynamically. Just add the name, type and quality of service for each topic in the [params.yaml](config/params.yaml) file.

## Requirements
- [ROS2](https://docs.ros.org/en/galactic/Installation.html) - galactic
- [uuv_msgs](https://github.com/paagutie/uuv_msgs) (not strictly)
- [dvl_msgs](https://github.com/paagutie/dvl_msgs) (not strictly)


## Installation
- Clone the repositories and compile them:
```
$ source /opt/ros/galactic/setup.bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/paagutie/uuv_msgs.git
$ git clone https://github.com/paagutie/dvl_msgs.git
$ git clone https://github.com/paagutie/bag_recorder.git
$ cd ..
$ colcon build
```

### Usage
- Edit the [params.yaml](config/params.yaml) file or add/remove ros message structures if necessary, then start the node in c++.
```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 run bag_recorder bag_recorder_node --ros-args --params-file ~/ros2_ws/src/bag_recorder/config/params.yaml

```
The node will automatically create a subscriber of type **std_msgs/msg/Bool** with a topic named /rosbag/status. It is then possible to publish to this topic from another ROS2 node or directly from the console in order to start the rosbag.
```
$ ros2 topic pub --once /rosbag/status std_msgs/msg/Bool 'data: true'
```

### Add or remove message structures

If you want to add or remove more ROS2 message structures, just edit the file [recorder.hpp](include/bag_recorder/recorder.hpp) 

