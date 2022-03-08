

`roslaunch` is a tool for
1. launching multiple ROS nodes locally and remotely via SSH
2. setting parameters

Itincludes options to automatically respawn processes that have already died.
roslaunch takes in one or more XML configuration files (with the .launch
extension) that specify the parameters to set and nodes to launch, as well as
the machines that they should be run on.
The git repo is [here](https://github.com/ros2/launch).

Tutorial [here](https://docs.ros.org/en/galactic/Tutorials/Launch/Creating-Launch-Files.html)

Motivation [here](https://design.ros2.org/articles/roslaunch.html)

- [ros2](https://github.com/ros2/ros2cli/blob/f93e41a6ce6b968d28e5f859e12eca24cca42665/ros2cli/setup.py#L61) definiton of ros2 command line script at insalltion level
- [cli](https://github.com/ros2/ros2cli/blob/master/ros2cli/ros2cli/cli.py)
- [ros2launch](https://github.com/ros2/launch_ros/tree/galactic/ros2launch) [here](https://github.com/ros2/launch_ros.git)

- [ros2 launch](https://github.com/ros2/launch_ros/blob/galactic/ros2launch/ros2launch/command/launch.py)

- [`launch_a_launch_file`](https://github.com/ros2/launch_ros/blob/7dfb3a09c1c80979be5678394bd00aec43e25279/ros2launch/ros2launch/api/api.py#L143)

- [docu](https://roboticsbackend.com/ros2-launch-file-example/)
- [docu](https://ubuntu.com/blog/ros2-launch-required-nodes)
