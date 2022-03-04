# Some definitions
- **package prefix** The new name fo the old catkin install space [see here](https://answers.ros.org/question/288501/ros2-equivalent-of-rospackagegetpath/).

# Default ROS2 package managing tool

- **`ros2 pkg executables`** List the executables inside a package
- **`ros2 pkg list`** List available packages like `rospack list`
- **`ros2 pkg xml`** print the manifest of the package.
- **`ros2 pkg create` creates a pacakge** This command generates a folder with the set of files required to make a package. By default it generates `ament-cmake` pacakges. But it can generaty the following type of packages:
    - **cmake**: generic cmake package intendet for general-purpose non-ros packages
    - **ament-cmake**: an example is [here](https://github.com/ros/ros_tutorials/tree/galactic-devel/turtlesim)
    - **ament-python**: package not based on cmake but in python setup-tools

# `colcon`
Package creation in ROS 2 uses ament as its build system and colcon as its build tool.
This means that `colcon` command line tool to build and `ament` is the cmake/python setup-tools api that allows you to write ros-packages that can be compiled with colcon.

# Overlay and underlay

Sourcing the `local_setup` of the overlay will only add the packages available in the overlay to your environment. setup sources the overlay as well as the underlay it was created in, allowing you to utilize both workspaces.

So, sourcing your main ROS 2 installation’s setup and then the `dev_ws` overlay's `local_setup`, like you just did, is the same as just sourcing `dev_ws`'s setup, because that includes the environment of the underlay it was created in.
