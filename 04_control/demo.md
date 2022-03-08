

## Hardware abstraction layer 

This [part](https://github.com/ros-controls/ros2_control_demos/tree/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/include/ros2_control_demo_hardware).

- URDF description: 
	- [`rrbot.urdf.xacro`](https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_description/rrbot_description/urdf/rrbot.urdf.xacro) Just a Xacro which calls 
	- [`rrbot_description.urdf.xacro`](https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_description/rrbot_description/urdf/rrbot_description.urdf.xacro)
	- [`rrbot.materials.xacro`](https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_description/rrbot_description/gazebo/rrbot.materials.xacro)
	- [`rrbot.ros2_control.xacro`](https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_description/rrbot_description/ros2_control/rrbot.ros2_control.xacro) Defines a `RRBotSystemPositionOnlyHardware` exported as plugin [here](https://github.com/ros-controls/ros2_control_demos/blob/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/ros2_control_demo_hardware.xml#L2), [defined here](https://github.com/ros-controls/ros2_control_demos/blob/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/src/rrbot_system_position_only.cpp#L26) and [declared here](https://github.com/ros-controls/ros2_control_demos/blob/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/include/ros2_control_demo_hardware/rrbot_system_position_only.hpp#L35)



# `rrbot.launch.py`

1. loads the URDF `rrbot_description/urdf/rrbot.urdf.xacro` into the parameter `robot_description`
2. loads the parameters in `ros2_control_demo_bringup/config/rrbot_controllers.yaml`

