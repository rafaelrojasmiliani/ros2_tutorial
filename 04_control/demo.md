

## Hardware abstraction layer

This [part](https://github.com/ros-controls/ros2_control_demos/tree/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/include/ros2_control_demo_hardware).

- URDF description:
	- [`rrbot.urdf.xacro`](https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_description/rrbot_description/urdf/rrbot.urdf.xacro) Just a Xacro which calls
	- [`rrbot_description.urdf.xacro`](https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_description/rrbot_description/urdf/rrbot_description.urdf.xacro)
	- [`rrbot.materials.xacro`](https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_description/rrbot_description/gazebo/rrbot.materials.xacro)
	- [`rrbot.ros2_control.xacro`](https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_description/rrbot_description/ros2_control/rrbot.ros2_control.xacro) Defines a `RRBotSystemPositionOnlyHardware` exported as plugin [here](https://github.com/ros-controls/ros2_control_demos/blob/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/ros2_control_demo_hardware.xml#L2), [defined here](https://github.com/ros-controls/ros2_control_demos/blob/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/src/rrbot_system_position_only.cpp#L26) and [declared here](https://github.com/ros-controls/ros2_control_demos/blob/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/include/ros2_control_demo_hardware/rrbot_system_position_only.hpp#L35)



# `rrbot.launch.py`

1. loads the URDF `rrbot_description/urdf/rrbot.urdf.xacro` into the parameter `robot_description` [here](https://github.com/ros-controls/ros2_control_demos/blob/07e8d416624215602d58bbad9064d761cc145d09/ros2_control_demo_bringup/launch/rrbot.launch.py#L27)
2. loads the parameters in `ros2_control_demo_bringup/config/rrbot_controllers.yaml` [here](https://github.com/ros-controls/ros2_control_demos/blob/07e8d416624215602d58bbad9064d761cc145d09/ros2_control_demo_bringup/launch/rrbot.launch.py#L40)

3. Loads the node `controller_manager ros2_control_node`. [here](https://github.com/ros-controls/ros2_control_demos/blob/07e8d416624215602d58bbad9064d761cc145d09/ros2_control_demo_bringup/launch/rrbot.launch.py#L53)

4. Initlialies the joint state broadcaster [here](https://github.com/ros-controls/ros2_control_demos/blob/07e8d416624215602d58bbad9064d761cc145d09/ros2_control_demo_bringup/launch/rrbot.launch.py#L82)

5. Launch the forward position controller [here](https://github.com/ros-controls/ros2_control_demos/blob/07e8d416624215602d58bbad9064d761cc145d09/ros2_control_demo_bringup/launch/rrbot.launch.py#L88)

6. Forces rviz to run after the joint broadcaster [here](https://github.com/ros-controls/ros2_control_demos/blob/07e8d416624215602d58bbad9064d761cc145d09/ros2_control_demo_bringup/launch/rrbot.launch.py#L95)

7. Forces the forward control to start after the joint broadcaster [here](https://github.com/ros-controls/ros2_control_demos/blob/07e8d416624215602d58bbad9064d761cc145d09/ros2_control_demo_bringup/launch/rrbot.launch.py#L103)

The ros control node instantiates a controller manager [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/src/ros2_control_node.cpp#L34) and performs the control loop [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/src/ros2_control_node.cpp#L50).

The controller manager inherits from `rclcpp::Node`, is constructed with an executer [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/src/controller_manager.cpp#L56) and init services [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/src/controller_manager.cpp#L98)


The control spawner is implemented [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/scripts/spawner.py).
Its python module is [here](https://github.com/ros-controls/ros2_control/tree/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/controller_manager)

loads teh controller [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/scripts/spawner.py#L129) goto [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/controller_manager/controller_manager_services.py#L66) and ends up [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/controller_manager/controller_manager_services.py#L22).

The load controller message is [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager_msgs/srv/LoadController.srv)

The control managere instantiates the load controller servics [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/src/controller_manager.cpp#L120) and bind it to [this method](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/src/controller_manager.cpp#L934) which at the end calls [load controller](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/src/controller_manager.cpp#L157) which calls [add controller impl](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/src/controller_manager.cpp#L611).

The control manager uses [control spec](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/include/controller_manager/controller_spec.hpp#L36) to handle controller intrafes [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_manager/include/controller_manager/controller_spec.hpp#L39).


##  Modular Actuator

1. declare argumetns with default value and description
    - prefix
    - slowdown
    - `robot_controller`
2. launch `rrbot_base.launch.py` with
    - controller files `rrbot_modular_actuators.yaml`
    - `description_file` as `rrbot_modular_actuators.urdf.xacro`
    - forwards prefix, slowdown, `robot_controller`

## `rrbot_base.launch.py`

1. Declare arguments
| Paramter | Default value |
| ------- | -------------  |
|`runtime_config_package` `ros2_control_demo_bringup` |
| `controllers_file` | `rrbot_controllers.yaml` |
| `description_package` | `rrbot_description` |
| `description_file`  | No |
| `prefix`  | `""` |
| `use_sim` | `false` |
| `use_fake_hardware` | `true` |
| `fake_sensor_commands`  | `false` |
| `slowdown` |  `3.0` |
| `robot_controller` | `forward_position_controller` |
| `start_rviz` | `true` |

2. Load the contend to the description file
```python

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            "... ",
            "prefix:=",prefix," ","use_sim:=",use_sim," ","use_fake_hardware:=",
            use_fake_hardware," ","fake_sensor_commands:=",fake_sensor_commands,
            " ","slowdown:=",slowdown,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
```
3. Load the content of the controller configuration
```python
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            controllers_file,
        ]
    )
```
4.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "rrbot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "-c", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
