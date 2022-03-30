

[documentation](http://control.ros.org/). and see [here](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md).

## Visibility control
[read here](https://gcc.gnu.org/wiki/Visibility).
## Control Managers

- State [declared here](https://github.com/ros2/rclcpp/blob/54c2a8ac5bf14b9353765e94db5042630b710a75/rclcpp_lifecycle/include/rclcpp_lifecycle/state.hpp#L34)

- [`control_manager.hpp`](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/include/controller_manager/controller_manager.hpp)

- [`ControlManager` definition](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/controller_manager.cpp)

- [`ResoureceStorate`](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/src/resource_manager.cpp#L38)

- **Concetp of Handle**
    - [**`StateInterface`**](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/handle.hpp#L107)

- **`Actuator`** [declared here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/actuator.hpp#L32)
    - **`ActuatorInterface`** [declared here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/actuator_interface.hpp#L66)

- **`System`** [declared here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/system.hpp#L33)
    - **`SytemInterface`** [declared here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/system_interface.hpp#L67)

- **`Sensor`** [declared here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/sensor.hpp#L33)
    - **`SensorInterface`** [declared here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/sensor_interface.hpp#L66)

- **`return_type`** [declared here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/types/hardware_interface_return_values.hpp#L22)

- [Control Manager node](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp). Executes a [read](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/controller_manager.cpp#L1340), [update](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/controller_manager.cpp#L1342), [write](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/controller_manager.cpp#L1391) loop [here](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/ros2_control_node.cpp#L50)
    - [read](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/controller_manager.cpp#L1340), invoques a [`hardware_interface::ResoureceManager`](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/hardware_interface/include/hardware_interface/resource_manager.hpp#L37) method's [read](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/hardware_interface/src/resource_manager.cpp#L891). This method [loops](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/hardware_interface/src/resource_manager.cpp#L891) separatly through `hardware_interface::ResoureceManager::resource_storage_->{actuators_, sensors_, systems_}[i].read()`. Defitions of `actuators_`, `sensors_` and `systems_` is [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/src/resource_manager.cpp#L171)


## Controller interface

- [`control interface`](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/controller_interface/include/controller_interface/controller_interface.hpp#L68)

Specific contollers are in [`ros2_controllers`](https://github.com/ros-controls/ros2_controllers)

### Forward controllers
[documentation](https://github.com/ros-controls/ros2_controllers/blob/master/forward_command_controller/doc/userdoc.rst)

Defined [here](https://github.com/ros-controls/ros2_controllers/blob/c51ba1e45ff3868f7d50b164e3ed8162db4440f1/forward_command_controller/include/forward_command_controller/forward_command_controller.hpp#L47) [defined here](https://github.com/ros-controls/ros2_controllers/blob/c51ba1e45ff3868f7d50b164e3ed8162db4440f1/forward_command_controller/src/forward_command_controller.cpp#L33).

# ActuatorInterface

Example [declared here](https://github.com/ros-controls/ros2_control_demos/blob/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/include/ros2_control_demo_hardware/rrbot_actuator.hpp#L36)  and [defined here](https://github.com/ros-controls/ros2_control_demos/blob/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/src/rrbot_actuator.cpp#L33)


| method | type | use |
| ------ | ---- | --- |
| Copy constructure | deleted | |
| Move constructure | defauled | |
| `on_init` | |  Initialization of the hardware interface from data parsed from the robot's URDF. |
| `export_state_interfaces` | pure virtual | Exports all state interfaces for this hardware interface. returns `std::vector`  of [stateInterface](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/handle.hpp#L107) |
| `export_command_interfaces` | pure virtual | xports all command interfaces for this hardware interface. returns `std::vector` of [commandInterface](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/handle.hpp#L117) |
| `prepare_command_mode_switch` | | Prepare for any mode-switching required by the new command interface combination. |
| `perform_command_mode_switch` | | Perform switching to the new command interface. |
| `read`  | pure virtual | |
| `write` | pure virtual | |
| `get_name ` | | |
| `get_state` | | |
| `set_state` | | |

# Sensor Interface

Example [declared here ](https://github.com/ros-controls/ros2_control_demos/blob/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/include/ros2_control_demo_hardware/external_rrbot_force_torque_sensor.hpp#L37) and [defined here](https://github.com/ros-controls/ros2_control_demos/blob/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/src/external_rrbot_force_torque_sensor.cpp#L32)

| method | type | use |
| ------ | ---- | --- |
| Copy constructure | deleted | |
| Move constructure | defauled | |
| Destrutroe | vritual defauled | |
| `on_init` | |  Initialization of the hardware interface from data parsed from the robot's URDF. |
| `export_state_interfaces` | pure virtual | Exports all state interfaces for this hardware interface. returns `std::vector`  of [stateInterface](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/handle.hpp#L107) |
| `read`  | pure virtual | |
| `get_name ` | | |
| `get_state` | | |
| `set_state` | | |


# System Interface

Example [declared here](https://github.com/ros-controls/ros2_control_demos/blob/c9ab5c18e130742180d28009acbefa2f78f1a64e/ros2_control_demo_hardware/include/ros2_control_demo_hardware/rrbot_system_multi_interface.hpp#L40)  and [defined here](https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_hardware/src/rrbot_system_multi_interface.cpp)


| method | type | use |
| ------ | ---- | --- |
| Copy constructure | deleted | |
| Move constructure | defauled | |
| `on_init` | |  Initialization of the hardware interface from data parsed from the robot's URDF. |
| `export_state_interfaces` | pure virtual | Exports all state interfaces for this hardware interface. returns `std::vector`  of [stateInterface](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/handle.hpp#L107) |
| `export_command_interfaces` | pure virtual | xports all command interfaces for this hardware interface. returns `std::vector` of [commandInterface](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/include/hardware_interface/handle.hpp#L117) |
| `prepare_command_mode_switch` | | Prepare for any mode-switching required by the new command interface combination. |
| `perform_command_mode_switch` | | Perform switching to the new command interface. |
| `read`  | pure virtual | |
| `write` | pure virtual | |
| `get_name ` | | |
| `get_state` | | |
| `set_state` | | |

## Control Interface virtual funcions

| Function |  input  |  output | function |
| -------- | ---    | -------  | -------- |
| `command_interface_configuration` | none | `InterfaceConfiguration` |  |
| `state_interface_configuration` | none | `InterfaceConfiguration` | |
| `on_init` | none | `LifecycleNodeInterface::CallbackReturn` | |
| `update` | none | `controller_interface::return_type` | |


## Control Interface protected instances

| Instance/Varible | Description |
| ---------------- | ----------- |
|`std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;` ||
|`std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;` ||
|`std::shared_ptr<rclcpp::Node> node_;` ||
|`rclcpp_lifecycle::State lifecycle_state_;` ||
|`unsigned int update_rate_ = 0;` ||

## Control Forward control
Implements the following instances

Implements subscribers   [here](https://github.com/ros-controls/ros2_controllers/blob/c51ba1e45ff3868f7d50b164e3ed8162db4440f1/forward_command_controller/src/forward_command_controller.cpp#L80) which class `rt_command_ptr_.writeFromNonRT(recv_msg)`. 


Then in `update` it calls ` rt_command_ptr_.readFromRT();`, then copies the values into `ControlInterface::command_interfaces_[i]`[here](https://github.com/ros-controls/ros2_controllers/blob/c51ba1e45ff3868f7d50b164e3ed8162db4440f1/forward_command_controller/src/forward_command_controller.cpp#L162)

| Instance/Varible | Description |
| ---------------- | ----------- |
|`std::vector<std::string> joint_names_` | |
|`std::string interface_name_` ||
| `realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_` | |
| `rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_` ||
| `std::string logger_name_` ||

| Function |  Description   |
| -------- | ---    |
| `command_interface_configuration` | [here](https://github.com/ros-controls/ros2_controllers/blob/c51ba1e45ff3868f7d50b164e3ed8162db4440f1/forward_command_controller/src/forward_command_controller.cpp#L89) |
| `state_interface_configuration` | [here](https://github.com/ros-controls/ros2_controllers/blob/c51ba1e45ff3868f7d50b164e3ed8162db4440f1/forward_command_controller/src/forward_command_controller.cpp#L103) |
| `on_init` | [here](https://github.com/ros-controls/ros2_controllers/blob/c51ba1e45ff3868f7d50b164e3ed8162db4440f1/forward_command_controller/src/forward_command_controller.cpp#L40) |
| `update` | [here](https://github.com/ros-controls/ros2_controllers/blob/c51ba1e45ff3868f7d50b164e3ed8162db4440f1/forward_command_controller/src/forward_command_controller.cpp#L140) | 

