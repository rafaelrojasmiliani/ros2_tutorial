

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

- [Control Manager node](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp). Executes a [read](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/controller_manager.cpp#L1340), [update](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/controller_manager.cpp#L1342), [write](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/controller_manager.cpp#L1391) loop [here](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/ros2_control_node.cpp#L50)
    - [read](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/controller_manager.cpp#L1340), invoques a [`hardware_interface::ResoureceManager`](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/hardware_interface/include/hardware_interface/resource_manager.hpp#L37) method's [read](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/hardware_interface/src/resource_manager.cpp#L891). This method [loops](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/hardware_interface/src/resource_manager.cpp#L891) separatly through `hardware_interface::ResoureceManager::resource_storage_->{actuators_, sensors_, systems_}[i].read()`. Defitions of `actuators_`, `sensors_` and `systems_` is [here](https://github.com/ros-controls/ros2_control/blob/500233ac3d7f337881922ae7a96abd0a87b70dfa/hardware_interface/src/resource_manager.cpp#L171) 

