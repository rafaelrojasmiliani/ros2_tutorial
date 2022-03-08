

[documentation](http://control.ros.org/). and see [here](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md).

## Visibility control
[read here](https://gcc.gnu.org/wiki/Visibility).
## Control Managers

- [`control_manager.hpp`](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/include/controller_manager/controller_manager.hpp)

- [`ControlManager` definition](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/controller_manager.cpp)

- [Control Manager node](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp). Excutes a [read](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/controller_manager.cpp#L1340), [update](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/controller_manager.cpp#L1342), [write](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/controller_manager.cpp#L1391) loop [here](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/ros2_control_node.cpp#L50)
    - [read](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/controller_manager/src/controller_manager.cpp#L1340), invoques a [`hardware_interface::ResoureceManager`](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/hardware_interface/include/hardware_interface/resource_manager.hpp#L37) method's [read](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/hardware_interface/src/resource_manager.cpp#L891). This method [loops](https://github.com/ros-controls/ros2_control/blob/6b495ef86889cbfdc1ee22ac77efa2ff589727ee/hardware_interface/src/resource_manager.cpp#L891) separatly through `hardware_interface::ResoureceManager::resource_storage_->{actuators_, sensors_, systems_}`
