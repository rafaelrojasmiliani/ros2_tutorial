
- use plugins from external libraries [here](https://github.com/ros/pluginlib/issues/161)

There are two class loaders
    - [`ros/class_loader`](https://github.com/ros/class_loader/tree/humble) ROS-independent package for loading plugins during runtime and the foundation of the higher level ROS pluginlib library.
    - [`ros/pluginlib`](https://github.com/ros/pluginlib/tree/humble). Header-only library

Example of how a plugin is loaded in foxy

- [1](https://github.com/ros2/rosbag2/blob/9a74ad8ee9394c86b1940ac05cfd5c20b1a2fddc/rosbag2_storage/src/rosbag2_storage/impl/storage_factory_impl.hpp#L75)
- [**2**]()

## `ros/pluginlib`

### `ClassLoader`

This is a template class [declared here](https://github.com/ros/pluginlib/blob/humble/pluginlib/include/pluginlib/class_loader.hpp) and [defined here](https://github.com/ros/pluginlib/blob/fc2d015a594bb582c1143a9e2b8b8d9f6f8c75fc/pluginlib/include/pluginlib/class_loader_imp.hpp#L72)

- **Constructor** see [here](https://github.com/ros/pluginlib/blob/fc2d015a594bb582c1143a9e2b8b8d9f6f8c75fc/pluginlib/include/pluginlib/class_loader_imp.hpp#L72)


- [`amend_index_cpp::get_resource`](https://github.com/ament/ament_index/blob/f019d6c40991799a13b18c9c3dcc583e3fde0381/ament_index_cpp/src/get_resource.cpp#L28)

```C++
#include "ament_index_cpp/get_package_prefix.hpp"    // https://github.com/ament/ament_index/blob/rolling/ament_index_cpp/src/get_package_prefix.cpp
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_search_paths.hpp"
const char* lookup_name = rosbag2_storage::StorageTraits<rosbag2_storage::storage_interfaces::ReadWriteInterface>::name;
auto class_loader =
std::make_shared<pluginlib::ClassLoader<rosbag2_storage::storage_interfaces::ReadWriteInterface>>("rosbag2_storage", lookup_name);
std::string databaseName = "rosbag_string_database
const auto& registered_classes = class_loader->getDeclaredClasses();
		for (const auto& cla : registered_classes)
		{
			LOG_WARN("--- class " + cla);
		}
		auto unmanaged_instance = class_loader->createUnmanagedInstance("sqlite3");
		LOG_WARN("-------------------------- ");
		LOG_WARN("-------------------------- ");
		auto instance = std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface>(unmanaged_instance);
		LOG_WARN("-------------------------- ");
```
