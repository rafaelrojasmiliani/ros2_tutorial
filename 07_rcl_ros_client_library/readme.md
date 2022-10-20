
[`rcl_interfaces`](https://github.com/ros2/rcl_interfaces/tree/master/rcl_interfaces#topics) contains the messages and services which ROS client libraries will use under the hood to communicate higher level concepts such as parameters.

[ROS2 Client Library](https://github.com/ros2/rcl/tree/rolling/rcl)

[ROS2 Client Library Interfaces](https://github.com/ros2/rcl_interfaces)


RAII Resource acquisition is initialization


[Definition Node interface for parameters](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp)
[Declaration Node interface for parameters](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/include/rclcpp/node_interfaces/node_parameters.hpp)

[`NodeParametersInterface` declaration](https://github.com/ros2/rclcpp/blob/145933b03793af008a54c9203185a23ec2fde9b0/rclcpp/include/rclcpp/node_interfaces/node_parameters_interface.hpp#L71)



Load parameters in yaml [here](https://github.com/ros2/rclcpp/issues/1029)

- **1** `Node::declare_paramenter` [defined](https://github.com/ros2/rclcpp/blob/145933b03793af008a54c9203185a23ec2fde9b0/rclcpp/include/rclcpp/node.hpp#L423) and [defined here](https://github.com/ros2/rclcpp/blob/145933b03793af008a54c9203185a23ec2fde9b0/rclcpp/src/rclcpp/node.cpp#L314)

- **2** `rclcpp::node_interfaces::NodeParametersInterface::declare_parameter` [declared here](https://github.com/ros2/rclcpp/blob/145933b03793af008a54c9203185a23ec2fde9b0/rclcpp/include/rclcpp/node_interfaces/node_parameters.hpp#L118) and [defined here](https://github.com/ros2/rclcpp/blob/145933b03793af008a54c9203185a23ec2fde9b0/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp#L575)

- **3** which calls the static function [`declare_parameter_helper`](https://github.com/ros2/rclcpp/blob/145933b03793af008a54c9203185a23ec2fde9b0/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp#L499)

- **4** calls [`__declare_parameter_common`](https://github.com/ros2/rclcpp/blob/145933b03793af008a54c9203185a23ec2fde9b0/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp#L435)

- **5** [`__set_parameters_atomically_common`](https://github.com/ros2/rclcpp/blob/145933b03793af008a54c9203185a23ec2fde9b0/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp#L398)
	- **5.1** [`__check_parameter_value_in_range`](https://github.com/ros2/rclcpp/blob/145933b03793af008a54c9203185a23ec2fde9b0/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp#L197)
	- **5.2** [`__call_on_set_parameters_callbacks`](https://github.com/ros2/rclcpp/blob/145933b03793af008a54c9203185a23ec2fde9b0/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp#L353)
	- **5.3** [`__call_post_set_parameters_callbacks`](https://github.com/ros2/rclcpp/blob/145933b03793af008a54c9203185a23ec2fde9b0/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp#L376)



## Ros parameter class

[declared here](https://github.com/ros2/rclcpp/blob/145933b03793af008a54c9203185a23ec2fde9b0/rclcpp/include/rclcpp/parameter.hpp#L52) and [defined here](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/parameter.cpp)

## ROS parameter value
[declared here](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/include/rclcpp/parameter_value.hpp) and [defined here](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/parameter_value.cpp)


## C interface

```C++
std::vector<rclcpp::Parameter> createParamsListFromYAMLs(const std::vector<std::string>& yaml_paths)
{
	std::vector<rclcpp::Parameter> res;

	rcutils_allocator_t allocator = rcutils_get_default_allocator();
	std::map<std::string, rclcpp::Parameter> parameters;
	std::string combined_name_;


	std::cout << "Parsing parameters for " << combined_name_ << std::endl;

	for (const std::string& yaml_path : yaml_paths)
	{
		rcl_params_t* yaml_params = rcl_yaml_node_struct_init(allocator);

		if (nullptr == yaml_params)
		{
			throw std::bad_alloc();
		}

		if (!rcl_parse_yaml_file(yaml_path.c_str(), yaml_params))
		{
			std::ostringstream ss;
			ss << "Failed to parse parameters from file '" << yaml_path << "': " << rcl_get_error_string().str;
			rcl_reset_error();
			throw std::runtime_error(ss.str());
		}

		rclcpp::ParameterMap initial_map = rclcpp::parameter_map_from(yaml_params);
		rcl_yaml_node_struct_fini(yaml_params);
		auto iter = initial_map.find("/move_group");

		if (initial_map.end() == iter)
		{
			continue;
		}

		// Combine parameter yaml files, overwriting values in older ones
		for (auto& param : iter->second)
		{
			parameters[param.get_name()] = param;
		}
	}

	res.reserve(parameters.size());

	for (auto& kv : parameters)
	{
		res.emplace_back(kv.second);
	}

	return res;
}
```
