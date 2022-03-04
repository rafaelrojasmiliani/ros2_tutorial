# ROS2 node

A ROS node is a POSIX process with the minimal ROS functionalities that makes a ROS node.
The API to instantiate these minimal inside a posix process is caled ROS Client library or `rcl`.
In C++ a node is defined [here](https://github.com/ros2/rclcpp/blob/galactic/rclcpp/include/rclcpp/node.hpp) with macros-based definitions given [here](https://github.com/ros2/rclcpp/blob/1037822a63330495dcf0de5e8f20544375a5f116/rclcpp/include/rclcpp/macros.hpp).
In ROS2 it is possible to compose seveeral nodes in a single process [see here](https://docs.ros.org/en/galactic/Tutorials/Composition.html).

- `rclcpp::init` [declared here](https://github.com/ros2/rclcpp/blob/16914e31a15417d5751c4cb611f591bbaa458eca/rclcpp/include/rclcpp/context.hpp#L128) [defined here](https://github.com/ros2/rclcpp/blob/54c2a8ac5bf14b9353765e94db5042630b710a75/rclcpp/src/rclcpp/utilities.cpp#L34) initialize the "node context".
- **Node Creation**. Nodes inherit from `enable_shared_from_this`, so they provide safety method to generare share pointers to node's instances. They also [define](https://github.com/ros2/rclcpp/blob/54c2a8ac5bf14b9353765e94db5042630b710a75/rclcpp/src/rclcpp/utilities.cpp#L34) useful static factory methods such as `Node::make_shared`, which calls a `std::make_shared` with the correct arguments to instantiate a new node.
- **Publisher creation**
    - `Node::create_publisher` [example here](https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) [declared here](https://github.com/ros2/rclcpp/blob/1037822a63330495dcf0de5e8f20544375a5f116/rclcpp/include/rclcpp/node_impl.hpp#L73).
    - `rclcpp::create_publisher` [defined here](https://github.com/ros2/rclcpp/blob/1037822a63330495dcf0de5e8f20544375a5f116/rclcpp/include/rclcpp/create_publisher.hpp#L94)

- **Subscriver creation** [example here](https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html). At its base, subscriptions use a class [declared here](https://github.com/ros2/rclcpp/blob/1037822a63330495dcf0de5e8f20544375a5f116/rclcpp/include/rclcpp/subscription.hpp#L69) which should never be directly used the the user. the `rcl` provides usefull methods
    - `Node::create_subscription`
    - `rclcpp::create_subscription` [declared and defined here](https://github.com/ros2/rclcpp/blob/1037822a63330495dcf0de5e8f20544375a5f116/rclcpp/include/rclcpp/create_subscription.hpp#L183)

- **Service server creation** [example here](https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html)
    - `Node::create_service`
- **Service client creation** Requires a custom class `rclcpp::Client`
    - `Node::create_client`
    - `Client::async_send_request`

- **Live cycle** [see here](https://github.com/ros2/demos/blob/master/lifecycle/README.rst)
    - `rclcpp::ok` **contex dependent**
    - `rclcpp::shutdown`
    - `rclcpp::spin_until_future_complete`
    - `rclcpp::spin` **requires a node**

- **Parameters** [example here](https://docs.ros.org/en/galactic/Tutorials/Using-Parameters-In-A-Class-CPP.html) **Dynamic reconfigure does intentionally not existing in ROS 2** and its features have been rolled into the ROS 2 parameters. So dynamic reconfigure should not be ported to ROS 2. If any functionality is missing it should be added to the existing ROS 2 parameters [see here](https://discourse.ros.org/t/dynamic-reconfigure-porting/6497/2), [here](http://design.ros2.org/articles/ros_parameters.html) and [here](https://github.com/ros-planning/navigation2/issues/177).
    - `Node::declare_parameter`
    - `Node::get_parameter`

- **Actions** [example here](https://docs.ros.org/en/galactic/Tutorials/Actions/Writing-a-Cpp-Action-Server-Client.html)
    - `rclcpp_action::create_server`
    - `rclcpp_action::create_client`

# Interfaces

[tutorial](https://docs.ros.org/en/galactic/Concepts/About-ROS-Interfaces.html).


- **Creting custom interfaces**
    - `rosidl_generate_interfaces`
    - `rosidl_target_interfaces`
    - CMake example to generate message
    ```
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/Num.msg"
        "srv/AddThreeInts.srv"
        )
    ```
    - `package.xml` example to import message generation stuff
    ```XML
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
    - linking agains inerfaces in the same package
    ```
    rosidl_target_interfaces(target_to_line ${PROJECT_NAME} "rosidl_typesupport_cpp")
    ```
