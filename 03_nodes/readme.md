# ROS2 Node

## Intro

In ROS1 a node could be identified with a POSIX process with the minimal ROS functionalities, e.g. parameter server, log topics, etc.
However, in ROS2 a single POSIX process or thread can contain more than one node, and this identification is no longer possible.
The act of running several nodes in the same process is called [composition](https://docs.ros.org/en/foxy/Tutorials/Composition.html) and it is implmented by [`rclcpp_components`](https://github.com/ros2/rclcpp/tree/master/rclcpp_components).

In ROS2 the concept of node is intimately related to the concept of [executors](https://docs.ros.org/en/galactic/Concepts/About-Executors.html).
See [here](http://design.ros2.org/articles/node_lifecycle.html) to deepen on ros2 nodes.
There are two types of nodes:

| Normal nodes   |  Life cycle nodes                 |
| ------------   | --------------------------------- |
| `rclcpp::Node` | `rclcpp_lifecycle::LifecycleNode` |
| `launch_ros.actions.Node` | `launch_ros.actions.LifecycleNode` |

## Executors


An Executor uses one or more threads of the underlying operating system to invoke the callbacks of subscriptions, timers, service servers, action servers, etc. on incoming messages and events that make up a node.
In other words, it implements the exploitation of the OS to query the rcl and middleware layers for incoming messages and other events and calls the corresponding callback functions until the node shuts down.
In order not to counteract the QoS settings of the middleware, an incoming message is not stored in a queue on the Client Library layer but kept in the middleware until it is taken for processing by a callback function.
This is a crucial difference to ROS 1.
A wait set is used to inform the Executor about available messages on the middleware layer, with one binary flag per queue.

There are several types of Executors.
Each one in an implementation of the pure virutal base clasee [`Executor`](https://github.com/ros2/rclcpp/blob/011ea39e9909d74faaa0cbbbbb1def8d48760e9e/rclcpp/include/rclcpp/executor.hpp#L65) partialiy defined [here](https://github.com/ros2/rclcpp/blob/master/rclcpp/src/rclcpp/executor.cpp).

- [`Executor::spin_once`](https://github.com/ros2/rclcpp/blob/011ea39e9909d74faaa0cbbbbb1def8d48760e9e/rclcpp/src/rclcpp/executor.cpp#L471)
- [`Executor::spin_once_impl`](https://github.com/ros2/rclcpp/blob/011ea39e9909d74faaa0cbbbbb1def8d48760e9e/rclcpp/src/rclcpp/executor.cpp#L462)
- [`Executor::execute_any_executable`](https://github.com/ros2/rclcpp/blob/011ea39e9909d74faaa0cbbbbb1def8d48760e9e/rclcpp/src/rclcpp/executor.cpp#L503)


Currently, rclcpp provides three Executor types, derived from a shared parent class:
- **Single-Threaded Executor** use a single thread. Declared [here](https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/executors/single_threaded_executor.hpp) and defined [here](https://github.com/ros2/rclcpp/blob/master/rclcpp/src/rclcpp/executors/single_threaded_executor.cpp)
- **Multi-Threaded Executor** creates a configurable number of threads to allow for processing multiple messages or events in parallel. Declared [here](https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/executors/multi_threaded_executor.hpp) and defined [here](https://github.com/ros2/rclcpp/blob/master/rclcpp/src/rclcpp/executors/multi_threaded_executor.cpp)
- **Static Single-Threaded Executor** optimizes the runtime costs for scanning the structure of a node in terms of subscriptions, timers, service servers, action servers, etc. It performs this scan only once when the node is added, while the other two executors regularly scan for such changes. Therefore, the Static Single-Threaded Executor should be used only with nodes that create all subscriptions, timers, etc. during initialization. It is declared [here](https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/executors/static_single_threaded_executor.hpp) and defined [here](https://github.com/ros2/rclcpp/blob/master/rclcpp/src/rclcpp/executors/static_single_threaded_executor.cpp)

Executors can be used to run multiple nodes by calling, thanks to [`Executor::add_node`](https://github.com/ros2/rclcpp/blob/011ea39e9909d74faaa0cbbbbb1def8d48760e9e/rclcpp/src/rclcpp/executor.cpp#L251) e.g.
```C++
rclcpp::Node::SharedPtr node1 = ...
rclcpp::Node::SharedPtr node2 = ...
rclcpp::Node::SharedPtr node3 = ...

rclcpp::executors::StaticSingleThreadedExecutor executor;
executor.add_node(node1);
executor.add_node(node2);
executor.add_node(node2);
executor.spin();
```


### `rclcpp::sping`

The simplest implementation of an executore is donde in `rclcpp::spin` declared [here](https://github.com/ros2/rclcpp/blob/011ea39e9909d74faaa0cbbbbb1def8d48760e9e/rclcpp/include/rclcpp/executors.hpp#L45) and definded [here](https://github.com/ros2/rclcpp/blob/011ea39e9909d74faaa0cbbbbb1def8d48760e9e/rclcpp/src/rclcpp/executors.cpp#L31) as follows

```C++
void
rclcpp::spin(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_ptr);
  exec.spin();
  exec.remove_node(node_ptr);
}
```

`rclcpp::spin` instantiates a Single-Threaded Executor, and is intended to be used in the `main` as

```C++
int main(int argc, char* argv[])
{
   // Some initialization.
   rclcpp::init(argc, argv);
   ...

   // Instantiate a node.
   rclcpp::Node::SharedPtr node = ...

   // Run the executor.
   rclcpp::spin(node);

   // Shutdown and exit.
   ...
   return 0;
}
```

### Callback groups

The rclcpp allows organizing the callbacks of a node in groups.
Such a callback group can be created by the [`Node::create_callback_group`](https://github.com/ros2/rclcpp/blob/011ea39e9909d74faaa0cbbbbb1def8d48760e9e/rclcpp/src/rclcpp/node.cpp#L306) function of the Node class.
The callback group must be stored throughout execution of the node (eg.
as a class member), or otherwise the executor won’t be able to trigger the callbacks.
Then, this callback group can be specified when creating a subscription, timer, etc.
For example by the subscription options:
```C++
my_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

rclcpp::SubscriptionOptions options;
options.callback_group = my_callback_group;

my_subscription = create_subscription<Int32>("/topic", rclcpp::SensorDataQoS(),
                                             callback, options);

```

All subscriptions, timers, etc.
that are created without the indication of a callback group are assigned to the default callback group.
The default callback group can be queried via `NodeBaseInterface::get_default_callback_group`.

There are two types of callback groups, where the type has to be specified at instantiation time:

- **Mutually exclusive:** Callbacks of this group must not be executed in parallel.

- **Reentrant:** Callbacks of this group may be executed in parallel.

Callbacks of different callback groups may always be executed in parallel. The Multi-Threaded Executor uses its threads as a pool to process a many callbacks as possible in parallel according to these conditions.

Since Galactic, the interface of the Executor base class has been refined by a new function [`Executor::add_callback_group`](https://github.com/ros2/rclcpp/blob/011ea39e9909d74faaa0cbbbbb1def8d48760e9e/rclcpp/src/rclcpp/executor.cpp#L237).
This allows distributing callback groups to different Executors.
By configuring the underlying threads using the operating system scheduler, specific callbacks can be prioritized over other callbacks.
For example, the subscriptions and timers of a control loop can be prioritized over all other subscriptions and standard services of a node.
The [`examples_rclcpp_cbg_executor`](https://github.com/ros2/examples/tree/master/rclcpp/executors/cbg_executor) package provides a demo of this mechanism.

### Scheduling semantics

If the processing time of the callbacks is shorter than the period with which messages and events occur, the Executor basically processes them in FIFO order.
However, if the processing time of some callbacks is longer, messages and events will be queued on the lower layers of the stack.
The wait set mechanism reports only very little information about these queues to the Executor.
In detail, it only reports whether there are any messages for a certain topic or not.
The Executor uses this information to process the messages (including services and actions) in a round-robin fashion - but not in FIFO order.
In addition, it prioritizes all timer events over the messages.
The following flow diagram visualizes this scheduling semantics.

This semantics was first described in a paper by Casini et al. at ECRTS 2019.


## Normal nodes

In C++ a node is defined [here](https://github.com/ros2/rclcpp/blob/galactic/rclcpp/include/rclcpp/node.hpp) with macros-based definitions given [here](https://github.com/ros2/rclcpp/blob/1037822a63330495dcf0de5e8f20544375a5f116/rclcpp/include/rclcpp/macros.hpp).
In ROS2 it is possible to compose several nodes in a single process [see here](https://docs.ros.org/en/galactic/Tutorials/Composition.html).

- `rclcpp::init` [declared here](https://github.com/ros2/rclcpp/blob/16914e31a15417d5751c4cb611f591bbaa458eca/rclcpp/include/rclcpp/context.hpp#L128) [defined here](https://github.com/ros2/rclcpp/blob/54c2a8ac5bf14b9353765e94db5042630b710a75/rclcpp/src/rclcpp/utilities.cpp#L34) initialize the "node context".
- **Node Creation**. Nodes inherit from `enable_shared_from_this`, so they provide safety method to generare share pointers to node's instances. They also [define](https://github.com/ros2/rclcpp/blob/54c2a8ac5bf14b9353765e94db5042630b710a75/rclcpp/src/rclcpp/utilities.cpp#L34) useful static factory methods such as `Node::make_shared`, which calls a `std::make_shared` with the correct arguments to instantiate a new node.

- [managed nodes](https://answers.ros.org/question/301844/ros2-node-vs-managed-node/) [more](https://answers.ros.org/question/341887/recommendation-to-manage-multiple-lifecycle-nodes/)
# Interfaces

[tutorial](https://docs.ros.org/en/galactic/Concepts/About-ROS-Interfaces.html).

## Register nodes

[`RCLCPP_COMPONENTS_REGISTER_NODE`]


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

# ROS parameters
- **Parameters** [example here](https://docs.ros.org/en/galactic/Tutorials/Using-Parameters-In-A-Class-CPP.html) **Dynamic reconfigure does intentionally not existing in ROS 2** and its features have been rolled into the ROS 2 parameters. So dynamic reconfigure should not be ported to ROS 2. If any functionality is missing it should be added to the existing ROS 2 parameters [see here](https://discourse.ros.org/t/dynamic-reconfigure-porting/6497/2), [here](http://design.ros2.org/articles/ros_parameters.html) and [here](https://github.com/ros-planning/navigation2/issues/177).
    - `Node::declare_parameter`
    - `Node::get_parameter`

# Actions
- **Actions** [example here](https://docs.ros.org/en/galactic/Tutorials/Actions/Writing-a-Cpp-Action-Server-Client.html)
    - `rclcpp_action::create_server`
    - `rclcpp_action::create_client`
