
The rosbag api is [here](https://github.com/ros2/rosbag2/tree/master).

A good example of the api is [here](https://github.com/ros2/rosbag2/blob/master/rosbag2_tests/test/rosbag2_tests/test_rosbag2_cpp_api.cpp)


## Serialization

Seems that serialization combines ridl with rmw.

- `SerializationBase` [declared here](https://github.com/ros2/rclcpp/blob/ea8daa37845e6137cba07a18eb653d97d87e6174/rclcpp/include/rclcpp/serialization.hpp#L48) [defined here](https://github.com/ros2/rclcpp/blob/ea8daa37845e6137cba07a18eb653d97d87e6174/rclcpp/src/rclcpp/serialization.cpp#L30)

- `Serialization` [declared and defined here](https://github.com/ros2/rclcpp/blob/ea8daa37845e6137cba07a18eb653d97d87e6174/rclcpp/include/rclcpp/serialization.hpp#L83)

- The core of messages serialization is [here](https://github.com/ros2/rclcpp/blob/ea8daa37845e6137cba07a18eb653d97d87e6174/rclcpp/src/rclcpp/serialization.cpp#L43)
- The core of messages deserialization is [here](https://github.com/ros2/rclcpp/blob/ea8daa37845e6137cba07a18eb653d97d87e6174/rclcpp/src/rclcpp/serialization.cpp#L65)


## ROScpp utils

[repo here](https://github.com/ros2/rcpputils).



## File system path

[the class `path` is declared here](https://github.com/ros2/rcpputils/blob/8a94c1c189561bfd8115e1cbdd2c94f5706ef323/include/rcpputils/filesystem_helper.hpp#L74) and [defined here](https://github.com/ros2/rcpputils/blob/8a94c1c189561bfd8115e1cbdd2c94f5706ef323/src/filesystem_helper.cpp#L74).


## ROSBag

rosbag2 introduces the verb called `bag` available as `ros2 bag`:
* record
```
$ ros2 bag record <topic1> <topic2> … <topicN>
```
If not further specified, `ros2 bag record` will create a new folder named to the current time stamp and stores all data within this folder.
A user defined name can be given with `-o, --output`.
By default rosbag2 does not record with compression enabled. However, compression can be specified using the following CLI options.

For example, `ros2 bag record -a --compression-mode file --compression-format zstd` will record all topics and compress each file using the [zstd](https://github.com/facebook/zstd) compressor.
* play
```
$ ros2 bag play <bag_file>
```
* info
```
$ ros2 bag info <bag_file>
```
* convert
Rosbag2 provides a tool `ros2 bag convert` (or, `rosbag2_transport::bag_rewrite` in the C++ API).
This allows the user to take one or more input bags, and write them out to one or more output bags with new settings.
Here is an example command:
```
ros2 bag convert --input /path/to/bag1 --input /path/to/bag2 storage_id --output-options output_options.yaml
```

## Storage format plugin architecture

Looking at the output of the `ros2 bag info` command, we can see a field called `storage id:`.
rosbag2 specifically was designed to support multiple storage formats.
As of now, this repository comes with two storage plugins.
The first plugin, sqlite3 is chosen by default.

## Serialization format plugin architecture

By design, ROS 2 is middleware agnostic and thus can leverage multiple communication frameworks.
The default middleware for ROS 2 is DDS which has `cdr` as its default binary serialization format.
However, other middleware implementation might have different formats.
If not specified, `ros2 bag record -a` will record all data in the middleware specific format.
This however also means that such a bag file can't easily be replayed with another middleware format.

rosbag2 implements a serialization format plugin architecture which allows the user the specify a certain serialization format.
When specified, rosbag2 looks for a suitable converter to transform the native middleware protocol to the target format.
This also allows to record data in a native format to optimize for speed, but to convert or transform the recorded data into a middleware agnostic serialization format.

By default, rosbag2 can convert from and to CDR as it's the default serialization format for ROS 2.
