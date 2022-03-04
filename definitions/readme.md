
# Definitions

- Service Discovery: Is an automatic methodology to find the services available in a computer network and enable the exploitation of these services. This includes:
    - Specific protocol of the services
    - IP of the host and port of the service
- **RMW** ROS Middleware: [(repo here)](https://github.com/ros2/rmw) The ROS API exposes communication concepts like publish / subscribe. In ROS 1 these concepts are built on TCPROS. ROS 2 build them on top of DDS, but it does not expose any DDS implementation, keeping the middleware interface free of DDS specific concepts to enable implementations of the interface using a different middleware, e.g. multiple DDS implementations. RMW is the abstraction layer introduced to implemented different DDS implementations. RMW defines the API between the ROS client library and any specific implementation of DDS. The API interface to custom DDS implementations in [here](https://github.com/ros2/rmw_implementation/tree/master/rmw_implementation)
    - **Implementations of DDS**
    - **eProsima's Fast DDS** [interface](https://github.com/ros2/rmw_fastrtps) [dds implementation](https://github.com/eProsima/Fast-DDS). Also implments RTPS
    - [**Cyclone DDS**](https://github.com/ros2/rmw_cyclonedds). Also implements RTPS.

- **RCL** ROS Client Library: [(repo here)](https://github.com/ros2/rcl) Is the developer-oriented custom language implementation of ROS communications methods.

- **DDS** Distributed Data Services and **RTPS** Real-Time Publish and Subscribe: ROS 2 communications are based on DDS. On the top of DDS it implements RTPS [(see here)](https://www.omg.org/spec/DDSI-RTPS/2.5/PDF). It provides **discovery** and serialization and transportation.
    - **`ROS_DOMAIN_ID`** environment variable to set the multicasting-based service discovery UDP port. This will build  relay ROS network. This allows to have different ros networks in the same LAN. From this perspective it has the same use of the `ROS_MASTER_URI` enviroment variable.

    This article explains the motivation behind using DDS implementations, and/or the RTPS wire protocol of DDS, in detail. In summary, DDS is an end-to-end middleware that provides features which are relevant to ROS systems, such as distributed discovery (not centralized like in ROS 1) and control over different “Quality of Service” options for the transportation.

- **rcl** Ros Client Library [(repo)](https://github.com/ros2/rcl/). API to instantied a ROS client inside a posix process in order to make this posix process a ros node.

- **colcon** [(documentation)](https://colcon.readthedocs.io/en/released/index.html) [(tutorial)](https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html)  the new generation  command line tool to improve the workflow of building, testing and using multiple software packages

- **ament package**: a single unit of software which is described using a package.xml manifest file.
- **ament CMake package**
- **ament Python package**

- **ament**: is the name given to the new developments of **catkin** implemented in ROS2.
is a meta-build systems i.e. it sits atop other build systems such as CMake and Python setuptools.
It provides extra functionalities to make those build systems easier to use (managing dependencies, building multiple packages in a single workspace). It does two things:
    - **1.** Add API (python or c++) to the underlying build system 
    - **2.** Provide a command-line tool that can be used to iterate in dependency order over a workspace full of packages, building and perhaps installing each one.

- **Migrating catkin to ament** The effort should be minimized. Migration to a new meta-build system should not be required without a very good reason. Developers should be able to mix and match meta-build systems. Workspaces do not need be homogeneous. Catkin and ament packages can coexists in one workspace, with dependencies going in both directions. The primary interface between packages is their CMake configuration file. 

- **Adding ROS packages to a ROS 2 workspace and building with ament build** Let's say that you want to add some existing ROS packages to your ROS 2 workspace and don’t want to migrate the ROS packages from catkin to ament (or vice versa). Here are two patches that let you do that:
    - **`ament_package`**: Add support for format 1 package manifests, instead of requiring format 2. This change isn’t strictly related to catkin vs. ament, because format 2 has been around for a while and catkin supports it, so developers could already update their manifests to format 2. But there’s a ton of ROS code out there that uses format 1, so we should support it. This implementation could be improved, e.g., by reasoning over the various flavors of depend tags and how they differ between formats 1 and 2.
    - **`ament_tools`**: Add a new catkin build type to ament. This implementation just treats catkin packages the same as plain cmake packages, which seems to work fine. It could be made more sophisticated
