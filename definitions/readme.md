
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
