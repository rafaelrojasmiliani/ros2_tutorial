# This file tells docker what image must be created
# in order to be ahble to test this library
FROM rafa606/ros-base-galactic-vim

SHELL ["bash", "-c"]

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
        ros-galactic-turtlesim sudo \
        ros-galactic-force-torque-sensor-broadcaster \
        ros-galactic-forward-command-controller \
        ros-galactic-joint-state-broadcaster \
        ros-galactic-joint-trajectory-controller \
        ros-galactic-joint-state-publisher-gui \
        ros-galactic-rviz2 \
        ros-galactic-xacro \
        ros-galactic-hardware-interface \
        ros-galactic-gazebo-ros2-control \
        ros-galactic-example-interfaces \
        ros-galactic-rmw-fastrtps-cpp \
        ros-galactic-ros-testing \
        ros-galactic-rttest \
        ros-galactic-tlsf-cpp \
        libopencv-dev \
    && rm -rf /var/lib/apt/lists/* \
    && echo "umask 000" >> /etc/bash.bashrc \
    && echo "source /opt/ros/galactic/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace
