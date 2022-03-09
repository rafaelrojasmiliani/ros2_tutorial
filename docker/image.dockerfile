# This file tells docker what image must be created
# in order to be ahble to test this library
FROM rafa606/ros-base-foxy-vim

SHELL ["bash", "-c"]

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
        ros-foxy-turtlesim sudo \
        ros-foxy-force-torque-sensor-broadcaster \
        ros-foxy-forward-command-controller \
        ros-foxy-joint-state-broadcaster \
        ros-foxy-joint-trajectory-controller \
        ros-foxy-joint-state-publisher-gui \
        ros-foxy-rviz2 \
        ros-foxy-xacro \
        ros-foxy-hardware-interface \
        ros-foxy-gazebo-ros2-control \
        ros-foxy-example-interfaces \
        ros-foxy-rmw-fastrtps-cpp \
        ros-foxy-ros-testing \
        ros-foxy-rttest \
        ros-foxy-tlsf-cpp \
        ros-foxy-ament-lint \
        ros-foxy-rosidl-default-generators \
        ros-foxy-ament-lint-auto \
        ros-foxy-ament-cmake-flake8 \
        ros-foxy-ros2launch \
        ros-foxy-ament-pep257 \
        libopencv-dev \
    && rm -rf /var/lib/apt/lists/* \
    && echo "umask 000" >> /etc/bash.bashrc \
    && echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace
