# This file tells docker what image must be created
# in order to be ahble to test this library
FROM rafa606/ros-base-galactic-vim

SHELL ["bash", "-c"]

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
        ros-galactic-turtlesim sudo \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd -g 11011 ros2user \
    && useradd -l -u 11011 -g ros2user ros2user \
    && install -d -m 0755 -o ros2user -g 11011 /home/ros2user \
    && usermod -aG sudo ros2user \
    && echo "$ros2user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "umask 000" >> /etc/bash.bashrc





WORKDIR /workspace
