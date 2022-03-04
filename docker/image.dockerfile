# This file tells docker what image must be created
# in order to be ahble to test this library
FROM rafa606/ros-base-galactic-vim

SHELL ["bash", "-c"]
# user handling
ARG myuser
ARG myuid
ARG mygroup
ARG mygid
ARG scriptdir
RUN addgroup --gid ${mygid} ${mygroup} --force-badname \ 
    && adduser --gecos "" --disabled-password  --uid ${myuid} --gid ${mygid} ${myuser} --force-badname \
    && echo "${myuser} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
        ros-galactic-turtlesim \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
