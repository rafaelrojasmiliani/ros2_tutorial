#!/usr/bin/env vash

myuid=$(id -u $USER)
mygid=$(id -g $USER)
mygroup=$(id -g -n $USER)

docker build -t "rafa606/ros2-tutorial" -f ./image.dockerfile .

exit 0
