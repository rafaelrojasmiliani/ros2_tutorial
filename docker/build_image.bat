
@ECHO OFF

docker build -t "rafa606/ros2-tutorial" ^
    --no-cache -f ./image.dockerfile .
