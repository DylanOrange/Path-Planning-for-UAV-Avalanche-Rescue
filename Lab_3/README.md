# Lab 3, Autonomous Systems WS21

## Installation

Follow  the [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/) to install Docker.

For systems with NVIDIA GPUs, install `nvidia-container-toolkit` to make sure that rViz GUI inside the docker can run smoothly. To install it, follow the [Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html), section `Docker` -> `Installing on Ubuntu and Debian`.

## Build the image

To build the docker image run

```
sudo DOCKER_BUILDKIT=1 docker build -t lab3 . --no-cache --ssh default=$SSH_AUTH_SOCK
```


## Run the image

To run the image

```
xhost +local:root
sudo docker run -it -e DISPLAY=unix$DISPLAY -e --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" lab3
```

For systems with nvidia-container-toolbox installed, replace the last command with

```
sudo docker run -it -e DISPLAY=unix$DISPLAY -e --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --gpus 'all,"capabilities=display"' lab3
```


To start the two_drones package:

```
source devel/setup.bash
roslaunch two_drones_pkg two_drones.launch
```
then you can see two drones flying like lab2
