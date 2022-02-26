# Lab 6, Autonomous Systems WS21, Group aerialscrew

## Task 1
When r=4 and we have k segments:
waypoint constraints: k-1, free derivative constraints: 2(k-1), fixed derivative constrainst:2

When we have r-th derivative and k segments: waypoint constraints: k-1, free derivative constraints: (r-2)(k-1), fixed derivative constrainst:r-2

## Task 2


### Setup
in `[lab_path]/docker`, build the image from a Dockerfile
```
sudo DOCKER_BUILDKIT=1 docker build -t lab6 . --no-cache --ssh default=$SSH_AUTH_SOCK
```
and run it
```
sudo docker run -it -d --network=host --name lab6 lab6:latest bash
```

### Starting and Interacting
Start the docker container
```
sudo docker start lab6
```

Open an interactive shell
```
sudo docker exec -it lab6 bash
```

to exit the docker shell type `exit`

### Stopping and Deleting
stop the container:
```
sudo docker stop lab6
```
delete the container
```
sudo docker rm lab6
```

### Getting things going

#### Terminal 1, on host, after building in workspace:
```
source devel/setup.bash
roslaunch unity_bridge unity_sim.launch
```

#### Terminal 2, in docker
you will need to adapt this to work with your launch file
```
source devel/setup.bash
rosrun controller_pkg controller_node
```
#### Terminal 3, in docker
```
source devel/setup.bash
roslaunch basic_waypoint_pkg waypoint_mission.launch
```

