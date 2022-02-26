# ToDo
1) adapt the docker file building the aas_ws of this assignment
2) replace this template text with instructions on how to setup up your docker and run the code
3) please adapt this documentation; replace all `your_docker_name` by `lab_`[X]`_docker_`[your team name]


# Docker


## Setup
in `[lab_path]/docker`, build the image from a Dockerfile
```
sudo DOCKER_BUILDKIT=1 docker build -t lab4 . --no-cache --ssh default=$SSH_AUTH_SOCK
```
and run it
```
sudo docker run -it -d --network=host --name lab4 lab4:latest bash
```

## Starting and Interacting
Start the docker container
```
sudo docker start lab4
```

Open an interactive shell
```
sudo docker exec -it lab4 bash
```

to exit the docker shell type `exit`

## Stopping and Deleting
stop the container:
```
sudo docker stop lab4
```
delete the container
```
sudo docker rm lab4
```

# Getting things going

You will need three terminals

### Terminal 1, on host:
```
roscore
```

### Terminal 2, on host, after building in workspace:
```
source devel/setup.bash
roslaunch unity_bridge unity_sim.launch
```

### Terminal 3, in docker
you will need to adapt this to work with your launch file
```
source devel/setup.bash
roslaunch controller_pkg controller.launch
```

