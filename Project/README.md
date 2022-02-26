# Group Projects

## Run current code
12.2 update: an implementation of the algorithm in paper [Multipurpose UAV for search and rescue operations in mountain avalanche events](https://gitlab.lrz.de/ga84sij/autonomous-systems-2021-group/-/blob/main/Project/papers/Multipurpose%20UAV%20for%20search%20and%20rescue%20operations%20in%20mountain%20avalanche%20events.pdf).
The scenario is temporarily reduced to a 2D problem and the implementation can pass some simple tests

20.1 update: add controller module

21.1 update: add trajectory generation module

29.1 update: add simple sensor model

To run it, first build the package in the source directory:
```
catkin build
```
then open four terminals:

## Terminal 1: open the simulation environment

```
source devel/setup.bash
roslaunch unity_bridge unity_sim.launch
```

## Terminal 2: run the control module

```
source devel/setup.bash
roslaunch controller_pkg controller.launch
```

## Terminal 3: run the sensor model

```
source devel/setup.bash
roslaunch sensor_model sensor.launch
```

## Terminal 4: run the trajectory generation module

```
source devel/setup.bash
roslaunch basic_waypoint_pkg waypoint_mission.launch
```

# Final submission
## Presentation
The final presentation will take place on March 24th, 09:00 - 13:00.

## Deliverables

### Code

### Documentation
Carefully document how to set up and run your code as well as what happens where, why and coded by whom. 
Please keep the readability of your documentation in mind. 
Please use the official GitLab Wiki (find it in the left sidebar, very bottom) or READMEs.
Make sure to link your Wiki in your main README, so we find it. 

### Presentation
Please add a PDF version of your slides to the project folder of your GitLab before the final presentation. 

### Report
Please add a PDF and a Tex version of your final report to the project folder of your GitLab before the final presentation. 
You might want to adapt your .gitignore file to not commit LaTex compilation support files. 
Please make sure to submit your 4-page final report in [IEEE conference style](https://www.overleaf.com/latex/templates/ieee-conference-template/grfzhhncsfqn).
You can use [TUM Share LaTex](https://latex.tum.de/ldap/login) to collaborate. 


