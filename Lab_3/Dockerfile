FROM ros:melodic

# install ros packages and required packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop=1.4.1-0* openssh-client python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

#source ROS
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'

#clone files
#set up the workdir in the image
WORKDIR /root


RUN mkdir /root/.ssh
RUN ssh-keyscan gitlab.lrz.de >> /root/.ssh/known_hosts
#clone our repo into the image(/root) using SSH
RUN --mount=type=ssh git clone git@gitlab.lrz.de:ga84sij/autonomous-systems-2021-group.git

#build ROS package
#cd to the ROS src directory
WORKDIR /root/autonomous-systems-2021-group/Lab_2/catkin_ws
RUN catkin config --extend /opt/ros/melodic
RUN catkin build
RUN /bin/bash -c 'source devel/setup.bash'
