#include  "ros/ros.h"
#include <planner.h>

#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_planner");
    ros::NodeHandle n;
	
    BasicPlanner planner(n);  // instantiate basic planner

    // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
    ROS_WARN_STREAM("PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
    std::cin.get();
    for (int i = 0; i < 10; i++) {
        ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
    }

    planner.globalsearch();
    planner.display_victims();

    ROS_WARN_STREAM("global trip. done.");

    ros::spin();

    return 0;
}