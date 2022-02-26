/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include  "ros/ros.h"
#include <planner.h>

#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_planner");
    ros::NodeHandle n;
	
    BasicPlanner planner(n);  // instantiate basic planner
    ros::Duration(1.0).sleep();

    // define goal point
    Eigen::Vector3d goal_position, goal_velocity;
    goal_position << 0.0, 0.0, 2.0;
    goal_velocity << 0.0, 0.0, 0.0;

    // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
    ROS_WARN_STREAM("PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
    std::cin.get();
    for (int i = 0; i < 10; i++) {
        ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
    }

    mav_trajectory_generation::Trajectory trajectory;
    planner.planTrajectory(goal_position, goal_velocity, &trajectory);
    planner.publishTrajectory(trajectory);
    ROS_WARN_STREAM("DONE. GOODBYE.");

    return 0;
}