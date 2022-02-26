#ifndef BASIC_WAYPOINT_PKG_PLANNER_H
#define BASIC_WAYPOINT_PKG_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

class BasicPlanner {
public:
    BasicPlanner(ros::NodeHandle& nh);

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

    void onCurrentState(const nav_msgs::Odometry& cur_state);

    void setMaxSpeed(double max_v);

    void onSignal_intensity(const std_msgs::Float64MultiArray::ConstPtr& msg);

    void localsearch(const Eigen::Vector3d& startpoint, const int& i, const Eigen::Vector3d& beginpoint);

    void globalsearch();

    double mean_signal(const std::vector<double> signal_cache);

    void interpolate(const Eigen::Vector3d& beginpoint, const Eigen::Vector3d& endpoint,Eigen::MatrixXd *middlepoint, const int& num);

    int max_signal_index();

    void display_victims();

    bool reachgoal(const Eigen::VectorXd& goal_pos);

    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        mav_trajectory_generation::Trajectory* trajectory,
                        const Eigen::MatrixXd *way_points);

    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        const Eigen::VectorXd& start_pos,
                        const Eigen::VectorXd& start_vel,
                        double v_max, double a_max,
                        mav_trajectory_generation::Trajectory* trajectory);

    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

private:
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;
    ros::Subscriber sub_odom_;
    ros::Subscriber current_state;
    ros::Subscriber signal_intensity;

    ros::NodeHandle& nh_;
    Eigen::Vector3d x;
    Eigen::Vector3d v; 
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;
    Eigen::Vector3d goal_velocity;
    Eigen::MatrixXd global_path;
    Eigen::Vector3d goal_position;
    Eigen::MatrixXd found_victims;
    Eigen::VectorXd signal;
    mav_trajectory_generation::Trajectory trajectory;
    std::vector<std::vector<double>> pred_victims;
    // std::vector<double> signal;
    std::vector<int> pred_victims_index;
    double max_v_; // m/s
    double max_a_; // m/s^2
    double max_ang_v_;
    double max_ang_a_;
    double grid_width;
    double sensor_range;
    int num_victims;
    XmlRpc::XmlRpcValue position;

};

#endif // BASIC_WAYPOINT_PKG_PLANNER_H
