#ifndef BASIC_WAYPOINT_PKG_PLANNER_H
#define BASIC_WAYPOINT_PKG_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>

class BasicPlanner {
public:
    BasicPlanner(ros::NodeHandle& nh);

    void onCurrentState(const nav_msgs::Odometry& cur_state);

    void setMaxSpeed(double max_v);

    void onSignal_intensity(const std_msgs::Float64MultiArray::ConstPtr& msg);

    void localsearch(const Eigen::Vector3d& startpoint, const int& i, const Eigen::Vector3d& beginpoint);

    void globalsearch();

    double mean_signal(const std::vector<double> &signal_cache);

    void interpolate(const Eigen::Vector3d& beginpoint, const Eigen::Vector3d& endpoint,Eigen::MatrixXd *middlepoint, const int& num);

    int max_signal_index();

    void display_victims();

    void remove_failures();

    double mean_signal_wo_failures(const std::vector<double> &signal_cache,  const std::vector<int> &failure_index);

    bool reachgoal(const Eigen::VectorXd& goal_pos);

    double distance2goal(const Eigen::VectorXd& goal_pos);

    bool publishState(const double &still , const double &velocity, const Eigen::Vector3d &startpoint, const Eigen::Vector3d &endpoint);

    Eigen::Vector3d potentialField(const double &alpha, const Eigen::Vector3d &x, const Eigen::Vector3d &endpoint);

private:

    ros::Publisher pub_state;
    ros::Subscriber current_state;
    ros::Subscriber signal_intensity;

    ros::NodeHandle& nh_;
    Eigen::Vector3d x;
    Eigen::Vector3d v; 
    Eigen::MatrixXd global_path;
    Eigen::Vector3d goal_position;
    Eigen::VectorXd signal;
    Eigen::Vector4d grid;
    Eigen::MatrixXd positions_of_victims;

    int num_victims;
    int angle;

    double depth;
    double error_rate;
    double distance_error;
    double sensor_range;
    double global_velocity;
    double local_velocity;
    double height;

    XmlRpc::XmlRpcValue position;
    XmlRpc::XmlRpcValue position_victims;

    std::vector<int> pred_victims_index;
    std::vector<double> pred_victims_error; 
    std::vector<double> pred_victims_error_distance; 
    std::vector<std::vector<double>> pred_victims;
    
    ros::Time start;

};

#endif // BASIC_WAYPOINT_PKG_PLANNER_H
