#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>

#define PI M_PI

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>


class sensorNode{
  ros::NodeHandle nh;
  ros::Subscriber  current_state;
  ros::Publisher sensor_signal;
  ros::Timer timer;
  XmlRpc::XmlRpcValue position;

  double range;
  double hz;  
  
  Eigen::Vector3d x;  
  Eigen::MatrixXd victims_config;

public:
  sensorNode():hz(100.0), range(20){
      nh.getParam(ros::this_node::getName() + "/victims/position", position);
      std::cout<<"position is "<<position.size()<<std::endl;
      victims_config.resize(position.size(),3);

      for(int n = 0; n < position.size(); n++){
        for(int j = 0; j < 3; j++){
          victims_config(n, j) = position[n][j];
        }
      }
      current_state = nh.subscribe("current_state", 1, &sensorNode::onCurrentState, this);
      timer = nh.createTimer(ros::Rate(hz), &sensorNode::computeLoop, this);
      sensor_signal = nh.advertise<std_msgs::Float64MultiArray>("sensor_signal", 1);
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      x << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
      // std::cout<<"current position"<<x<<std::endl;
  }

  void computeLoop(const ros::TimerEvent& t){

    Eigen::MatrixXd victims;
    victims = victims_config;
    // std::cout<<"victims before"<<victims<<std::endl;
    std_msgs::Float64MultiArray msg;
    victims.rowwise() -= x.transpose();
    std::cout<<"current postion is "<<x<<std::endl;
    // std::cout<<"victims"<<victims<<std::endl;

    Eigen::VectorXd distances = victims.rowwise().norm();
    std::vector<double> signal;

    for(int i = 0; i < distances.size(); i++){
      // std::cout<<"distance"<<distances(i)<<std::endl;
      double intensity = range/distances(i);
      // std::cout<<"intensity"<<intensity<<std::endl;
      if(intensity < 1) intensity = 0;
      signal.push_back(intensity);
    }
    msg.data = signal; 
    sensor_signal.publish(msg);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "sensor_node");
  ROS_INFO_NAMED("sensor", "sensor started!");
  sensorNode n;
  ros::spin();
}
