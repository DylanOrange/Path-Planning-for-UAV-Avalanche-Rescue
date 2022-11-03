#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <Eigen/Dense>
#include <sstream>
#include <iostream>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <math.h>

#define PI M_PI
#define TFOUTPUT 1

class trajPublish{

        ros::NodeHandle nh;
        ros::Subscriber  received_state;
        ros::Publisher desired_state_pub;
        ros::Timer timer;
        ros::Time start;

        double hz, state_signal, v;
        Eigen::Vector3d start_point, end_point, direction;
        tf::TransformBroadcaster br;

public:
        trajPublish():hz(2000.0){
                desired_state_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
                received_state = nh.subscribe("state_signal", 1, &trajPublish::statecallback, this);
                timer = nh.createTimer(ros::Rate(hz), &trajPublish::computeLoop, this);
        }

        void statecallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
                for(int i = 0; i<msg->data.size(); i++){
                        std::cout<<" publisher hears"<<msg->data[i]<<" "<<std::endl;
                }
                state_signal = msg->data[0];
                v = msg->data[1];
                start_point << msg->data[2], msg->data[3], msg->data[4];
                end_point << msg->data[5], msg->data[6], msg->data[7];
                if (state_signal == 0){
                std::cout<<" begin timer!"<<std::endl;
                start = ros::Time::now();
                }
        }

        void computeLoop(const ros::TimerEvent& t){
                tf::Vector3 origin(0,0,0);
                // std::cout<<" start time is "<<start<<std::endl;
                // std::cout<<" current time is "<<ros::Time::now()<<std::endl;

                double time_scale = (ros::Time::now()-start).toSec();
                // std::cout<<" time_scale is "<<time_scale<<std::endl;

                // Quantities to fill in
                tf::Transform desired_pose(tf::Transform::getIdentity());

                geometry_msgs::Twist velocity;
                velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
                velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
                geometry_msgs::Twist acceleration;
                acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
                acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;

                if (state_signal == 1){
                // Static Pose
                        tf::Vector3 displacement(start_point(0), start_point(1), start_point(2));
                        desired_pose.setOrigin(origin+displacement);
                        tf::Quaternion q;
                        q.setRPY(0,0,PI/4);
                        // count++;
                        // std::cout<<"Desired Orientation" << count << std::endl;
                        desired_pose.setRotation(q);
                }
                else{
                        // std::cout<<" fly to target point! "<<std::endl;
                        Eigen::Vector3d direction = end_point - start_point;
                        direction.normalize();
                        // double R = 5.0;
                        // double timeScale = 2.0;
                        desired_pose.setOrigin(
                                        tf::Vector3(start_point(0), start_point(1), start_point(2)) + tf::Vector3(v*direction[0]*time_scale, v*direction[1]*time_scale, v*direction[2]*time_scale)
                                );
                        tf::Quaternion q;
                        q.setRPY(0,0,PI/4);
                        desired_pose.setRotation(q);
                        velocity.linear.x = v*direction[0];
                        velocity.linear.y = v*direction[1];
                        velocity.linear.z = v*direction[2];

                        velocity.angular.x = 0.0;
                        velocity.angular.y = 0.0;
                        velocity.angular.z = 0.0;

                        acceleration.linear.x = 0;
                        acceleration.linear.y = 0;
                        acceleration.linear.z = 0;
                 }

                trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
                msg.transforms.resize(1);
                msg.transforms[0].translation.x = desired_pose.getOrigin().x();
                msg.transforms[0].translation.y = desired_pose.getOrigin().y();
                msg.transforms[0].translation.z = desired_pose.getOrigin().z();
                msg.transforms[0].rotation.x = desired_pose.getRotation().getX();
                msg.transforms[0].rotation.y = desired_pose.getRotation().getY();
                msg.transforms[0].rotation.z = desired_pose.getRotation().getZ();
                msg.transforms[0].rotation.w = desired_pose.getRotation().getW();
                msg.velocities.resize(1);
                msg.velocities[0] = velocity;
                msg.accelerations.resize(1);
                msg.accelerations[0] = acceleration;
                desired_state_pub.publish(msg);

                br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
                                                "world", "av-desired"));
        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    trajPublish n;
    ros::spin();
}
