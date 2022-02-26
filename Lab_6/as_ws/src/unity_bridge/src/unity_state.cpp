#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include "TCPStreamReader.h"

#define TFOUTPUT 1
class StateServer
{
  public:
    StateServer(std::string const& ip_address, std::string const& port) 
    :  nh("~"), stream_reader(ip_address, port) {
      InitializePublishers();
    }

    void Connect() {
      ROS_INFO("[unity_state] Waiting for connection...");
      stream_reader.WaitConnect();
      ROS_INFO("[unity_state] Got a connection...");      
    }

    void PublishStateStream() {
      if(!stream_reader.Good()) {
        ROS_ERROR("[unity_state] state server connection broken");
        exit(1);
      }
    
      try {
        auto data = stream_reader.ReadBytes(13 * sizeof(float));
        float* begin = reinterpret_cast<float*>(data.get());

        // NOTE Unity is left-handed so we must:
        //     - for position: flip y and z
        //     - for quaternions:
        //         - flip y and z
        //         - flip all signs in the vector part
        //     - for angular velocities:
        //         - flip y and z
        //         - flip all signs (pseudo-vector)

        nav_msgs::Odometry state_msg;
        state_msg.header.stamp = ros::Time::now();
        state_msg.header.frame_id = "world";
        state_msg.child_frame_id = "av";
        state_msg.pose.pose.position.x = begin[0];
        state_msg.pose.pose.position.y = begin[2];
        state_msg.pose.pose.position.z = begin[1];
        state_msg.pose.pose.orientation.x = -begin[3];
        state_msg.pose.pose.orientation.y = -begin[5];
        state_msg.pose.pose.orientation.z = -begin[4];
        state_msg.pose.pose.orientation.w = begin[6];
        // Ignoring covariance
        state_msg.twist.twist.linear.x = begin[7];
        state_msg.twist.twist.linear.y = begin[9];
        state_msg.twist.twist.linear.z = begin[8];
        state_msg.twist.twist.angular.x = -begin[10];
        state_msg.twist.twist.angular.y = -begin[12];
        state_msg.twist.twist.angular.z = -begin[11];
        pose_pub.publish(state_msg);

#if TFOUTPUT
        tf::Transform pose;
        tf::poseMsgToTF(state_msg.pose.pose, pose);
        br.sendTransform(tf::StampedTransform(pose, ros::Time::now(), "world", "av"));
#endif
        
      } catch(std::exception & ex) {
        ROS_ERROR("Shutting down state server");
        stream_reader.Shutdown();
      }
    }

  private:

  void InitializePublishers() {
    pose_pub = nh.advertise<nav_msgs::Odometry>("/current_state", 10);
  }

  ros::NodeHandle nh;
  ros::Publisher pose_pub;
  ros::Publisher twist_pub;
  TCPStreamReader stream_reader;
#if TFOUTPUT
  tf::TransformBroadcaster br;
#endif
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "current_state");
  ros::NodeHandle nh;
  
  std::string ip_address, port;
  port="12347";
  ip_address="127.0.0.1";
    
  StateServer unity_state_server(ip_address, port);
  unity_state_server.Connect();

  while (ros::ok()) {    
    unity_state_server.PublishStateStream();
    ros::spinOnce();
  }

  return 0;
}
