 #include <ros/ros.h>

 #include <ros/console.h>
 #include <tf/transform_listener.h>
 #include <tf/transform_datatypes.h>
 #include <geometry_msgs/Point.h>
 #include <mav_msgs/Actuators.h>
 #include <nav_msgs/Odometry.h>
 #include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
 #include <math.h>
 #include <eigen3/Eigen/Dense>
 #include <eigen_conversions/eigen_msg.h>

 #define PI M_PI

 class controllerNode{
   ros::NodeHandle nh;
   ros::Subscriber uav_d;    //desired state subscriber
   ros::Subscriber uav_now;  //current state subscriber
   ros::Publisher rotor_spd; //propeller speed publisher
   ros::Timer ctrl_timer;    //timer for main control loop

   ros::Time start;

   // Controller parameters
   double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

   double m;              // mass of the UAV
   double g;              // gravity acceleration
   double d;              // distance from the center of propellers to the c.o.m.
   double cf,             // Propeller lift coefficient
          cd;             // Propeller drag coefficient
   Eigen::Matrix3d J;     // Inertia Matrix
   Eigen::Vector3d e3;    // [0,0,1]
   Eigen::MatrixXd F2W;   // Wrench-rotor speeds map

   // Current state
   Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
   Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
   Eigen::Matrix3d R;     // current orientation of the UAV
   Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

   // Desired state
   Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
   Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
   Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
   double yawd;           // desired yaw angle

   double hz;             // frequency of the main control loop


   static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
     Eigen::Vector3d out;
     out << in(2,1), in(0,2), in(1,0);
     return out;
   }

   static double signed_sqrt(double val){
     return val>0?sqrt(val):-sqrt(-val);
   }

 public:
   controllerNode():e3(0,0,1),F2W(4,4),hz(1000.0){
       //bind the nodes to the topics
       uav_d = nh.subscribe("desired_state", 100, &controllerNode::onDesiredState, this);
       uav_now = nh.subscribe("current_state", 100, &controllerNode::onCurrentState, this);
       //binding to created timer
       start = ros::Time::now();
       ctrl_timer = nh.createTimer(ros::Duration(1/hz), &controllerNode::controlLoop, this);
       rotor_spd = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);

       // Controller gains
       kx = 7.0;
       kv = 7.0;
       kr = 7.0;
       komega = 1.5;

       // kx = 5.5;
       // kv = 4.5;
       // kr = 5;
       // komega = 1.1;

       // Initialize constants
       m = 1.0;
       cd = 1e-5;
       cf = 1e-3;
       g = 9.81;
       d = 0.3;
       J << 0.1,0.0,0.0,0.0,0.1,0.0,0.0,0.0,0.2;
       ROS_INFO("[controller_node] Controller initialised");;
   }

   void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state){
       //desired position
       xd << des_state.transforms[0].translation.x, des_state.transforms[0].translation.y, des_state.transforms[0].translation.z;
       //desired velocity
       vd << des_state.velocities[0].linear.x, des_state.velocities[0].linear.y, des_state.velocities[0].linear.z;
       //desired acceleration
       ad << des_state.accelerations[0].linear.x, des_state.accelerations[0].linear.y, des_state.accelerations[0].linear.z;
       //desired yaw from quaternion instance
       yawd = tf::getYaw(des_state.transforms[0].rotation);
   }

   void onCurrentState(const nav_msgs::Odometry& cur_state){
       //current position
       x << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
       //current velocity
       v << cur_state.twist.twist.linear.x, cur_state.twist.twist.linear.y, cur_state.twist.twist.linear.z;
       //current angular velocity
       omega << cur_state.twist.twist.angular.x, cur_state.twist.twist.angular.y, cur_state.twist.twist.angular.z;
       //current orientation
       Eigen::Quaterniond q_rot(cur_state.pose.pose.orientation.w, cur_state.pose.pose.orientation.x, cur_state.pose.pose.orientation.y, cur_state.pose.pose.orientation.z);
       //initialize the rotation matrix from quaternion
       R = q_rot.normalized().toRotationMatrix();
       //transform omega from world to body frame and overwrite
       omega = R.transpose()*omega;

   }

   void controlLoop(const ros::TimerEvent& t){
     Eigen::Vector3d ex, ev, er, eomega;

     // Compute position and velocity errors. Objective: fill in ex, ev.
     ex = x - xd;
     ev = v - vd;

     //b3d corrected for ros application

     std::cout << "current x  is: " << x << std::endl;
     std::cout << "desired xd  is: " << xd << std::endl;
     std::cout << "current v  is: " << v << std::endl;
     std::cout << "desired vd  is: " << vd << std::endl;

     std::cout << "ex  is: " << ex << std::endl;
     std::cout << "ev  is: " << ev << std::endl;
     Eigen::Vector3d b3d (-kx*ex - kv*ev + m*g*e3 + m*ad); //((-kx*ex - kv*ev + m*g*e3 + m*ad).norm());
     b3d.normalize();

     //assumption b1d not parallel to b3d
     Eigen::Vector3d b1d (1,0,0);
     b1d.normalize();

     //b2d is the normalized cross product of b1d and b3d
     Eigen::Vector3d b2d = b3d.cross(b1d);
     b2d.normalize();

     //calculate b1d and normalize analog to b2d
     Eigen::Vector3d b1d_again(b2d.cross(b3d));
     b1d_again.normalize();

     //assemble the matrix
     Eigen::Matrix3d Rd;
     Rd << b1d_again, b2d, b3d;

     //error of orientation
     er = 0.5*Vee((Rd.transpose()*R - R.transpose()*Rd));

     //truncated error of rotation rate
     eomega = omega;

     //control inputs
     double f = ((-kx*ex - kv*ev + m*g*e3 + m*ad).transpose())*(R*e3);

     //std::cout << f;

     Eigen::Vector3d M;
     M = (-kr*er - komega*eomega + omega.cross(J*omega));

     // Recover the rotor speeds from the wrench computed above
     Eigen::Matrix4d RHS;
     RHS << 1, 1, 1, 1,
       (d/sqrt(2)), (d/sqrt(2)), -(d/sqrt(2)), -(d/sqrt(2)),
       -(d/sqrt(2)), (d/sqrt(2)), (d/sqrt(2)), -(d/sqrt(2)),
       cd/cf, -cd/cf, cd/cf, -cd/cf;
     RHS = RHS*cf;

     Eigen::Vector4d LHS;
     LHS << f, M(0), M(1), M(2);

     Eigen::Vector4d F;
     F = RHS.inverse()*LHS;

     std::cout <<"matrxi F is"<< F <<std::endl;

     float one = signed_sqrt(F(0));
     float two = signed_sqrt(F(1));
     float three = signed_sqrt(F(2));
     float four = signed_sqrt(F(3));

     mav_msgs::Actuators msg;
     msg.angular_velocities.resize(4);
     msg.angular_velocities[0] = one;
     msg.angular_velocities[1] = two;
     msg.angular_velocities[2] = three;
     msg.angular_velocities[3] = four;
     std::cout << "controller cmd1 is: " << one << std::endl;
     std::cout << "controller cmd2 is: " << two << std::endl;
     std::cout << "controller cmd3 is: " << three << std::endl;
     std::cout << "controller cmd4 is: " << four << std::endl;

     rotor_spd.publish(msg);
   }
 };

 int main(int argc, char** argv){
   ros::init(argc, argv, "controller_node");
   controllerNode n;
   ros::spin();
 }
