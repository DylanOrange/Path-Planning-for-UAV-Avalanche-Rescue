#include <planner.h>
#include <math.h>
//#include <yaml-cpp/yaml.h>

BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        max_v_(0.2),
        max_a_(0.2),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()),
        grid_width(10.0),
        sensor_range(20.0),
        num_victims(10)
        {

    goal_velocity << 0.0, 0.0, 0.0;
    // goal_position << -75.0, 0.0, 20.0;
    pred_victims_index.push_back(100);
    signal = Eigen::VectorXd::Zero(num_victims);


    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_v", max_v_)){
        ROS_WARN("[example_planner] param max_v not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_a", max_a_)){
        ROS_WARN("[example_planner] param max_a not found");
    }

    nh_.getParam(ros::this_node::getName() + "/waypoints/position", position);

    global_path.resize(3, position.size()); //all waypoints in global path
      for(int n = 0; n < position.size(); n++){
        for(int j = 0; j < 3; j++){
          global_path(j, n) = position[n][j];
        }
      }

    // create publisher for RVIZ markers
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

    // subscriber for Odometry
    sub_odom_ = nh.subscribe("odom", 1, &BasicPlanner::uavOdomCallback, this);
    current_state = nh.subscribe("current_state", 1, &BasicPlanner::onCurrentState, this);
    signal_intensity = nh.subscribe("sensor_signal", 1, &BasicPlanner::onSignal_intensity, this);
}

 void BasicPlanner::onCurrentState(const nav_msgs::Odometry& cur_state){

      x << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
      v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;
    //   std::cout<<"call back x is "<<x<<std::endl;
    //   omega << cur_state.twist.twist.angular.x,cur_state.twist.twist.angular.y,cur_state.twist.twist.angular.z;
    //   Eigen::Quaterniond q;
    //   tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
    //   R = q.toRotationMatrix();

    //   // Rotate omega
    //   omega = R.transpose()*omega;

  }

void BasicPlanner::onSignal_intensity(const std_msgs::Float64MultiArray::ConstPtr& msg){
    // ROS_INFO(" call back planner heard sensor signal!");
    for(int i = 0; i < msg->data.size(); i++){
    // ROS_INFO("[%f]", msg->data[i]);
    signal(i) = msg->data[i];
    }
    // for(int i = 0; i < signal.size(); i++){
    //     std::cout<<"callback receive signal"<<signal(i)<<std::endl;
    // }
  }

// Callback to get current Pose of UAV
void BasicPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);
    std::cout << "planner read current postion "<<current_pose_.translation() << std::endl;

    // store current velocity
    tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
}

bool BasicPlanner::reachgoal(const Eigen::VectorXd& goal_pos){

    // std::cout<<"reach current pose is "<<x(0)<<std::endl;
    // std::cout<<"reach goal pose is "<<goal_pos(0)<<std::endl;

    if(fabs(x(0)- goal_pos(0)) < 0.5 && fabs(x(1)- goal_pos(1))< 0.5 && fabs(x(2)- goal_pos(2)) < 0.5 ){
        std::cout<<"reach goal!! "<<std::endl;
        // ros::Duration(10.0).sleep();
        return true;
    }
    else{
        // std::cout<<"far from goal!! "<<std::endl;
        return false;
    }
}

void BasicPlanner::display_victims(){
    for (int i = 0; i < pred_victims.size(); i++){
        std::cout<<"find victims"<<std::endl;
        std::cout<<pred_victims[i][0]<<" "<<pred_victims[i][1]<<" "<<pred_victims[i][2]<<std::endl;
    }
}

double BasicPlanner::mean_signal(const std::vector<double> signal_cache){
	double sum = 0;
	for (int i = 0; i < signal_cache.size(); i++) sum += signal_cache[i];
	return sum/signal_cache.size();
}

void BasicPlanner::localsearch(const Eigen::Vector3d& startpoint, const int& i, const Eigen::Vector3d& beginpoint){
    double distance;
    int interpo_num = 3;
    std::vector<double> victim; 
    std::vector<double> signal_cache; 
    Eigen::MatrixXd local_waypoint, waypoints, local_waypoint_backup;
    Eigen::Vector3d endpoint, middlepoint;
    Eigen::Vector3d h_line, v_line;
    Eigen::Vector3d h_point1, h_point2, v_point1, v_point2;


    // h_line = (startpoint - global_path.col(last_waypoint));
    h_line = (startpoint - beginpoint);
    h_line.normalize();
    endpoint = startpoint + h_line*sensor_range*2.0;
    std::cout<<" h fly start point is"<<startpoint(0)<<" "<<startpoint(1)<<" "<<startpoint(2)<<std::endl;
    std::cout<<" h fly planning end point is"<<endpoint(0)<<" "<<endpoint(1)<<" "<<endpoint(2)<<std::endl;

    distance = (startpoint - endpoint).norm();
    std::cout<<" h fly planning distance is"<<distance<<std::endl;

    this->interpolate(startpoint, endpoint, &local_waypoint, interpo_num);
    this->planTrajectory(endpoint, goal_velocity, &trajectory, &local_waypoint);
    this->publishTrajectory(trajectory);

    std::cout<<" plan h finished! h-fly until the signal disappears!"<<std::endl;

    while(true){
        ros::spinOnce();
        if(signal(i) > 1e-3){;}
        // std::cout<<"h-fly until the signal disappears!"<<std::endl;}
        else break;
    }
    std::cout<<" h fly finished!"<<std::endl;

    h_point1 = x; // signal disappears, plan backward trajectory
    // h_line = (startpoint - h_point1);
    // h_line.normalize();

    std::cout<<" h fly real end point is"<<h_point1(0)<<" "<<h_point1(1)<<" "<<h_point1(2)<<std::endl;

    distance = (h_point1 - startpoint).norm();
    std::cout<<" h fly real distance is"<<distance<<std::endl;

    endpoint = h_point1 - h_line *sensor_range*2.0;
    std::cout<<" h backward fly planning end point is"<<endpoint(0)<<" "<<endpoint(1)<<" "<<endpoint(2)<<std::endl;

    distance = (h_point1 - endpoint).norm();
    std::cout<<" h backward fly planning distance is"<<distance<<std::endl;

    this->interpolate(h_point1, endpoint, &local_waypoint, interpo_num);
    this->planTrajectory(endpoint, goal_velocity, &trajectory, &local_waypoint);
    this->publishTrajectory(trajectory);

    std::cout<<" plan h backward finished! h-backward fly until the signal disappears!"<<std::endl;

    ros::Rate loop_rate(10);
    // ros::Duration(5.0).sleep();

    while(true){
        ros::spinOnce();
        loop_rate.sleep();
        signal_cache.push_back(signal(i));
        if(signal(i) < 1e-3){
            if(this->mean_signal(signal_cache) > 1e-3){  
                std::cout<<"have stored"<<signal_cache.size()<<"signal"<<std::endl;
                break;
            }
        }
    }
    signal_cache.clear();
    std::cout<<" h backward fly finished!"<<std::endl;

    h_point2 = x;
    middlepoint = (h_point1 + h_point2)/2;

    distance = (h_point2 - h_point1).norm();
    std::cout<<" h backward fly real distance is"<<distance<<std::endl;

    v_line<< -h_line(1), h_line(0), h_line(2);
    v_line.normalize();
    endpoint = middlepoint + v_line*sensor_range*2.0;

    this->interpolate(middlepoint, endpoint, &local_waypoint, interpo_num);
    this->interpolate(h_point2, middlepoint, &local_waypoint_backup, interpo_num);

    waypoints.resize(3, interpo_num*2-1);
    waypoints << local_waypoint_backup, middlepoint, local_waypoint; 

    std::cout<<" v fly start point is"<<h_point2(0)<<" "<<h_point2(1)<<" "<<h_point2(2)<<std::endl;
    std::cout<<" v fly middle point is"<<middlepoint(0)<<" "<<middlepoint(1)<<" "<<middlepoint(2)<<std::endl;

    distance = (h_point2 - middlepoint).norm();
    std::cout<<" v fly first distance is"<<distance<<std::endl;

    distance = (endpoint - middlepoint).norm();
    std::cout<<" v fly second distance is"<<distance<<std::endl;
    std::cout<<" v fly planning end point is"<<endpoint(0)<<" "<<endpoint(1)<<" "<<endpoint(2)<<std::endl;

    this->planTrajectory(endpoint, goal_velocity, &trajectory, &waypoints);
    this->publishTrajectory(trajectory);

    std::cout<<" plan v fly finished! v fly until the signal disappears!"<<std::endl;
    // ros::Duration(8.0).sleep();
    
    // while(true){
    //     ros::spinOnce();
    //     if(signal(i) > 1e-3){;}
    //     // std::cout<<"v fly until the signal disappears!"<<std::endl;}
    //     else break;
    // }

    ros::Rate second_loop_rate(10);

    while(true){
        ros::spinOnce();
        second_loop_rate.sleep();
        signal_cache.push_back(signal(i));
        if(signal(i) < 1e-3){
            if(this->mean_signal(signal_cache) > 1e-3){  
                std::cout<<"have stored"<<signal_cache.size()<<"signal"<<std::endl;
                break;
            }
        }
    }
    signal_cache.clear();

    std::cout<<" v fly finished!"<<std::endl;

    v_point1 = x;
    endpoint = middlepoint - sensor_range*v_line*2.0;

    std::cout<<" v backward fly start point is"<<v_point1(0)<<" "<<v_point1(1)<<" "<<v_point1(2)<<std::endl;
    std::cout<<" v backward fly planning end point is"<<endpoint(0)<<" "<<endpoint(1)<<" "<<endpoint(2)<<std::endl;
    distance = (v_point1 - endpoint).norm();
    std::cout<<" v backward fly planning distance is"<<distance<<std::endl;

    this->interpolate(v_point1, endpoint, &local_waypoint, interpo_num);
    this->planTrajectory(endpoint, goal_velocity, &trajectory, &local_waypoint);
    this->publishTrajectory(trajectory);
    std::cout<<" plan v backward fly finished! v backward fly until the signal disappears!"<<std::endl;

    // ros::Duration(10.0).sleep();
    // while(true){
    //     ros::spinOnce();
    //     if(signal(i) > 1e-3){;}
    //     // std::cout<<"v -backward fly until the signal disappears!"<<std::endl;}
    //     else break;
    // }

    ros::Rate third_loop_rate(10);

    while(true){
        ros::spinOnce();
        third_loop_rate.sleep();
        signal_cache.push_back(signal(i));
        if(signal(i) < 1e-3){
            if(this->mean_signal(signal_cache) > 1e-3){  
                std::cout<<"have stored"<<signal_cache.size()<<"signal"<<std::endl;
                break;
            }
        }
    }
    signal_cache.clear();

    std::cout<<" v backward fly finished!"<<std::endl;

    v_point2 = x;
    std::cout<<" v backward fly real end point is"<<v_point2(0)<<" "<<v_point2(1)<<" "<<v_point2(2)<<std::endl;

    for(int index = 0; index < v_point2.size(); index++){
        victim.push_back((v_point1(index) + v_point2(index))/2);
    }
    pred_victims.push_back(victim);
    pred_victims_index.push_back(i);
    std::cout<<"find"<< i+1 << "victim! "<< victim[0]<<" "<<victim[1]<<" "<<victim[2]<<std::endl;

    this->planTrajectory(middlepoint, goal_velocity, &trajectory, NULL);
    this->publishTrajectory(trajectory);
    std::cout<<" plan finished,fly back to the middle point!"<<std::endl;

    while(true){
        ros::spinOnce();
        if(!this->reachgoal(middlepoint)){;}
        // std::cout<<" fly back to the middle point!"<<std::endl;}
        else break;
    }
    std::cout<<" fly back to the middle point, local search finished!"<<std::endl;
}

// void BasicPlanner::globalsearch(){
//     this->planTrajectory(goal_position, goal_velocity, &trajectory, &global_path);
//     this->publishTrajectory(trajectory);
//     std::cout<<"global search begins!"<<std::endl;

//     int maxsignal;
//     while(true){

//         while(true){
//             ros::spinOnce();
//             // std::cout<<"current position x is "<<x<<std::endl;
//             maxsignal = this->max_signal();
//             if(signal(maxsignal)>0) {
//                 std::cout<<"received signal"<< signal(maxsignal)<<std::endl;
//                 break;}
//             // else std::cout<<"there is no victim nearby!"<<std::endl;
//         }

//         int next_waypoint;
//         Eigen::Vector3d startpoint;
//         Eigen::MatrixXd waypoints;

//         startpoint = x;
//         std::cout<<"looking for " << maxsignal+1 <<"th victims!"<<std::endl;
//         std::cout<<"start point is " << startpoint <<std::endl;

//         next_waypoint = this->next_waypoint(startpoint);
//         std::cout<<"next_waypoint's index is " << next_waypoint <<std::endl;
//         this->localsearch(startpoint, maxsignal, next_waypoint-1);
//         waypoints = global_path.rightCols(global_path.cols() - next_waypoint);
//         this->planTrajectory(goal_position, goal_velocity, &trajectory, &waypoints);
//         this->publishTrajectory(trajectory);
//         std::cout<<"global search continues!"<<std::endl;

//         if(this->reachgoal(goal_position)) break;
//     }
// }

void BasicPlanner::globalsearch(){
    int interpolater_num = 5;
    int maxsignal;
    Eigen::Vector3d global_goal_position, beginpoint;
    Eigen::MatrixXd local_waypoint;
    for(int i = 0; i < global_path.cols(); i++){

        global_goal_position = global_path.col(i);
        if(i == 0){
            beginpoint = x;
            this->interpolate(beginpoint, global_goal_position, &local_waypoint, 3);
            this->planTrajectory(global_goal_position, goal_velocity, &trajectory, &local_waypoint);
            this->publishTrajectory(trajectory);
            while(true){
                ros::spinOnce();
                if(!this->reachgoal(global_goal_position)){;}
                else break;
            }
        }
        else{
            beginpoint = global_path.col(i-1);
            this->interpolate(beginpoint, global_goal_position, &local_waypoint, interpolater_num);
            this->planTrajectory(global_goal_position, goal_velocity, &trajectory, &local_waypoint);
            this->publishTrajectory(trajectory);
        }

        while(true){
            ros::spinOnce();
            maxsignal = this->max_signal_index();
            // std::cout<<"maxsignal index is "<<maxsignal<<std::endl;
            if(maxsignal == -1){;}
            //     std::cout<<"you have already found all victims!"<<std::endl;
            // }
            else if(signal(maxsignal)>0) {
                std::cout<<"received signal"<< signal(maxsignal)<<std::endl;
                // int next_waypoint;
                Eigen::Vector3d search_point, begin_point;
                // Eigen::MatrixXd waypoints;

                search_point = x;
                std::cout<<"looking for " << maxsignal+1 <<"th victims!"<<std::endl;
                std::cout<<"start point is " << search_point <<std::endl;

                // next_waypoint = this->next_waypoint(startpoint);
                // std::cout<<"next_waypoint's index is " << next_waypoint <<std::endl;
                this->localsearch(search_point, maxsignal, beginpoint);
                // waypoints = global_path.rightCols(global_path.cols() - next_waypoint);

                ros::spinOnce();
                this->interpolate(x, global_goal_position, &local_waypoint, 3);
                this->planTrajectory(global_goal_position, goal_velocity, &trajectory, &local_waypoint);
                this->publishTrajectory(trajectory);
                std::cout<<"global search continues!"<<std::endl;
            }

            if(!this->reachgoal(global_goal_position)){;}
            // std::cout<<" fly back to the middle point!"<<std::endl;}
            else break;
        }
        std::cout<<"reach local waypoint "<<global_goal_position(0)<<global_goal_position(1)<<global_goal_position(2)<<std::endl;
     }
}

int BasicPlanner::max_signal_index(){
    double max = 0;
    int maxindex = 0;
    for (int i = 0; i < signal.size(); i++){
        // std::cout<<"signal " <<i<<"th is"<< signal(i)<<std::endl;
        for (int j = 0; j < pred_victims_index.size(); j++){
            if(pred_victims_index[j] == i) break;
            else if (j == pred_victims_index.size()-1){
                if(max<signal(i)){
                    max = signal(i);
                    std::cout<<"max signal is " <<max<<std::endl;
                    maxindex = i;
                    std::cout<<"max signal index is " <<maxindex<<std::endl;
                }
            }
        }
        if( i == signal.size()-1){
            if(max == 0) return -1;
            else return maxindex;
        }
    }
    // std::cout<<"max signal is " <<max<<std::endl;
}

void BasicPlanner::interpolate(const Eigen::Vector3d& beginpoint, const Eigen::Vector3d& endpoint,Eigen::MatrixXd *middlepoint, const int& num){
    Eigen::Vector3d line, interval;
    middlepoint->resize(3,num-1);
    double distance;
    line = endpoint - beginpoint;
    distance = line.norm();
    line.normalize();
    interval = (distance / num) * line;
    for(int i = 1; i < num; i++){
        middlepoint->col(i-1) = beginpoint + i*interval;
    }
}


// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool BasicPlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    mav_trajectory_generation::Trajectory* trajectory,
                                    const Eigen::MatrixXd *way_points = NULL) {

    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 3;

    // std::cout<<"planner start pose is "<<x<<std::endl;
    // Array for all waypoints and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::SNAP;

    // we have 2 vertices:
    // Start = current position
    // end = desired position and velocity
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);


    /******* Configure start point *******/
    // set start point constraints to current position and set all derivatives to zero
    start.makeStartOrEnd(x,
                         derivative_to_optimize);


    // set start point's velocity to be constrained to current velocity
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        v);

    // add waypoint to list
    vertices.push_back(start);

    // XmlRpc::XmlRpcValue position, velocity, acceleration;

    // nh_.getParam(ros::this_node::getName() + "/waypoints/position", position);
    // nh_.getParam(ros::this_node::getName() + "/waypoints/velocity", velocity);
    // nh_.getParam(ros::this_node::getName() + "/waypoints/acceleraiton", acceleration);
    // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(position[0][0],position[0][1],position[0][2]));
    // vertices.push_back(middle);
    // middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
    // std::cout << "planner  read "<<position[0] << std::endl;
    if (way_points != NULL){
        Eigen::MatrixXd middlepoint = *way_points;
        for(int i=0; i<way_points->cols(); i++){
            // if(loop == 0 && i == 8) i = i + 1;
            middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(middlepoint(0,i), middlepoint(1,i), middlepoint(2,i)));
            // std::cout << "waypoints: " << middlepoint(0,i) << middlepoint(1,i) << middlepoint(2,i)<<std::endl;
            // if(loop == 1 && i == 8)
            // {
            //     middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(velocity[i][0],velocity[i][1],velocity[i][2]));
            //     middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(acceleration[i][0],acceleration[i][1],acceleration[i][2]));
            // }
            vertices.push_back(middle);
            middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
            // if(loop == 1 && i == 8)
            // {
            //     middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
            //     middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
            // }
        }
    }

    /******* Configure end point *******/
    // set end point constraints to desired position and set all derivatives to zero
    end.makeStartOrEnd(goal_pos,
                       derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);

    // add waypoint to list
    vertices.push_back(end);

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));

    return true;
}

bool BasicPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 0.2; // Distance by which to separate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "world";
    pub_trajectory_.publish(msg);

    return true;
}
