#include <planner.h>
#include <math.h>
//#include <yaml-cpp/yaml.h>

#define PI 3.1415926

BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        sensor_range(20.0),
        num_victims(10),
        angle(7),
        depth(5.0),
        global_velocity(8.0),
        local_velocity(5.0),
        height(20.0)
        {
    
    grid<<0.0, 0.0, -385.0, 300.0;
    pred_victims_index.push_back(100);
    signal = Eigen::VectorXd::Zero(num_victims);

    nh_.getParam(ros::this_node::getName() + "/waypoints/position", position);

    global_path.resize(3, position.size()); //all waypoints in global path
      for(int n = 0; n < position.size(); n++){
        for(int j = 0; j < 3; j++){
          global_path(j, n) = position[n][j];
        }
      }

    nh.getParam(ros::this_node::getName() + "/victims/position", position_victims);
    positions_of_victims.resize(position_victims.size(),3);

    for(int n = 0; n < position_victims.size(); n++){
      for(int j = 0; j < 3; j++){
        positions_of_victims(n, j) = position_victims[n][j];
      }
    }

    pub_state = nh.advertise<std_msgs::Float64MultiArray>("state_signal", 1);
    current_state = nh.subscribe("current_state", 1, &BasicPlanner::onCurrentState, this);
    signal_intensity = nh.subscribe("sensor_signal", 1, &BasicPlanner::onSignal_intensity, this);
}

 void BasicPlanner::onCurrentState(const nav_msgs::Odometry& cur_state){

      x << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
      v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;

  }

void BasicPlanner::onSignal_intensity(const std_msgs::Float64MultiArray::ConstPtr& msg){
    for(int i = 0; i < msg->data.size(); i++){
    signal(i) = msg->data[i];
    }
  }

double BasicPlanner::distance2goal(const Eigen::VectorXd& goal_pos){
    double distance;
    distance = (x - goal_pos).norm();
    return distance;
}

bool BasicPlanner::reachgoal(const Eigen::VectorXd& goal_pos){
    double distance = 0;
    double still;
    double distance_replan;
    std::vector<double> distance_cache; 
    ros::Rate loop_rate(10);
    while(true){
        ros::spinOnce();
        loop_rate.sleep();
        distance = this->distance2goal(goal_pos);
        // std::cout<<"distance is  "<<distance<<std::endl;
        distance_cache.push_back(distance);
        if(distance < 0.5){
            std::cout<<"reach goal!! "<<std::endl;
            break;
        }
        if(this->mean_signal(distance_cache) >= distance){;}
        else{
            std::cout<<"away from goal!! "<<std::endl;
            still = 0;
            std::cout<<"replan trajectory! "<<std::endl;
            this->publishState(still, local_velocity, x, goal_pos); 
            if(this->reachgoal(goal_pos)) break;
        }                    
    }
    distance_cache.clear();
    return true;
}

Eigen::Vector3d BasicPlanner::potentialField(const double &alpha, const Eigen::Vector3d &x, const Eigen::Vector3d &endpoint){
        Eigen::Vector3d dU, x_next;
        dU = x - endpoint;
        x_next = x - alpha*dU;
        return x_next;
}

void BasicPlanner::display_victims(){
    double time = (ros::Time::now()-start).toSec();
    for (int i = 0; i < pred_victims.size(); i++){
        std::cout<<"find victims"<<std::endl;
        std::cout<<pred_victims[i][0]<<" "<<pred_victims[i][1]<<" "<<pred_victims[i][2]<<std::endl;
        std::cout<<"The error rate of this victim is "<<pred_victims_error[i]<<"%"<<std::endl;
        std::cout<<"The distance error of this victim is "<<pred_victims_error_distance[i]<<"m"<<std::endl;
    }
    std::cout<<"find"<<pred_victims.size()<<" victims"<<std::endl;
    std::cout<<"The mean error rate of all victims is "<<mean_signal(pred_victims_error)<<"%"<<std::endl;
    std::cout<<"The mean distance error of all victims is "<<mean_signal(pred_victims_error_distance)<<"m"<<std::endl;  
    this->remove_failures();
    std::cout<<"The whole process costs "<<time<<" s"<<std::endl;  
}

void BasicPlanner::remove_failures(){
    std::vector<int> failure_index;
    for (int i = 0; i < pred_victims.size(); i++){
        if(pred_victims_error_distance[i] > 5.0){
            std::cout<<i+1<<" th case doesn't work "<<std::endl;
            failure_index.push_back(i);
        }
    }
    std::cout<<"The mean error rate without failure cases is "<<mean_signal_wo_failures(pred_victims_error, failure_index)<<"%"<<std::endl;
    std::cout<<"The mean distance error without failure cases is "<<mean_signal_wo_failures(pred_victims_error_distance, failure_index)<<"m"<<std::endl;  
}

double BasicPlanner::mean_signal(const std::vector<double> &signal_cache){
	double sum = 0;
	for (int i = 0; i < signal_cache.size(); i++) sum += signal_cache[i];
	return sum/signal_cache.size();
}

double BasicPlanner::mean_signal_wo_failures(const std::vector<double> &signal_cache,  const std::vector<int> &failure_index){
    if(failure_index.size() > 0){
        double sum = 0;
        for (int i = 0; i < signal_cache.size(); i++){
            for(int j = 0; j < failure_index.size(); j++){
                if(i == failure_index[j]){
                    sum += 0;
                    break;
                }
                if(j == (failure_index.size() - 1)){
                    sum += signal_cache[i];
                }
            }
        }
        return sum/(signal_cache.size() - failure_index.size());
    }
    else{
        return mean_signal(signal_cache);
    }
}

void BasicPlanner::localsearch(const Eigen::Vector3d& startpoint, const int& i, const Eigen::Vector3d& beginpoint){
    double distance;
    double still;
    std::vector<double> victim; 
    std::vector<double> signal_cache; 
    Eigen::Vector3d endpoint, middlepoint, victim_position, plane_point, plane_vector;
    Eigen::Vector3d h_line, v_line;
    Eigen::Vector3d h_point1, h_point2, v_point1, v_point2;
    Eigen::Vector3d x_prev, x_next;
    plane_point<<0,0,height - depth;
    plane_vector<<sin(angle *PI/180.0), 0.0, cos(angle *PI/180.0);

    h_line = (startpoint - beginpoint);
    h_line.normalize();
    endpoint = startpoint + h_line*sensor_range*2.0;

    still = 0.0;
    this->publishState(still, global_velocity, startpoint, endpoint);
    distance = (startpoint - endpoint).norm();

    std::cout<<" h fly start point is"<<startpoint(0)<<" "<<startpoint(1)<<" "<<startpoint(2)<<std::endl;
    std::cout<<" h fly planning end point is"<<endpoint(0)<<" "<<endpoint(1)<<" "<<endpoint(2)<<std::endl;
    std::cout<<" plan h finished! h-fly until the signal disappears!"<<std::endl;

    while(true){
        ros::spinOnce();
        if(signal(i) > 1e-3){;}
        else break;
    }
    std::cout<<" h fly finished!"<<std::endl;

    h_point1 = x; // signal disappears, plan backward trajectory
    distance = (h_point1 - startpoint).norm();

    std::cout<<" h fly real end point is"<<h_point1(0)<<" "<<h_point1(1)<<" "<<h_point1(2)<<std::endl;
    std::cout<<" h fly real distance is"<<distance<<std::endl;

    // endpoint = h_point1 - h_line *sensor_range*2.0;
    // distance = (h_point1 - endpoint).norm();

    // still = 0.0;
    // this->publishState(still, global_velocity, h_point1, endpoint);

    // std::cout<<" h backward fly planning end point is"<<endpoint(0)<<" "<<endpoint(1)<<" "<<endpoint(2)<<std::endl;
    // std::cout<<" plan h backward finished! h-backward fly until the signal disappears!"<<std::endl;

    // ros::Rate loop_rate(50);

    // while(true){
    //     ros::spinOnce();
    //     loop_rate.sleep();
    //     signal_cache.push_back(signal(i));
    //     if(signal(i) < 1e-3){
    //         if(this->mean_signal(signal_cache) > 1e-3){  
    //             std::cout<<"have stored"<<signal_cache.size()<<"signal"<<std::endl;
    //             break;
    //         }
    //     }
    // }
    // signal_cache.clear();
    // std::cout<<" h backward fly finished!"<<std::endl;

    // ros::spinOnce();
    h_point2 = startpoint;
    middlepoint = (h_point1 + h_point2)/2;
    // std::cout<<" h backward fly real end point is"<<h_point2(0)<<" "<<h_point2(1)<<" "<<h_point2(2)<<std::endl;
    std::cout<<" middle point is"<<middlepoint(0)<<" "<<middlepoint(1)<<" "<<middlepoint(2)<<std::endl;
    distance = (h_point1 - middlepoint).norm();
    std::cout<<" fly to middle point, distance is "<<distance<<std::endl;

    still = 0.0;
    this->publishState(still, global_velocity, h_point1, middlepoint);
    if(this->reachgoal(middlepoint)) ;

    v_line<< -(h_line(1)/(h_line(0) - h_line(2)*tan(angle *PI/180.0))), 1.0, \
    (h_line(1) *tan(angle *PI/180.0) /(h_line(0) - h_line(2)*tan(angle *PI/180.0)));
    v_line.normalize();

    std::cout<<" h line is"<<h_line(0)<<" "<<h_line(1)<<" "<<h_line(2)<<std::endl;
    std::cout<<" v line is"<<v_line(0)<<" "<<v_line(1)<<" "<<v_line(2)<<std::endl;
    endpoint = middlepoint + v_line*sensor_range*2.0;

    still = 0.0;
    this->publishState(still, global_velocity, middlepoint, endpoint);

    distance = (endpoint - middlepoint).norm();
    std::cout<<" v fly distance is"<<distance<<std::endl;
    std::cout<<" v fly planning end point is"<<endpoint(0)<<" "<<endpoint(1)<<" "<<endpoint(2)<<std::endl;
    std::cout<<" plan v fly finished! v fly until the signal disappears!"<<std::endl;

    while(true){
        ros::spinOnce();
        if(signal(i) > 1e-3){;}
        else break;
    }
    std::cout<<" v fly finished!"<<std::endl;

    ros::spinOnce();
    v_point1 = x;
    endpoint = middlepoint - sensor_range*v_line*2.0;

    still = 0.0;
    this->publishState(still, global_velocity, v_point1, endpoint);

    std::cout<<" v backward fly start point is"<<v_point1(0)<<" "<<v_point1(1)<<" "<<v_point1(2)<<std::endl;
    std::cout<<" v backward fly planning end point is"<<endpoint(0)<<" "<<endpoint(1)<<" "<<endpoint(2)<<std::endl;
    std::cout<<" plan v backward fly finished! v backward fly until the signal disappears!"<<std::endl;

    ros::Rate third_loop_rate(50);

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

    ros::spinOnce();
    v_point2 = x;
    std::cout<<" v backward fly real end point is"<<v_point2(0)<<" "<<v_point2(1)<<" "<<v_point2(2)<<std::endl;

    Eigen::Vector3d corepoint, coredirection;
    corepoint = (v_point2 + v_point1)/2 ;
    coredirection << (h_line(2)*v_line(1) - h_line(1)*v_line(2))/(h_line(0)*v_line(1) - h_line(1)*v_line(0)), \
    (h_line(2)*v_line(0) - h_line(0)*v_line(2))/(h_line(1)*v_line(0) - h_line(0)*v_line(1)),\
    -1;
    coredirection.normalize();

    double t=(plane_vector.dot(plane_point - corepoint))/(coredirection.dot(plane_vector));
    victim_position = corepoint + coredirection*t;

    std::cout<<" corepoint is "<<corepoint<<std::endl;
    std::cout<<" coredirection is "<<coredirection<<std::endl;
    std::cout<<" t is "<<t<<std::endl;
    std::cout<<" victim_position is "<<victim_position<<std::endl;

    for(int index = 0; index < v_point2.size(); index++){
        victim.push_back(victim_position(index));
    }

    std::cout<<" fly to the victim!"<<std::endl;

    still = 0.0;
    this->publishState(still, local_velocity, v_point2, victim_position);
    if(this->reachgoal(victim_position)) std::cout<<" reach victim!"<<std::endl;

    //uncomment here to use potential field planner

    // while(true){
    //     ros::spinOnce();
    //     std::cout<<" x is"<<x<<std::endl;

    //     Eigen::Vector3d x_prev;
    //     x_prev = x;

    //     x_next = this->potentialField(0.5,x,victim_position);
    //     std::cout<<" x_next is"<<x_next<<std::endl;

    //     still = 0.0;
    //     this->publishState(still, local_velocity, x, x_next);

    //     if(this->reachgoal(x_next)) std::cout<<" reach x_next!"<<std::endl;

    //     if((x_next - x_prev).norm() <= 0.5){
	// 	    break;
	//     }
    // }

    // ros::spinOnce();
    // std::cout<<" potential field guides me to  "<<x<<std::endl;

    pred_victims.push_back(victim);
    pred_victims_index.push_back(i);
    std::cout<<"find"<< i+1 << "victim! "<< victim[0]<<" "<<victim[1]<<" "<<victim[2]<<std::endl;
    std::cout<<"real victim"<< positions_of_victims(i,0)<<" "<<positions_of_victims(i,1)<<" "<<positions_of_victims(i,2)<<std::endl;

    error_rate = 100*(fabs(victim[0]-positions_of_victims(i,0))/fabs(positions_of_victims(i,0)) +\
     fabs(victim[2]-positions_of_victims(i,2))/fabs(positions_of_victims(i,2)) +\
     fabs(victim[1]-positions_of_victims(i,1))/fabs(positions_of_victims(i,1)))/3;  

    std::cout<<"The error rate of "<<i+1<<"th victim is "<<error_rate<<"%"<<std::endl;  
    pred_victims_error.push_back(error_rate);

    Eigen::Vector3d vector_distance;
    vector_distance << victim[0]-positions_of_victims(i,0), victim[1]-positions_of_victims(i,1), victim[2]-positions_of_victims(i,2);
    distance_error = vector_distance.norm();  // 
    std::cout<<"The distance error of "<<i+1<<"th victim is "<<distance_error<<"m"<<std::endl;  //
    pred_victims_error_distance.push_back(distance_error); 

    still = 0.0;
    this->publishState(still, global_velocity, victim_position, startpoint);
    std::cout<<" plan finished,fly back to the start point!"<<std::endl;
    if(this->reachgoal(startpoint)) std::cout<<" fly back to the start point, local search finished!"<<std::endl;
    
}

void BasicPlanner::globalsearch(){
    int maxsignal;
    double still;
    Eigen::Vector3d global_goal_position, beginpoint;
    Eigen::MatrixXd local_waypoint;
    for(int i = 0; i < global_path.cols(); i++){
        global_goal_position = global_path.col(i);
        if(i == 0){
            beginpoint = x;
            still = 0;
            std::cout<<"start point is " << beginpoint <<std::endl;
            std::cout<<"target point is " << global_goal_position <<std::endl;
            this->publishState(still, global_velocity, beginpoint,global_goal_position);
            if(this->reachgoal(global_goal_position)){
                std::cout<<"search begins!"<<std::endl;
            }
            start = ros::Time::now();
        }
        else{
            beginpoint = global_path.col(i-1);
            still = 0;
            std::cout<<"start point is " << beginpoint <<std::endl;
            std::cout<<"target point is " << global_goal_position <<std::endl;
            this->publishState(still, global_velocity, beginpoint,global_goal_position);;
        }

        while(true){
            ros::spinOnce();
            maxsignal = this->max_signal_index();
            if(maxsignal == -1){;}
            else if(signal(maxsignal)>0) {
                std::cout<<"received signal"<< signal(maxsignal)<<std::endl;
                Eigen::Vector3d search_point;
                search_point = x;

                std::cout<<"looking for " << maxsignal+1 <<"th victims!"<<std::endl;
                std::cout<<"start point is " << search_point <<std::endl;

                this->localsearch(search_point, maxsignal, beginpoint);
                ros::spinOnce();
                still = 0;
                this->publishState(still, global_velocity, x, global_goal_position);
                std::cout<<"global search continues!"<<std::endl;
            }

            ros::spinOnce();
            if(this->distance2goal(global_goal_position) > 1.0 ){
                if(x(0) > grid(0) + 2*sensor_range || x(0) < grid(2) - 2*sensor_range || x(1) > grid(3) + 2*sensor_range || x(1) < grid(1) - 2*sensor_range){
                    std::cout<<"Fly out of the field! "<<std::endl;
                    if(this->reachgoal(global_goal_position)) break;
                }
            }
            else break;
        }
        std::cout<<"reach local waypoint "<<global_goal_position(0)<<global_goal_position(1)<<global_goal_position(2)<<std::endl;
        if (i == global_path.cols() - 1){
            still = 1;
            this->publishState(still, global_velocity, global_goal_position, global_goal_position);
        }
     }
}

int BasicPlanner::max_signal_index(){
    double max = 0;
    int maxindex = 0;
    for (int i = 0; i < signal.size(); i++){
        for (int j = 0; j < pred_victims_index.size(); j++){
            if(pred_victims_index[j] == i) break;
            else if (j == (pred_victims_index.size()-1)){
                if(max<signal(i)){
                    max = signal(i);
                    maxindex = i;
                }
            }
        }
        if( i == signal.size()-1){
            if(max == 0) return -1;
            else return maxindex;
        }
    }
}

bool BasicPlanner::publishState(const double &still , const double &velocity, const Eigen::Vector3d &startpoint, const Eigen::Vector3d &endpoint){
    std_msgs::Float64MultiArray msg;
    std::vector<double> state;
    state.push_back(still);
    state.push_back(velocity);
    for(int i =0; i<3; i++){
        state.push_back(startpoint[i]);
    }
    for(int i =0; i<3; i++){
        state.push_back(endpoint[i]);
    }
    msg.data = state; 
    pub_state.publish(msg);
    std::cout<<"publish done!"<<std::endl;

    return true;
}
