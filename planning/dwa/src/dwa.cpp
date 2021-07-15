/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include "dwa/dwa.h"

#define PI 3.141592653

using Traj = std::vector<std::array<float, 5>>;
using Obstacle = std::vector<std::array<float, 2>>;
using State = std::array<float, 5>;
using Window = std::array<float, 4>;
using Point = std::array<float, 2>;
using Control = std::array<float, 2>;

class Config{
public:
  float max_speed = 13.889;
  float min_speed = 0.0;
  float max_yawrate = 40.0 * PI / 180.0;
  float max_accel = 1.0;
  float robot_radius = 0.3;
  float max_dyawrate = 40.0 * PI / 180.0;
  float v_reso = 0.01;
  float yawrate_reso = 0.1 * PI / 180.0;
  float dt = 0.1;
  float predict_time = 6.8;
  float to_goal_cost_gain = 1.0;
  float speed_cost_gain = 1.0;
  float max_time_seq = 10.0;
};

State motion(State x, Control u, float dt){
  x[2] += u[1] * dt;
  x[0] += u[0] * std::cos(x[2]) * dt;
  x[1] += u[0] * std::sin(x[2]) * dt;
  x[3] = u[0];
  x[4] = u[1];
  return x;
};

Window calc_dynamic_window(State x, Config config){

  return {{
    std::max((x[3] - config.max_accel * config.dt), config.min_speed),
    std::min((x[3] + config.max_accel * config.dt), config.max_speed),
    std::max((x[4] - config.max_dyawrate * config.dt), -config.max_yawrate),
    std::min((x[4] + config.max_dyawrate * config.dt), config.max_yawrate)
  }};
};


Traj calc_trajectory(State x, float v, float y, Config config){

  Traj traj;
  traj.push_back(x);
  float time = 0.0;
  while (time <= config.predict_time){
    x = motion(x, std::array<float, 2>{{v, y}}, config.dt);
    traj.push_back(x);
    time += config.dt;
  }
  return traj;
};


float calc_obstacle_cost(Traj traj, Obstacle ob, Config config){
  // calc obstacle cost inf: collistion, 0:free
  // printf("size: %d\n", traj.size());
  int skip_n = 2;
  float minr = std::numeric_limits<float>::max();

  for (unsigned int ii=0; ii<traj.size(); ii+=skip_n){
    for (unsigned int i=0; i< ob.size(); i++){
      float ox = ob[i][0];
      float oy = ob[i][1];
      float dx = traj[ii][0] - ox;
      float dy = traj[ii][1] - oy;

      float r = std::sqrt(dx*dx + dy*dy);
      if (r <= config.robot_radius){
          return std::numeric_limits<float>::max();
      }

      if (minr >= r){
          minr = r;
      }
    }
  }

  return 1.0 / minr;
};

float calc_to_goal_cost(Traj traj, Point goal, Config config){

  float goal_magnitude = std::sqrt(goal[0]*goal[0] + goal[1]*goal[1]);
  float traj_magnitude = std::sqrt(std::pow(traj.back()[0], 2) + std::pow(traj.back()[1], 2));
  float dot_product = (goal[0] * traj.back()[0]) + (goal[1] * traj.back()[1]);
  float error = dot_product / (goal_magnitude * traj_magnitude);
  float error_angle = std::acos(error);
  float cost = config.to_goal_cost_gain * error_angle;

  return cost;
};

Traj calc_final_input(
  State x, Control& u,
  Window dw, Config config, Point goal,
  std::vector<std::array<float, 2>>ob){

    float min_cost = 10000.0;
    Control min_u = u;
    min_u[0] = 0.0;
    Traj best_traj;

    // evalucate all trajectory with sampled input in dynamic window
    for (float v=dw[0]; v<=dw[1]; v+=config.v_reso){
      for (float y=dw[2]; y<=dw[3]; y+=config.yawrate_reso){

        Traj traj = calc_trajectory(x, v, y, config);
        // printf("Cal_traj size: %d\n", traj.size());
        float to_goal_cost = calc_to_goal_cost(traj, goal, config);
        // printf("to_goal_cost: %f\n", to_goal_cost);
        float speed_cost = config.speed_cost_gain * (config.max_speed - traj.back()[3]);
        // printf("speed_cost: %f\n", speed_cost);
        float ob_cost = calc_obstacle_cost(traj, ob, config);
        // printf("ob_cost: %f\n", ob_cost);
        float final_cost = to_goal_cost + speed_cost + ob_cost;

        if (min_cost >= final_cost){
          min_cost = final_cost;
          min_u = Control{{v, y}};
          best_traj = traj;
        }
      }
    }
    u = min_u;
    return best_traj;
};


Traj dwa_control(State x, Control & u, Config config,
  Point goal, Obstacle ob){
    // # Dynamic Window control
    Window dw = calc_dynamic_window(x, config);
    Traj traj = calc_final_input(x, u, dw, config, goal, ob);
    return u, traj;
}

void DWA::publishState(State x)
{
  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time::now();
  ps.header.frame_id = "map";
  ps.pose.position.x = x[0];
  ps.pose.position.y = x[1];
  tf::Quaternion quat;
  quat.setRPY(0, 0, x[2]);
  ps.pose.orientation.x = quat.x();
  ps.pose.orientation.y = quat.y();
  ps.pose.orientation.z = quat.z();
  ps.pose.orientation.w = quat.w();
  pose_pub_.publish(ps);
  printf("Publishing current state -- x: %f, y: %f, yaw: %f\n", x[0], x[1], x[2]);
}

void DWA::publishLocalPath(Traj ltraj){
	visualization_msgs::Marker marker;
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = "map";
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.id = 100;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.scale.x = 0.1;
	marker.color.g = 1.0;
	marker.color.a = 1.0;

	printf("local traj size: %d\n", ltraj.size());
	for(int i = 0; i < ltraj.size(); i++){
		geometry_msgs::Point point;
		point.x = ltraj[i][0];
		point.y = ltraj[i][1];
		marker.points.push_back(point);
	}
  	local_path_pub_.publish(marker);
}

void addToTrajectory(nav_msgs::Path &path, State x){
	geometry_msgs::PoseStamped ps;
	ps.pose.position.x = x[0];
	ps.pose.position.y = x[1];
	path.poses.push_back(ps);
}

void outCSV(const std::vector<float>data, std::string filename)
{
  std::ofstream f;
  f.open(filename);

  for (int i = 0; i < data.size(); ++i)
  {
    f << (data[i]) << std::endl;
  }
  f.close();
  printf("CSV output complete\n");
}

DWA::DWA(): nh_(){
	map_initialized_ = false;

	bool end_sim = false;
	ros::Rate loop_rate(10);

  	map_sub_ = nh_.subscribe("/adj_lanes", 1, &DWA::mapInfoCallback, this);
  	pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mpc_pose", 1);
  	traj_pub_ = nh_.advertise<nav_msgs::Path>("/traj", 1);
  	local_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/local_path", 1);

  	Control u({{0.0, 0.0}});
  	Config config;
  	State x({{-5.61911, -3.91944, 6.16358, 0.0, 0.0}});
	Point goal({{285.641, -40.3888}});
	// Obstacle ob({});

	// Past trajectory
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "map";

	// Add to trajectory
	Traj traj;
	traj.push_back(x);
	addToTrajectory(path, x);

	while(ros::ok()){
		float distance = 0.0;
		float time_seq = 0.0;
		std::vector<float> data;

		if (map_initialized_){
      printf("local map size: %d\n", local_map_.size());
      // local_map_.clear();
			while (1){

		    if (time_seq > config.max_time_seq) // end of simulation
		    {
		    	printf("End of simulation\n");
		    	printf("Distance travelled: %fm\n", distance);
  				std::string root = "/home/euigon/sim_ws/src/autonomous-car/planning/dwa/src/";
  				std::string path = root + "w_lanes_test.csv";
  				outCSV(data, path);
  				end_sim = true;
  				break;
		    }

		  	clock_t start = clock();
		    Traj ltraj = dwa_control(x, u, config, goal, local_map_);
		    State prev = x;
	    	x = motion(x, u, config.dt);
	    	distance += sqrt((x[0]-prev[0])*(x[0]-prev[0]) + (x[1]-prev[1])*(x[1]-prev[1])); 
				float op_time = (float)(clock() - start)/CLOCKS_PER_SEC;
			  ROS_WARN("Time seq %f: %0.3f s", time_seq, op_time);
			  data.push_back(op_time);
	    	publishLocalPath(ltraj);
	    	traj.push_back(x);
    		publishState(x);
    		addToTrajectory(path, x);
    		traj_pub_.publish(path);
	    	time_seq += config.dt;
			}
		}
		else
		{
			publishState(x);
		}

		if (end_sim){
			break;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

DWA::~DWA(){}


void DWA::mapInfoCallback(const path_msgs::Map map_msg)
{
  printf("***Map information received***\n");

  path_msgs::Map map_info = map_msg;

  lanes_.clear();

  for(int i = 0; i < map_info.lanes.size(); i++)
  {
    path_msgs::Lane lane = map_info.lanes[i];
    lanes_.push_back(lane);
  }

  offsetX_ = map_info.OffsetMapX;
  offsetY_ = map_info.OffsetMapY;

  setMap();

  map_initialized_ = true;
}

void DWA::setMap(){
	
	local_map_.clear();

	for(int i=0; i < lanes_.size(); ++i){
		for (int j=0; j < lanes_[i].geometry.size(); ++j){
			float x = lanes_[i].geometry[j].x - offsetX_;
			float y = lanes_[i].geometry[j].y - offsetY_;
			local_map_.push_back({{x, y}});
		}
	}
	// printf("Obs size: %d\n", local_map_.size());
	// printf("Map setting complete.\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dwa_node");
  DWA node;
  return 0;
}
