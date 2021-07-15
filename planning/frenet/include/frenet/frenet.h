#ifndef DWA_H_
#define DWA_H_

#include <ros/ros.h>
#include <map>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "path_msgs/Map.h"
#include "path_msgs/Lane.h"
#include "path_msgs/Trajectory.h"
#include <tf/transform_datatypes.h>
#include"cubic_spline.h"
#include"frenet_path.h"
#include"quintic_polynomial.h"
#include"quartic_polynomial.h"

using Traj = std::vector<std::array<float, 5>>;
using Obstacle = std::vector<std::array<float, 2>>;
using State = std::array<float, 5>;
using Window = std::array<float, 4>;
using Point = std::array<float, 2>;
using Control = std::array<float, 2>;

class Frenet
{

public:
	Frenet();
	~Frenet();

private:
	bool map_initialized_;
	bool global_initialized_;
	bool spline_initialized_;
	double offsetX_;
	double offsetY_;

  	std::vector<std::array<float, 2>>local_map_;
  	std::vector<float> wx_;
  	std::vector<float> wy_;
	ros::NodeHandle nh_;
	ros::Publisher pose_pub_; 
	ros::Publisher local_path_pub_; 
	ros::Subscriber map_sub_;
	ros::Subscriber global_path_sub_;

	void globalCallback(const path_msgs::Trajectory traj);
	void mapInfoCallback(const path_msgs::Map map);
	void publishState(float x, float y, float v);
	void publishLocalPath(std::vector<std::array<float, 2>> ltraj);
};

#endif