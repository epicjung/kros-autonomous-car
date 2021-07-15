#ifndef DWA_H_
#define DWA_H_

#include <ros/ros.h>
#include <map>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "path_msgs/Map.h"
#include "path_msgs/Lane.h"
#include <tf/transform_datatypes.h>

using Traj = std::vector<std::array<float, 5>>;
using Obstacle = std::vector<std::array<float, 2>>;
using State = std::array<float, 5>;
using Window = std::array<float, 4>;
using Point = std::array<float, 2>;
using Control = std::array<float, 2>;

class DWA
{

public:
	DWA();
	~DWA();

private:
	bool map_initialized_;
	double offsetX_;
	double offsetY_;
  	Obstacle local_map_;
	std::vector<path_msgs::Lane> lanes_;
	ros::NodeHandle nh_;
	ros::Publisher pose_pub_; 
	ros::Publisher traj_pub_;
	ros::Publisher local_path_pub_; 
	ros::Subscriber map_sub_;


	void mapInfoCallback(const path_msgs::Map map);
	void setMap();
	void publishState(State x);
	void publishLocalPath(Traj ltraj);
};

#endif