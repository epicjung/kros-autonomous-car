
#ifndef DYNAMIC_PLANNING_H
#define DYNAMIC_PLANNING_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include "obj_msgs/ObjList.h"
#include "obj_msgs/Obj.h"
#include "control_msgs/VehicleState.h"
#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#define PI 3.14159265359


class Simulation
{

public:

	Simulation();
	~Simulation();

  void run();


private:

  /* ros */
	ros::NodeHandle nh_, private_nh_;
  ros::Publisher obs_pub_;
  ros::Publisher mpc_pub_;
  ros::Publisher past_traj_pub_;
  ros::Publisher vehicle_state_pub_;
  ros::Publisher traj_pub_;
  ros::Subscriber ref_path_sub_;
  ros::Subscriber target_speed_sub_;
  ros::Subscriber target_acc_sub_;

  double time_seq_;
  double max_time_seq_;
  double latency_;
  bool path_received_;
  bool speed_received_;
  std::string filename_;
  std::vector<geometry_msgs::Point> ref_path_;
  nav_msgs::Path past_trajectory_;
  visualization_msgs::Marker traj_;

  /* mpc */
  double px_, py_, psi_;
  double v_, steer_angle_, throttle_;
  double target_speed_;
  double max_speed_obs_;
  double target_throttle_;

  /* callbacks */
  void refPathCallback(visualization_msgs::Marker marker);
  void targetSpeedCallback(std_msgs::Float64 msg);
  void targetThrottleCallback(const std_msgs::Float64 msg);
  std::vector<geometry_msgs::Point> getObsPath(std::string filename);
  std::vector<geometry_msgs::Point> getObsPath(obj_msgs::Obj obs);

  /* functions */
  void checkDynamic(obj_msgs::Obj ego, obj_msgs::Obj& obs);
  void insertObs(obj_msgs::ObjList& obslist, obj_msgs::Obj obs);
  void publishState(obj_msgs::Obj obj);
  void publishPastTrajectory();
  void solveMPC(std::vector<geometry_msgs::Point> path, obj_msgs::Obj &obj);

  obj_msgs::Obj createObstacle(double length, 
                              double width, 
                              double x, 
                              double y, 
                              double heading, 
                              double speed,
                              int type, int id);

};

#endif
