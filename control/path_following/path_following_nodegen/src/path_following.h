#ifndef PATH_TRACKING_H
#define PATH_TRACKING_H

#include "ros/ros.h"
#include "ros/timer.h"
#include "tf/tf.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <string.h>
#include <cstring>
#include <vector>
#include <cmath>
#include <deque>

#include <visualization_msgs/Marker.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "control_msgs/CanFrame.h"
#include "control_msgs/Vehicle_cmd.h"
#include "control_msgs/VehicleState.h"
#include "path_msgs/State.h"
#include "novatel_gps_msgs/Inspva.h"

#include "nav_msgs/Odometry.h"

//Convert GPS/fix to Odom (Utm52N)
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <gps_common/GPSFix.h>
#include <novatel_gps_msgs/NovatelHeading2.h>

using namespace std;

class PurePursuit
{
public:
	PurePursuit();
	~PurePursuit();

	void vehStateCallback(const control_msgs::VehicleState &msg);
	void refPathCallback(const visualization_msgs::Marker path);
	void odomCallback(const nav_msgs::OdometryPtr& msg);
	double getDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);
	double getSign(double angle);
	void findLookahead(vector<geometry_msgs::Point> path);
	void controlSteering();
	void visualizeLah();

private:
	ros::NodeHandle nh;

	ros::Publisher lookahead_marker_pub_;
	ros::Publisher steering_cmd_pub_;
	ros::Publisher lookahead_distance_pub_;
	
	// Subscribed variables
	vector<geometry_msgs::Point> ref_path_;
	geometry_msgs::Pose current_pose_;
	control_msgs::VehicleState vehicle_state_;

	// Publishing variable
	visualization_msgs::Marker lah_marker_;

	// variables
	double ego_to_lah_angle_;
	double lah_distance_;
	double steering_angle_;
	double front_to_rear_;
};

#endif // PATH_TRACKING_H
