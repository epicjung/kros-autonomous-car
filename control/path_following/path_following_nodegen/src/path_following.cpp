#include "path_following.h"
#define PI 3.1415926535897931

PurePursuit::PurePursuit(){

	static ros::Subscriber velocity_sub_ = nh.subscribe("/vehicle_state",1,&PurePursuit::vehStateCallback,this);
	static ros::Subscriber ref_path_sub_ = nh.subscribe("/ref_path", 1, &PurePursuit::refPathCallback, this);
	static ros::Subscriber odom_sub_ = nh.subscribe("/gps_odom", 1, &PurePursuit::odomCallback, this);
	static ros::Rate *rate = new ros::Rate(100);

	lookahead_marker_pub_ = nh.advertise<visualization_msgs::Marker>("look_ahead_point_marker",1);
	lookahead_distance_pub_ = nh.advertise<std_msgs::Float64>("/lookahead_distance", 1);
	steering_cmd_pub_ = nh.advertise<control_msgs::Vehicle_cmd>("/Steering_cmd",1);
	
	lah_distance_ = 8;
	front_to_rear_ = 2.65;

	while (ros::ok())
  	{
    	ros::spinOnce();

		vector<geometry_msgs::Point> current_path = ref_path_;

		if (current_path.size() > 0)
		{
			findLookahead(current_path);
			visualizeLah();
			controlSteering();
		}
		rate->sleep();
  	}
}

PurePursuit::~PurePursuit()
{

}

void PurePursuit::vehStateCallback(const control_msgs::VehicleState &msg)
{
	double k = 0.4;

	float velocity = msg.v_ego;

	if(velocity < 20)
		lah_distance_ = 8;
	else if(velocity < 40)
		lah_distance_ = k * velocity;
	else
		lah_distance_ = 16;

	printf("LookAhead: %f m\n", lah_distance_);
}


void PurePursuit::odomCallback(const nav_msgs::OdometryPtr& msg)
{
	current_pose_ = msg->pose.pose;
}

void PurePursuit::refPathCallback(const visualization_msgs::Marker path)
{
	ref_path_ = path.points;
}

void PurePursuit::visualizeLah(){
	lah_marker_.header.frame_id = "map";
	lah_marker_.header.stamp = ros::Time::now();
	lah_marker_.action = visualization_msgs::Marker::ADD;
	lah_marker_.pose.orientation.w = 1.0;
	lah_marker_.id = 0;
	lah_marker_.type = visualization_msgs::Marker::POINTS;
	lah_marker_.scale.x = 2;
	lah_marker_.scale.y = 2;
	lah_marker_.color.r = 1.0;
	lah_marker_.color.g = 1.0;
	lah_marker_.color.b = 1.0;
	lah_marker_.color.a = 1.0;

	lookahead_marker_pub_.publish(lah_marker_);
	lah_marker_.points.clear();
}

double PurePursuit::getDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
	double distance = sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
	return distance;
}

double PurePursuit::getSign(double angle){
	if(angle >= 0.0){
		return 1.0;
	}
	else{
		return -1.0;
	}
}

void PurePursuit::findLookahead(vector<geometry_msgs::Point> path){
	int index = 0;
	double min = 9999;
	double sum = 0.0;
	double difftheta;
	double ego_angle = tf::getYaw(current_pose_.orientation); // current ego angle
	double lah_angle; // lookahead-line angle
	double diff_angle;
	double sign;

	for (int i=1; i<path.size(); i++){
		sum += getDistance(path[i-1], path[i]);
		lah_angle = atan2(path[i].y - current_pose_.position.y, path[i].x - current_pose_.position.x);
		diff_angle = ego_angle - lah_angle;

		if(abs(diff_angle) >= PI){
			sign = getSign(diff_angle);
			diff_angle = diff_angle - (sign * 2 * PI);
		}

		if(sum > lah_distance_){
			index = i-1;
			ego_to_lah_angle_ = diff_angle;
			break;
		}
	}

	// reference path shorter than lookahead distance
	if (index == 0)
	{
		geometry_msgs::Point last_point = path[path.size()-1];
		lah_angle = atan2(last_point.y-current_pose_.position.y, last_point.x - current_pose_.position.x);
		diff_angle = ego_angle - lah_angle;

		if(abs(diff_angle) >= PI){
			sign = getSign(diff_angle);
			diff_angle = diff_angle - (sign * 2 * PI);
		}

		index = path.size()-1; // set the last point as lookahead point
		ego_to_lah_angle_ = difftheta;
	}

	lah_marker_.points.push_back(path[index]);
}

void PurePursuit::controlSteering()
{
	double steering_angle = atan2(2.0*front_to_rear_*sin(ego_to_lah_angle_), lah_distance_);
	steering_angle = (steering_angle * 180 / PI) * -13.9;
	printf("Steering: %f\n", steering_angle);

	// Publish steering angle to CAN
	control_msgs::Vehicle_cmd cmd;
	cmd.target_steering = steering_angle;
  	steering_cmd_pub_.publish(cmd);
}
