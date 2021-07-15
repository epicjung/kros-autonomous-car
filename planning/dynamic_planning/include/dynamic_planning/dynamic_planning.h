
#ifndef DYNAMIC_PLANNING_H
#define DYNAMIC_PLANNING_H

#include <ros/ros.h>
#include <vector>
#include <thread>
#include <mutex>
#include <map>
#include <cmath>
#include <queue>
#include <iostream>
#include <fstream>
#include <deque>
#include <functional>
#include <numeric>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "path_msgs/Node.h"
#include "path_msgs/Link.h"
#include "path_msgs/Cost.h"
#include "path_msgs/Bezier.h"
#include "path_msgs/Trajectory.h"
#include "path_msgs/Waypoint.h"
#include "path_msgs/Map.h"
#include "path_msgs/State.h"
#include "path_msgs/Avoidance.h"
#include "obj_msgs/ObjList.h"
#include "obj_msgs/Obj.h"
#include "obj_msgs/Ego.h"
// #include "object_tracker/Tracker.h"

#include "control_msgs/VehicleState.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "spline.h"
#include "timercpp.h"

#define PI 3.14159265359
#define HALF_LANE 1.5


enum class EControl
{
  KEEP = -1,
  STOP = 1,
  STOPLINE = 2,
  DECELERATE = 3,
  OTHERS = 4
};

enum class LControl
{
  KEEP = 0,
  LEFT = 1,
  RIGHT = 2,
  BOTH = 3
};

enum LCode
{
  CENTER = 1,
  UTURN = 2,
  LANE = 3,
  NO_LANE_CHANGE = 5,
  NO_PARKING = 8,
  NO_STOPPING = 9,
  BARRIER = 99
};

enum RType
{
  NORMAL = 1,
  ATYPICAL_STRAIGHT = 6,
  INTERSECTION = 7,
  ATYPICAL_TURN = 8,
  TURN = 9,
  BEFORE_TURN = 10
};

enum OType{
  NOINTEREST = 0,
  DYNAMIC = 1,
  STATIC = 2,
  FRONTOBJ  = 3,
  OPPOSITE = 4,
  RED = 5,
  EGO = 6,
  GOAL = 7,
  STOP = 8,
  ATYPICAL = 9,
  LANES = 10,
  FAKE = 11,
  RAW = 12
};


struct Candidate
{
  int index;
  double smc; // smoothness cost
  double sc; // obstacle cost
  double dc; // dynamic cost
  double gc; // global path following cost
  double max_k; // maximum curvature
  tk::spline sp; // spline path
  std::vector<geometry_msgs::Point> points;
  obj_msgs::Obj collision_obj;
  double collision_len;
  double time_to_collision;
  double throttle;
};

class DynamicPlanning
{

public:

	typedef enum STATE
	{
		INITIALIZING = -1,
		RELAYING = 0,
		CRUISING = 1,
    BLOCKED = 2,
    HARD_BRAKE = 3,
    STOP_GO = 4,
    RELAYING_I = 5,
    RELAYING_II = 6,
    RELAYING_III = 7,
	} State;

	DynamicPlanning();
	~DynamicPlanning();

  void run();


private:

  /* ros */
	ros::NodeHandle nh_, private_nh_;
  ros::Subscriber estop_status_sub_;
  ros::Subscriber current_odom_sub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber obstacle_index_sub_;
	ros::Subscriber global_trajectory_sub_;
  ros::Subscriber object_info_sub_;
  ros::Subscriber traffic_info_sub_;
  ros::Subscriber map_info_sub_;
  ros::Subscriber goal_pose_sub_;
  ros::Subscriber ego_state_sub_;
  ros::Subscriber lookahead_sub_;
  ros::Subscriber atypical_obj_sub_;
  ros::Publisher ego_speed_pub_;
  ros::Publisher test_path_pub_;
  ros::Publisher ego_acc_pub_;
  ros::Publisher ego_location_pub_;
  ros::Publisher lookahead_dist_pub_;
  ros::Publisher bounding_box_pub_;
  ros::Publisher road_state_pub_;
  ros::Publisher estop_pub_;
  ros::Publisher avoid_state_pub_;
  ros::Publisher replan_pub_;
  ros::Publisher reached_pub_;
  ros::Publisher adj_lanes_pub_;
  std::vector<ros::Publisher> publishers_;
  ros::Rate *rate_;
  tf::TransformBroadcaster tfBroadcaster;

  /* rviz visualization*/
  ros::Publisher rviz_current_pose_pub_;
  ros::Publisher rviz_ellipse_pub_;
  ros::Publisher rviz_stop_nodes_pub_;
  ros::Publisher rviz_status_pub_;
  ros::Publisher rviz_velocity_pub_;
  ros::Publisher ref_path_pub_;
  visualization_msgs::MarkerArray ellipse_markers_;
  visualization_msgs::MarkerArray velocity_markers_;

  /* Car specification */
  double width_;
  double length_;

	/* params */
	int loop_rate_;
  double locate_range_;

  /* topics and frame_ids */
  std::string gps_topic_;
  std::string map_frame_;

  /* classes */
	State state_;

  /* variables */
	geometry_msgs::PoseStamped current_pose_;
	path_msgs::Trajectory global_trajectory_;

  /* spline */
  double s_start_;
  double s_end_;
  double curv_;
  tk::spline sx_;
  tk::spline sy_;
  double global_path_len_;
  double ds_;

  /* avoidance */
  double L_cutin_;
  double L_follow_;

  /* speed limit */
  double lat_offset_;
  double sign_limit_;
  double stop_dist_;
  double lat_acc_limit_;
  double safety_gain_;
  double ref_speed_;

  /* path candidates */
  int path_size_;
  double offset_;
  double min_path_len_;
  double max_path_len_;
  double path_len_;
  int opt_path_;
  std::vector<Candidate> candidates_;

  /* cost */
  std::vector<double> R_;
  double ws_; // static cost weight
  double wsm_; // smoothness cost weight
  double wd_; // dynamic cost weight
  double wg_; // global-path following weight

  /* Ego */
  obj_msgs::Obj ego_;
  double ego_speed_;
  double ego_acc_;
  double yaw_;
  bool is_located_;
  bool first_located_;
  double max_decel_;
  double max_accel_;

  /* Obstacle */
  std::vector<obj_msgs::Obj> obs_list_;
  double lane_thres_;
  double sigma_;
  double obs_cost_;


  /* Initialization boolean */
  bool pose_initialized_;
  bool trajectory_initialized_;
  bool map_initialized_;
  bool dynamic_initialized_;
  bool path_following_initialized_;

  /* map */
  std::map <std::string, path_msgs::Link> links_;
  std::map <std::string, path_msgs::Node> nodes_;
  std::map <std::string, path_msgs::Lane> lanes_;
  double offsetX_;
  double offsetY_;

  /*** Functions ***/
  /* rviz */
  void visualizeStopNodes();
  void visualizeCurrentPose();
  void visualizePathCandidate(int optimal_index);
  void visualizeEllipse(obj_msgs::Obj obj);
  void visualizeVelocity(obj_msgs::Obj obj);

	/* callbacks */
  void currentOdomCallback(const nav_msgs::Odometry msg);
  void poseCallback(const geometry_msgs::PoseStamped msg);
  void trajectoryCallback(const path_msgs::Trajectory msg);
  void mapInfoCallback(const path_msgs::Map map_msg);
  void obsCallback(const obj_msgs::ObjList msg);
  void egoStateCallback(const control_msgs::VehicleState msg);

  /* Control */
  double getSpeedLimit();
  double getCurvatureSpeed(int index);
  double getAvoidSpeed(int index);
  double getCutInSpeed();
  double getBezierSpeed();
  double getEndOfPathSpeed();
  double getTargetSpeed(int index);
  double getTargetThrottle(int index);
  void setTargetSpeed(double target_speed);
  void setTargetThrottle(double target_acc);

  /* Object */
  void transformObject(const obj_msgs::Obj src, obj_msgs::Obj &dest);
  void transformAtypicalObject(obj_msgs::Obj &obj);
  void locateObj(obj_msgs::Obj &obj);
  obj_msgs::Obj getEllipse(obj_msgs::Obj obj, geometry_msgs::Point center, double scale, bool elongated);

  /* path generation */
  void generateDynamicPath();
  void generateCandidates(double min_offset, int end_index, double tangent_angle);
  Candidate generatePathCandidate(double offset, int end_index, double tangent_angle);
  void convertPathToCartesian();
  void generateCenterLine();
  void setR(int idx, bool is_obs, std::vector<path_msgs::Lane> lanes);
  double getPathLength();  
  int getLateralDir(double x, double y, double sk);
  int distanceToIndex(tk::spline s, int idx, double dist);
  void findOptimalPath();
  std::vector<double> getCollisionRisk();
  LControl laneStatus(path_msgs::Link link);
  std::tuple<double, double> getSP(obj_msgs::Obj obj);
  std::string getLinkID(obj_msgs::Obj obj);
  std::tuple<std::vector<path_msgs::Lane>, std::vector<path_msgs::Lane>> getAdjacentLanes(std::string link_id);
  void checkPath(std::vector<geometry_msgs::Point> points,
                                  std::vector<path_msgs::Lane> adj_lanes,
                                  int cand_idx);
  void publishReferencePath(std::vector<geometry_msgs::Point> path);


  void startThread();

  /* Cost */
  void setGlobalCost();
  void setObstacleCost();

  /* miscellaneous */
  std::tuple<double, double> locateObjectFromEgo(obj_msgs::Obj obj);
  void printObjStatus();
  double getTangentAngle();
  bool isCollision(double x, double y, obj_msgs::Obj obj);
  bool isSafeLane(path_msgs::Lane lane);
  bool isBezierAhead(int check_index);
  bool isAtIntersection(int index);
  bool isAtLaneChange(int index);
  bool isLocated(obj_msgs::Obj obj);
  std::string searchNextLink(int index);
  int searchTrajIndex(obj_msgs::Obj obj, path_msgs::Trajectory traj, double locate_range);
  int searchLinkIndex(geometry_msgs::Point point, path_msgs::Link link);
  int searchLinkIndexByLength(path_msgs::Link link, int from_idx, int to_idx, double refer_length);
  double calcLength(path_msgs::Link, int from_idx, int to_idx);
  double getDistance(double x1, double y1, double x2, double y2);
  double getAngleDiff(double first_angle, double second_angle);
};

#endif
