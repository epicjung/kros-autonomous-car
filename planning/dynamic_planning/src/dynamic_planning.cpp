#include "dynamic_planning/dynamic_planning.h"


void outCSV(const std::tuple<double, double>data, std::string filename)
{
  std::ofstream f(filename, std::ios::app);
  f << std::get<0>(data) << ","<<std::get<1>(data) << std::endl;
  f.close();
}

DynamicPlanning::DynamicPlanning()
: nh_(),
private_nh_("~"),
pose_initialized_(false),
trajectory_initialized_(false),
map_initialized_(false),
dynamic_initialized_(false),
path_following_initialized_(false)
{
  /* time-related */
  private_nh_.param<int>("loop_rate", loop_rate_, 100);

  /* topics and frame_ids */
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<std::string>("gps_topic", gps_topic_, "gps_odom");

  /* Trajectory */
  private_nh_.param<int>("path_size", path_size_, 4);
  private_nh_.param<double>("locate_range", locate_range_, 5.0);
  private_nh_.param<double>("offset", offset_, 0.5);
  private_nh_.param<double>("lat_acc_limit", lat_acc_limit_, 5);
  private_nh_.param<double>("safety_gain", safety_gain_, 0.8);
  private_nh_.param<double>("ref_speed", ref_speed_, 13.9);
  private_nh_.param<double>("sign_limit", sign_limit_, 50.0);
  private_nh_.param<double>("wsm", wsm_, 1.0);
  private_nh_.param<double>("ws", ws_, 1.0);
  private_nh_.param<double>("wd", wd_, 1.0);
  private_nh_.param<double>("wg", wg_, 1.0);
  private_nh_.param<double>("lane_thres", lane_thres_, 1.5);
  private_nh_.param<double>("sigma", sigma_, 0.5);
  private_nh_.param<double>("min_dist", min_path_len_, 15.0);
  private_nh_.param<double>("max_dist", max_path_len_, 45.0);

  /* avoidance */
  private_nh_.param<double>("cutin_thres", L_cutin_, 10.0);
  private_nh_.param<double>("follow_thres", L_follow_, 10.0);

  /* Car specification */
  private_nh_.param<double>("length", length_, 4.300);
  private_nh_.param<double>("width", width_, 2.3);
  private_nh_.param<double>("max_decel", max_decel_, 1.0);
  private_nh_.param<double>("max_accel", max_accel_, 1.0);

  /* Subscriber */
	current_odom_sub_ = nh_.subscribe(gps_topic_, 1, &DynamicPlanning::currentOdomCallback, this);
  current_pose_sub_ = nh_.subscribe("/mpc_pose", 1, &DynamicPlanning::poseCallback, this);
  global_trajectory_sub_ = nh_.subscribe("/global_trajectory", 1, &DynamicPlanning::trajectoryCallback, this);
  map_info_sub_ = nh_.subscribe("/map_info", 1, &DynamicPlanning::mapInfoCallback, this);
  object_info_sub_ = nh_.subscribe("/obj_info",1, &DynamicPlanning::obsCallback, this);
  ego_state_sub_ = nh_.subscribe("/vehicle_state", 1, &DynamicPlanning::egoStateCallback, this);

  /* Publisher */
  ego_speed_pub_ = nh_.advertise<std_msgs::Float64>("/Target_Velocity", 1);
  ego_acc_pub_ = nh_.advertise<std_msgs::Float64>("/target_throttle", 1);
  ref_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/ref_path", 1);
  rviz_current_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("/rviz/current_pose", 1);
  rviz_ellipse_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rviz/ellipse", 1);
  rviz_velocity_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rviz/velocity", 1);
  adj_lanes_pub_ = nh_.advertise<path_msgs::Map>("/adj_lanes", 1, true);
  test_path_pub_= nh_.advertise<path_msgs::Trajectory>("/test_path", 1, true);

  for(int i = 0; i < path_size_*2 + 1; i++)
  {
    std::string topic_name = "/rviz/candidate";
    publishers_.push_back(nh_.advertise<visualization_msgs::Marker>(topic_name + std::to_string(i),1));
  }

  // Initilization
  ego_speed_ = 0.0;
  ds_ = 0.5;
  first_located_ = false;
  // reset collision risk R
  for(int i = 0; i < 2*path_size_ + 1; i++)
  {
    R_.push_back(0.0);
  }
  rate_ = new ros::Rate(loop_rate_);
}

DynamicPlanning::~DynamicPlanning()
{
}

void DynamicPlanning::egoStateCallback(const control_msgs::VehicleState msg)
{
  ego_speed_ = msg.v_ego;
  ego_acc_ = msg.a_x;
}

void DynamicPlanning::obsCallback(const obj_msgs::ObjList msg)
{
  obs_list_.clear();
  ellipse_markers_.markers.clear();

  if (is_located_ && trajectory_initialized_)
  {

    // Iterate Through vehicles with boxes
    for(int i=0; i < msg.objlist.size(); i++)
    {
      if (msg.objlist[i].type != OType::NOINTEREST && msg.objlist[i].type != OType::OPPOSITE)
      // if (msg.objlist[i].type != OType::OPPOSITE)
      {
        obj_msgs::Obj lobj = msg.objlist[i];
        if (lobj.pose.position.x == 0.0 && lobj.pose.position.y == 0.0)
          continue;

        obj_msgs::Obj gobj = lobj;
        // transformObject(lobj, gobj);
        // locateObj(gobj);
        locateObj(gobj);
        if (gobj.index >= 0 && gobj.link_id != "")
        {
          double arc_diff = std::get<0>(locateObjectFromEgo(gobj));
          // printf("id: %d, type: %d, diff: %f m\n", gobj.id, gobj.type, arc_diff);
          gobj = getEllipse(gobj, gobj.pose.position, 1.0, false);
          visualizeEllipse(gobj);
          visualizeVelocity(gobj);
          obs_list_.push_back(gobj);
        }
        else
        {
          // printf("obj: %d not found\n", gobj.id);
        }
      }
    }

    // add rviz ellipse markers
    rviz_ellipse_pub_.publish(ellipse_markers_);
    rviz_velocity_pub_.publish(velocity_markers_);
  }
}

void DynamicPlanning::mapInfoCallback(const path_msgs::Map map_msg)
{
  path_msgs::Map map_info = map_msg;

  for(int i = 0; i < map_info.links.size(); i++)
  {
    path_msgs::Link link = map_info.links[i];
    links_.insert(make_pair(link.id, link));
  }

  for(int i = 0; i < map_info.nodes.size(); i++)
  {
    path_msgs::Node node = map_info.nodes[i];
    nodes_.insert(make_pair(node.id, node));
  }

  for(int i = 0; i < map_info.lanes.size(); i++)
  {
    path_msgs::Lane lane = map_info.lanes[i];
    lanes_.insert(make_pair(lane.id, lane));
  }

  offsetX_ = map_info.OffsetMapX;
  offsetY_ = map_info.OffsetMapY;

  ROS_INFO_STREAM("Map information received");

  map_initialized_ = true;
}

void DynamicPlanning::poseCallback(const geometry_msgs::PoseStamped msg)
{
  if (map_initialized_)
  {
    current_pose_.header.stamp = msg.header.stamp;
    current_pose_.header.frame_id = map_frame_;
    current_pose_.pose = msg.pose;
    ego_.id = 0;
    ego_.type = OType::EGO;
    ego_.pose = msg.pose;
    yaw_ = tf::getYaw(ego_.pose.orientation);
    pose_initialized_ = true;
    visualizeCurrentPose();
    startThread();
  }
}

void DynamicPlanning::currentOdomCallback(const nav_msgs::Odometry odom_msg)
{
  if (map_initialized_)
  {
    current_pose_.header.stamp = odom_msg.header.stamp;
    current_pose_.header.frame_id = map_frame_;
    current_pose_.pose = odom_msg.pose.pose;
    ego_.id = 0;
    ego_.type = OType::EGO;
    ego_.pose = odom_msg.pose.pose;
    // Transform from gps to hood or gps_to_rear
    yaw_ = tf::getYaw(ego_.pose.orientation);
    pose_initialized_ = true;
    startThread();
  }
}

void DynamicPlanning::trajectoryCallback(const path_msgs::Trajectory traj_msg)
{
  if (traj_msg.waypoints.size() > 0)
    trajectory_initialized_ = true;
  else
    trajectory_initialized_ = false;

  ROS_INFO_STREAM("Planned size: " << traj_msg.waypoints.size());

  global_trajectory_.waypoints.clear();
  for(int i = 0; i < traj_msg.waypoints.size(); i++)
  {
    if (i % 2 == 0)
    {
      global_trajectory_.waypoints.push_back(traj_msg.waypoints[i]);
    }
  }

  // Construct center line in s-p coordinate
  clock_t start = clock();
  
  generateCenterLine();

  ROS_WARN("GCL: Execution time: %0.3f s", (float)(clock() - start)/CLOCKS_PER_SEC);
}


void DynamicPlanning::run()
{
  ros::Rate loop_rate(loop_rate_);

  while (ros::ok())
  {
    ros::spinOnce();

    ROS_INFO_STREAM("Current: x=" << ego_.pose.position.x <<", y=" << ego_.pose.position.y);

    if (pose_initialized_ && trajectory_initialized_ && map_initialized_)
    {
      ROS_WARN("GPS, global path, map initialized...");
      break;
    }
    ros::Duration(1.0).sleep();
  }

  // relaying mode by default
  state_ = DynamicPlanning::STATE::RELAYING;

  // reset current index
  ego_.index = -1;

  // run dynamic planning
  while(ros::ok())
  {
    ros::spinOnce();

    rate_->sleep();
	}
}

/** 
 * Locate object w.r.t global path
 * 
 * @param[out] object located by index, arc-length, lateral offset, and link id
*/
void DynamicPlanning::locateObj(obj_msgs::Obj &obj)
{
  geometry_msgs::Point point;
  point.x = obj.pose.position.x;
  point.y = obj.pose.position.y;

  int min_idx = 0;
  double min_dist = getDistance(point.x, point.y, global_trajectory_.waypoints[min_idx].point.x, global_trajectory_.waypoints[min_idx].point.y);

  // iterate through the global path
  for(int i = 0; i < global_trajectory_.waypoints.size(); i++)
  {
    double dist = getDistance(point.x, point.y, global_trajectory_.waypoints[i].point.x, global_trajectory_.waypoints[i].point.y);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_idx = i;
    }
  }

  // locate the object within certain range from the path
  if (min_dist < locate_range_)
  {
    // printf("Obj searched at %d\n", min_idx);
    obj.index = min_idx;
    obj.s = std::get<0>(getSP(obj));
    obj.lat_offset = std::get<1>(getSP(obj));
    obj.link_id = getLinkID(obj);
  }
  else
  {
    obj.index = -1;
    // printf("Fail to search\n");
  }
}

/** 
 * Start of the program when GPS is received
 *
 * @param None
 * @return None 
*/
void DynamicPlanning::startThread()
{
  if (trajectory_initialized_ && pose_initialized_)
  {
    locateObj(ego_);
    // printf("Ego: %s, index %d, arc %f, lat offset %f m\n", ego_.link_id.c_str(), ego_.index, ego_.s, ego_.lat_offset);
    is_located_ = isLocated(ego_);
    if (is_located_)
    {
      if (!first_located_) {
        searchTrajIndex(ego_, global_trajectory_, 5.0);
        first_located_ = true;
      }

      printObjStatus();

      generateDynamicPath();

    }
  }
// }
}

/**
 * Check whether an object is successfully located w.r.t global path
 *
 * @param object to check
 * @return boolean
 */
bool DynamicPlanning::isLocated(obj_msgs::Obj obj)
{
  return obj.index >= 0 && obj.link_id != "";
}

/**
 * Calculated ego vehicle's tangent angle along global path
 *
 * @param None
 * @return tangent angle
 */
double DynamicPlanning::getTangentAngle()
{
  double path_angle;
  if (ego_.index < global_trajectory_.waypoints.size()-1)
  {
    geometry_msgs::Point current_wp = global_trajectory_.waypoints[ego_.index].point;
    geometry_msgs::Point next_wp = global_trajectory_.waypoints[ego_.index+1].point;
    path_angle = atan2(next_wp.y-current_wp.y, next_wp.x-current_wp.x);
  }
  else
  {
    geometry_msgs::Point current_wp = global_trajectory_.waypoints[ego_.index].point;
    geometry_msgs::Point prev_wp = global_trajectory_.waypoints[ego_.index-1].point;
    path_angle = atan2(current_wp.y-prev_wp.y, current_wp.x-prev_wp.x);
  }
  
  return getAngleDiff(yaw_, path_angle);
}

int DynamicPlanning::getLateralDir(double cx, double cy, double sk)
{
  double x1 = sx_(sk);
  double y1 = sy_(sk);
  double x2 = sx_(sk + 0.5);
  double y2 = sy_(sk + 0.5);
  double delta_x = x2 - x1;
  double delta_y = y2 - y1;
  double slope = 0.0;
  if (delta_x != 0) {
    slope = delta_y / delta_x;
  }
  else {
    std::cout << "invalid" << std::endl;
    return 1;
  }

  double check = cy - y1 - slope * (cx - x1);

  // First and fourth quadrants
  if ((delta_y > 0 && delta_x > 0) || (delta_y < 0 && delta_x > 0))
  {
    if (check <= 0)
    {
      return -1;
    }
    else
    {
      return 1;
    }
  }
  // Second and third quadrants
  else if ((delta_y > 0 && delta_x < 0) || (delta_y < 0 && delta_x < 0))
  {
    if (check <= 0)
    {
      return 1;
    }
    else
    {
      return -1;
    }
  }
}

/**
 * Calculate the global cost for each path candidate
 *
 * @param None
 * @return None
 */
void DynamicPlanning::setGlobalCost()
{
  double sum = 0.0;
  std::vector<double> offsets(candidates_.size(), 0.0);
  for(int i = 0; i < candidates_.size(); i++)
  {
    double offset = candidates_[i].sp(s_end_);
    // sum += offset * offset;
    // offsets[i] = offset * offset;
    sum += abs(offset);
    offsets[i] = abs(offset);
  }
  // Normalize
  for(int i = 0; i < offsets.size(); i++)
  {
    candidates_[i].gc = offsets[i] / sum;
  }
}

void DynamicPlanning::setR(int idx, bool is_obs, std::vector<path_msgs::Lane> contact_lanes)
{
  // Set obstacle R value
  if (is_obs)
  {
    R_[idx] = 1.0;
  }
  else // Set lane R value
  {
    for(int l = 0; l < contact_lanes.size(); l++)
    {
      int lane_code = contact_lanes[l].lane_code;
      int lane_type = contact_lanes[l].lane_type;
      int line_type = lane_type % 10;

      // No Parking line
      if (lane_code == LCode::UTURN || lane_code == LCode::NO_PARKING || lane_code == LCode::NO_STOPPING)
      {
        R_[idx] = 1.0;
      }
      // Center line
      else if (lane_code == LCode::CENTER)
      {
        R_[idx] = 1.0;
      }
      // Barriers
      else if (lane_code == LCode::BARRIER)
      {
        R_[idx] = 1.0;
      }
    }
  }
}

/**
 * Set cost for avoiding static objects, incl. lanes and objects from detection
 *
 * @param None
 * @return None
 */
void DynamicPlanning::setObstacleCost()
{
  // Get adjacent lanes
  std::tuple<std::vector<path_msgs::Lane>, std::vector<path_msgs::Lane>> adj_lanes = getAdjacentLanes(ego_.link_id);
  std::vector<path_msgs::Lane> llanes = std::get<0>(adj_lanes);
  std::vector<path_msgs::Lane> rlanes = std::get<1>(adj_lanes);
  std::vector<path_msgs::Lane> all_adj_lanes = llanes;
  all_adj_lanes.insert(all_adj_lanes.end(), rlanes.begin(), rlanes.end());
  
  // std::vector<path_msgs::Lane> all_adj_lanes;
  // DEBUG
  path_msgs::Map local_map;
  local_map.OffsetMapX = offsetX_;
  local_map.OffsetMapY = offsetY_;
  for(int i = 0; i < all_adj_lanes.size(); i++)
  {
    local_map.lanes.push_back(all_adj_lanes[i]);
    printf("%s\n", all_adj_lanes[i].id.c_str());
  }
  adj_lanes_pub_.publish(local_map);

  // reset collision risk R
  std::fill(R_.begin(), R_.end(), 0.0);

  // check each candidate whether it collides with static or dynamic objects
  for (int j = 0; j < candidates_.size(); j++)
  {
    checkPath(candidates_[j].points, all_adj_lanes, j);
  }

  // set static and dynamic cost
  for(int i = 0; i < candidates_.size(); i++)
  {
    double sc = 0.0; // static cost
    double dc = 0.0; // dynamic cost

    // printf("Calculating static cost...\n");
    // Gaussian convolution for R
    for(int k = -path_size_; k <= path_size_; k++)
    {
      double Rki;
      double g = 0.0;
      if (k+i < 0 || k+i > 2*path_size_)
      {
        Rki = 1.0;
      }
      else
      {
        Rki = R_[k+i];
      }
      if (Rki == 1.0)
      {
        g = 1.0 / sqrt(2*PI*sigma_) * exp(-pow(k,2) / (2*pow(sigma_, 2)));
      }
      // if (Rki == 0.5)
      // {
      //   g = 1.0 / sqrt(2*PI*sigma_) * exp(-pow(k,2) / (2*pow(sigma_, 2)));
      // }
      // else if (Rki == 1.0)
      // {
      //   g = 1.0 / sqrt(2*PI*sigma_*3) * exp(-pow(k,2) / (2*pow(sigma_*2.0, 2)));
      // }
      sc += g * Rki;
    }
    // printf("Cand %d: sc: %f\n", i, sc);

    // printf("Calculating dynamic cost...\n");

    // Collision check for dynamic objects
    obj_msgs::Obj collision_obj = candidates_[i].collision_obj;
    if (collision_obj.type == OType::DYNAMIC && ego_speed_ > 0.0)
    {
      double s = candidates_[i].collision_len; // length of path to the collision point
      double obs_ttc = candidates_[i].time_to_collision; // time for an obstacle to reach collision point
      double ego_ttc = s / ego_speed_; // time for ego to reach collision point
      double delta = obs_ttc - ego_ttc;
      double accel = 0.0;

      if (obs_ttc == 0.0) // collide from the beginning
      {
        // printf("Collison from the beginning\n");
        R_[i] = 1.0; // remove from candidates
        dc = 1000000.0; // temp
      }
      else
      {
        if (delta > 0)
        {
          // printf("Cutting in...\n");
          if (s+L_cutin_ < ego_speed_*obs_ttc)
          {
            accel = 0;
          }
          else
          {
            accel = 2 * (s+L_cutin_-ego_speed_*obs_ttc) / (obs_ttc*obs_ttc);
          }
          if (accel > max_accel_)
          {
            // printf("Too much acceleration required\n");
            // R_[i] = 1.0; // remove from candidates
            // dc = 1000000.0; // temp
            dc = abs(max_decel_) * (s+L_cutin_);
            candidates_[i].throttle = -max_decel_;
          }
          else
          {
            dc = abs(accel) * (s + L_cutin_);
            candidates_[i].throttle = accel;
          }
        }
        else
        {
          // printf("Following...\n");
          if (L_follow_ > s)
          {
            L_follow_ = s;
          }
          accel = 2 * (s-L_follow_-ego_speed_*obs_ttc) / (obs_ttc*obs_ttc);
          if (accel < -max_decel_)
          {
            // printf("Too much deceleration required\n");
            R_[i] = 1.0; // remove from candidates
            dc = 1000000.0; // temp
          }
          else if (accel > max_accel_)
          {
            // printf("Too much acceleration required\n");
            R_[i] = 1.0; // remove from candidates
            dc = 1000000.0; // temp
          }
          else
          {
            dc = abs(accel) * (s - L_follow_);
            candidates_[i].throttle = accel;
          }
        }
      }
        
      // printf("Cand %d: s: %f, obs_ttc: %f, ego_ttc: %f, accel: %f, dc: %f\n", i, s, obs_ttc, ego_ttc, accel, dc);
    }
    candidates_[i].sc = sc;
    candidates_[i].dc = dc;
  }
}

/**
 * Based on HD map, search adjacent lanes relative to the ego position
 *
 * @param ego's current link id
 * @return lanes on the left and lanes on the right
 */
std::tuple<std::vector<path_msgs::Lane>, std::vector<path_msgs::Lane>>
DynamicPlanning::getAdjacentLanes(std::string link_id)
{
  path_msgs::Link clink = links_.at(link_id);
  std::vector<path_msgs::Lane> rlanes;
  std::vector<path_msgs::Lane> llanes;
  std::deque<path_msgs::Link> links;
  std::deque<bool> state;
  links.push_back(clink);
  state.push_back(true);

  while(!(links.size() == 0))
  {
    path_msgs::Link link = links.front();
    bool is_search = state.front();
    links.pop_front();
    state.pop_front();
    LControl lane_status = laneStatus(link);
    // printf("Find current link's lanes\n");
    // Find selected link's lanes
    for(int i = 0; i < link.R_laneID.size(); i++)
    {
      // printf("link's right lane: %s\n", link.R_laneID[i].c_str());
      path_msgs::Lane rlane = lanes_.at(link.R_laneID[i]);
      if(!isSafeLane(rlane))
      {
        rlanes.push_back(rlane);
      }
    }
    for(int i = 0; i < link.L_laneID.size(); i++)
    {
      // printf("link's left lane: %s\n", link.L_laneID[i].c_str());
      path_msgs::Lane llane = lanes_.at(link.L_laneID[i]);
      if(!isSafeLane(llane))
      {
        llanes.push_back(llane);
      }
    }

    // Find selected link's right lane
    if (lane_status == LControl::RIGHT)
    {
      // printf("Add right link's next links' lanes\n");
      path_msgs::Link rlink = links_.at(link.RLID[0]);
      for(int i = 0; i < rlink.R_laneID.size(); i++)
      {
        path_msgs::Lane rlane = lanes_.at(rlink.R_laneID[i]);
        if(!isSafeLane(rlane))
        {
          rlanes.push_back(rlane);
        }
      }
    }
    // Find selected link's left lane
    else if (lane_status == LControl::LEFT)
    {
      // printf("Add left link's next links' lanes\n");
      path_msgs::Link llink = links_.at(link.LLID[0]);
      for(int i = 0; i < llink.L_laneID.size(); i++)
      {
        path_msgs::Lane llane = lanes_.at(llink.L_laneID[i]);
        if(!isSafeLane(llane))
        {
          llanes.push_back(llane);
        }
      }
    }
    // Find selected link's both lane
    else if (lane_status == LControl::BOTH)
    {
      // printf("Add left and right link's next links' lanes\n");
      path_msgs::Link rlink = links_.at(link.RLID[0]);
      path_msgs::Link llink = links_.at(link.LLID[0]);
      for(int i = 0; i < rlink.R_laneID.size(); i++)
      {
        path_msgs::Lane rlane = lanes_.at(rlink.R_laneID[i]);
        if(!isSafeLane(rlane))
        {
          rlanes.push_back(rlane);
        }
      }
      for(int i = 0; i < llink.L_laneID.size(); i++)
      {
        path_msgs::Lane llane = lanes_.at(llink.L_laneID[i]);
        if(!isSafeLane(llane))
        {
          llanes.push_back(llane);
        }
      }
    }

    // Search more next link's lanes
    if (is_search)
    {
      for(int j = 0; j < link.NLIDS.size(); j++)
      {
        if (!link.NLIDS[j].empty())
        {
          // printf("next link id: %s\n", link.NLIDS[j].c_str());
          path_msgs::Link nlink = links_.at(link.NLIDS[j]); // 교차로에서 터짐 (해결)
          links.push_back(nlink);
          // printf("nlink legnth: %f\n", nlink.length);
          if (nlink.length > path_len_)
          {
            state.push_back(false);
          }
          else
          {
            state.push_back(true);
          }
        }
      }
    }
  }

  // DEBUG
  for(int i = 0; i < rlanes.size(); i++)
  {
    // printf("RLANES %s\n", rlanes[i].id.c_str());
  }
  for(int i = 0; i < llanes.size(); i++)
  {
    // printf("LLANES %s\n", llanes[i].id.c_str());
  }
  return std::make_tuple(llanes, rlanes);
}

/**
 * Check whether a waypoint lies inside an ellipse-shaped object
 *
 * @param x, y coordinate of a waypoint, and an ellipse-shaped object
 * @return boolean
 */
bool DynamicPlanning::isCollision(double x, double y, obj_msgs::Obj obj) 
{
  double angle = obj.alpha;
  double a = obj.a;
  double b = obj.b;
  double x0 = obj.pose.position.x;
  double y0 = obj.pose.position.y;
  double equation = ((x-x0)*cos(angle) + (y-y0)*sin(angle))*((x-x0)*cos(angle) + (y-y0)*sin(angle)) / (a*a) +
                      ((x-x0)*sin(angle) - (y-y0)*cos(angle))*((x-x0)*sin(angle) - (y-y0)*cos(angle)) / (b*b);
  return (equation <= 1) ? true : false;
}

void DynamicPlanning::transformObject(const obj_msgs::Obj src, obj_msgs::Obj &dest)
{
  // Find transform
  tf::Transform base_link2map;
  base_link2map.setRotation(tf::Quaternion(ego_.pose.orientation.x,
                                      ego_.pose.orientation.y,
                                      ego_.pose.orientation.z,
                                      ego_.pose.orientation.w));
  base_link2map.setOrigin(tf::Vector3(ego_.pose.position.x,
                                  ego_.pose.position.y,
                                  ego_.pose.position.z));

  // Transform object from local to global frame
  tf::Vector3 lpose;
  tf::Vector3 lvel;
  // tf::Vector3 gvel;
  lpose.setValue(src.pose.position.x, src.pose.position.y, 0.0);

  // Local upper-left, upper-right, bottom-left, bottom-right
  tf::Vector3 lul; tf::Vector3 lur; tf::Vector3 lbl; tf::Vector3 lbr;
  lul.setValue(src.ul.x, src.ul.y, 0.0);
  lur.setValue(src.ur.x, src.ur.y, 0.0);
  lbl.setValue(src.bl.x, src.bl.y, 0.0);
  lbr.setValue(src.br.x, src.br.y, 0.0);

  // Local velocity
  lvel.setValue(src.velocity.x, src.velocity.y, 0.0);

  tf::Vector3 gpose = base_link2map * lpose;
  tf::Vector3 gul = base_link2map * lul;
  tf::Vector3 gur = base_link2map * lur;
  tf::Vector3 gbl = base_link2map * lbl;
  tf::Vector3 gbr = base_link2map * lbr;

  // Velocity transform ignores translation
  base_link2map.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Vector3 gvel = base_link2map * lvel;

  // Transformed object
  dest.pose.position.x = gpose.x();
  dest.pose.position.y = gpose.y();
  dest.velocity.x = gvel.x();
  dest.velocity.y = gvel.y();
  dest.ul.x = gul.x();
  dest.ul.y = gul.y();
  dest.ur.x = gur.x();
  dest.ur.y = gur.y();
  dest.bl.x = gbl.x();
  dest.bl.y = gbl.y();
  dest.br.x = gbr.x();
  dest.br.y = gbr.y();
}

/**
 * Check whether the candidate waypoints collide with static or dynamic objects
 * 
 * @param candidate's waypoints
 * @param adjacent lanes
 * @param index of the candidate path 
 * @return None
 */
void DynamicPlanning::checkPath(std::vector<geometry_msgs::Point> points,
                                std::vector<path_msgs::Lane> adj_lanes,
                                int cand_idx)
{
  bool is_static = false;
  bool is_dynamic = false;

  // reset collision object
  obj_msgs::Obj reset;
  candidates_[cand_idx].collision_obj = reset;
  candidates_[cand_idx].collision_len = 0.0;
  candidates_[cand_idx].time_to_collision = 0.0;

  // check collision with obstacles
  for(int i = 0; i < obs_list_.size(); i++)
  {
    double length = 0.0;
    obj_msgs::Obj obj = obs_list_[i];

    for (int j = 0; j < points.size(); j++)
    {
      double x = points[j].x;
      double y = points[j].y;

      // calculate length of the path
      if (j > 0)
      {
        length += getDistance(x, y, points[j-1].x, points[j-1].y);
      }
      
      // collision check with static objects 
      if (obj.type == OType::STATIC)
      {
        if (isCollision(x, y, obj))
        {
          is_static = true;
          candidates_[cand_idx].collision_obj = obj;
          // printf("Cand: %d, static obj id: %d, obj type: %d, detected\n", cand_idx, obj.id, obj.type);
          break;
        }
      }

      // collision check with dynamic objects 
      else if (obj.type == OType::DYNAMIC)
      {
        double arc_diff = std::get<0>(locateObjectFromEgo(obj));
        double lat_diff = std::get<1>(locateObjectFromEgo(obj));

        // only testing for cut-in and follow (need to be modified in the future)
        if (lat_diff >= HALF_LANE/2.0)
        {
          obj_msgs::Obj tracker = obj;
          tracker.alpha = obj.alpha;          

          /* test */
          // double a;
          // double b;
          // double a_prime;
          // double b_prime;

          // a_prime = obj.a;
          // b_prime = obj.b;
          // b = b_prime - (width_/2.0);
          // a = a_prime * b / b_prime;
          // tracker.a = a;
          // tracker.b = b;
          /* .... */

          // tracker.a = obj.a;
          // tracker.b = obj.b;
          for(double t = 0.0; t <= 10.0; t += 0.1)
          {

            tracker.pose.position.x = obj.pose.position.x + t * obj.velocity.x;
            tracker.pose.position.y = obj.pose.position.y + t * obj.velocity.y;

            if (isCollision(x, y, tracker))
            {
              // printf("Cand: %d, approaching..., len: %f, time: %f\n", cand_idx, length, t);
              is_dynamic = true;
              candidates_[cand_idx].collision_len = length;
              candidates_[cand_idx].collision_obj = tracker;
              candidates_[cand_idx].time_to_collision = t;
              // printf("Cand: %d, Approaching: collision..., index: %d, type: %d, id: %d\n", cand_idx, obj.index, obj.type, obj.id);
              // printf("Collision point -> x: %f, y: %f, ttc: %f\n", points[j].x, points[j].y, t);
              break;
            }
          }
        }
        else
        {
          // printf("Obstacle and ego at the same lane\n");
        }
        if (is_dynamic) break;
      }
    }
  }

  // Check collision with adjacent lanes 
  std::vector<path_msgs::Lane> contact_lanes;
  for(auto it = adj_lanes.begin(); it != adj_lanes.end(); it++)
  {
    // printf("Adj lanes: %s\n", it->id.c_str());
    if (!isSafeLane(*it))
    {
      for(int k = it->geometry.size()-1 ; k >=0; k--)
      {
        for(int j = 0; j < points.size(); j++)
        {
          double x = points[j].x;
          double y = points[j].y;
          double dist = getDistance(x, y, it->geometry[k].x - offsetX_, it->geometry[k].y - offsetY_);
          if (dist < lane_thres_)
          {
            if (std::find(contact_lanes.begin(), contact_lanes.end(), *it) == contact_lanes.end())
            {
              // printf("Cand: %d, contact lanes: %s\n",cand_idx, it->id.c_str());
              contact_lanes.push_back(*it);
            }
          }
        }
      }
    }
  }
  // printf("All adjacent lanes have been searched...\n");
  setR(cand_idx, is_static, contact_lanes);
}

LControl DynamicPlanning::laneStatus(path_msgs::Link clink)
{
  if (clink.LLID[0].empty() && clink.RLID[0].empty())
  {
    // printf("Lane change not available\n");
    return LControl::KEEP;
  }
  else if(!clink.LLID[0].empty() && clink.RLID[0].empty())
  {
    // printf("Left lane available\n");
    return LControl::LEFT;
  }
  else if(!clink.RLID[0].empty() && clink.LLID[0].empty())
  {
    // printf("Right lane available\n");
    return LControl::RIGHT;
  }
  else
  {
    // printf("Both lanes are available\n");
    return LControl::BOTH;
  }
}

void DynamicPlanning::visualizeVelocity(obj_msgs::Obj obj)
{
  visualization_msgs::Marker velocity_marker;
  velocity_marker.header.frame_id = "map";
  velocity_marker.header.stamp = ros::Time::now();
  velocity_marker.id =2000+obj.id;
  velocity_marker.type = visualization_msgs::Marker::ARROW;
  velocity_marker.action = visualization_msgs::Marker::ADD;
  velocity_marker.pose.orientation.w = 1.0;
  velocity_marker.points.resize(2);
  velocity_marker.points[0].x = obj.pose.position.x;
  velocity_marker.points[0].y = obj.pose.position.y;
  velocity_marker.points[0].z = obj.pose.position.z;
  velocity_marker.points[1].x = obj.pose.position.x + obj.velocity.x;
  velocity_marker.points[1].y = obj.pose.position.y + obj.velocity.y;
  velocity_marker.points[1].z = obj.pose.position.z + obj.velocity.z;
  velocity_marker.scale.x = 0.5;
  velocity_marker.scale.y = 1.0;
  velocity_marker.scale.z = 0;
  velocity_marker.color.a = 1.0;
  velocity_markers_.markers.push_back(velocity_marker);
}

void DynamicPlanning::visualizeEllipse(obj_msgs::Obj obj)
{
  visualization_msgs::Marker inner;
  visualization_msgs::Marker outer;
  inner.header.frame_id = map_frame_;
  inner.header.stamp = ros::Time::now();
  inner.id = 50000 + obj.id;
  inner.type = visualization_msgs::Marker::CYLINDER;
  inner.action = visualization_msgs::Marker::ADD;
  inner.lifetime = ros::Duration();

  outer.header.frame_id = map_frame_;
  outer.header.stamp = ros::Time::now();
  outer.id = 60000 + obj.id;
  outer.type = visualization_msgs::Marker::CYLINDER;
  outer.action = visualization_msgs::Marker::ADD;
  outer.lifetime = ros::Duration();

  tf::Quaternion quat;
  quat.setRPY(0, 0, obj.alpha);
  geometry_msgs::Quaternion quat_msg;
  quat_msg.x = quat.x();
  quat_msg.y = quat.y();
  quat_msg.z = quat.z();
  quat_msg.w = quat.w();
  inner.pose.orientation = quat_msg;
  inner.pose.position.x = obj.pose.position.x;
  inner.pose.position.y = obj.pose.position.y;
  inner.pose.position.z = 0.0;
  outer.pose.orientation = quat_msg;
  outer.pose.position.x = obj.pose.position.x;
  outer.pose.position.y = obj.pose.position.y;
  outer.pose.position.z = -0.02;

  // if (obj.type == OType::STATIC)
  // {
  //   double a;
  //   double b;
  //   double a_prime;
  //   double b_prime;

  //   a_prime = obj.a;
  //   b_prime = obj.b;
  //   b = b_prime - (width_/2.0 + 0.3);
  //   a = a_prime * b / b_prime;
  //   marker.scale.x = a * 2.0;
  //   marker.scale.y = b * 2.0;
  //   marker.scale.z = 0.1;
  // }
  // else
  // {
  //   marker.scale.x = obj.a * 2.0;
  //   marker.scale.y = obj.b * 2.0;
  //   marker.scale.z = 0.1;  
  // }

  double a;
  double b;
  double a_prime;
  double b_prime;

  a_prime = obj.a;
  b_prime = obj.b;
  b = b_prime - (width_/2.0);
  a = a_prime * b / b_prime;
  inner.scale.x = a * 2.0;
  inner.scale.y = b * 2.0;
  inner.scale.z = 0.1;
  inner.color.r = 105.0/255.0;
  inner.color.g = 105.0/255.0;
  inner.color.b = 105.0/255.0;
  inner.color.a = 1.0;

  outer.scale.x = obj.a * 2.0;
  outer.scale.y = obj.b * 2.0;
  outer.scale.z = 0.1;  
  
  outer.color.r = 0.75;
  outer.color.g = 0.75;
  outer.color.b = 0.75;
  outer.color.a = 0.5;
  ellipse_markers_.markers.push_back(inner);
  ellipse_markers_.markers.push_back(outer);
}

void DynamicPlanning::visualizeCurrentPose()
{
  visualization_msgs::Marker rviz_pose;

  rviz_pose.header.frame_id = map_frame_;
  rviz_pose.header.stamp = ros::Time::now();
  rviz_pose.action = visualization_msgs::Marker::ADD;
  rviz_pose.pose = ego_.pose;
  rviz_pose.id = 0;
  rviz_pose.type = visualization_msgs::Marker::CUBE;
  rviz_pose.scale.x = length_;
  rviz_pose.scale.y = width_;
  rviz_pose.scale.z = 0.5;
  rviz_pose.color.r = 1.0;
  rviz_pose.color.g = 0.0;
  rviz_pose.color.b = 0.0;
  rviz_pose.color.a = 1.0;

  rviz_current_pose_pub_.publish(rviz_pose);
}

std::tuple<double, double> DynamicPlanning::getSP(obj_msgs::Obj obj)
{
  double cx = obj.pose.position.x;
  double cy = obj.pose.position.y;
  int init_index = obj.index;
  // Quadratic Minimization method
  std::vector<double> ss(4);
  int iteration = 0;
  double sk_last;
  // Initial three
  double init_s = sx_.m_x[init_index];
  ss[0] = init_s - 7.0;
  ss[2] = init_s + 7.0;
  ss[1] = (ss[0] + ss[2]) / 2;
  // std::cout << "Getting lateral" << std::endl;
  while(true)
  {
    double s1 = ss[0];
    double s2 = ss[1];
    double s3 = ss[2];
    double Ds1 = getDistance(sx_(s1), sy_(s1), cx, cy);
    double Ds2 = getDistance(sx_(s2), sy_(s2), cx, cy);
    double Ds3 = getDistance(sx_(s3), sy_(s3), cx, cy);
    double sk = 0.5 * ((pow(s2,2)-pow(s3,2))*Ds1 + (pow(s3,2)-pow(s1,2))*Ds2 + (pow(s1,2)-pow(s2,2))*Ds3)
                / ((s2-s3)*Ds1 + (s3-s1)*Ds2 + (s1-s2)*Ds3);

    double term_cond = 1.0 / (sx_.m_x.size()*1000);
    
    ss[3] = sk;

    if(iteration > 0)
    {
      if (abs(sk_last-sk) < term_cond)
      {
        double lat_offset = getDistance(sx_(sk), sy_(sk), cx, cy);
        lat_offset = getLateralDir(cx, cy, sk) * lat_offset;
        // printf("Lateral offset: %f\n", lat_offset);
        return std::make_tuple(sk, lat_offset);
      }
    }

    sk_last = sk;


    // Eliminate the value which gives the largest P(s) among s1 s2 s3 sk
    double max_P = -1 * std::numeric_limits<double>::infinity();
    int max_idx = 0;

    for(int i = 0 ; i < 4; i++)
    {
      double s = ss[i];
      double P = (s-s2)*(s-s3)*Ds1 / ((s1-s2)*(s1-s3)) +
              (s-s1)*(s-s3)*Ds2 / ((s2-s1)*(s2-s3)) +
                  (s-s1)*(s-s2)*Ds3 / ((s3-s1)*(s3-s2));
      // std::cout << "P: " << P << std::endl;

      if (P > max_P)
      {
        max_P = P;
        max_idx = i;
      }
    }

    // std::cout << max_idx << std::endl;
    if (max_idx < 3)
    {
      ss[max_idx] = sk;
      for (int i = 0; i < 3; i++)
      {
        for (int j=i+1; j < 3; j++)
        {
          if (ss[i] == ss[j])
          {
            if (ss[j] < 0.5) ss[j] = ss[j] + 0.0001;
            else ss[j] = ss[j] - 0.0001;
          }
        }
      }
    }

    if(iteration > 100)
    {
      ROS_INFO_STREAM("Fail to find optimal lateral offset");
      double lat_offset = getDistance(sx_(init_s), sy_(init_s), cx, cy);
      // ROS_INFO("Lateral offset: %f", lat_offset);
      return std::make_tuple(init_s, lat_offset);
    }
    iteration++;
  }
}

void DynamicPlanning::generateCenterLine()
{
  double length = 0.0;
  std::vector<double> x; // lateral offset
  std::vector<double> y; // lateral offset
  std::vector<double> sl; // arc length

  x.push_back(global_trajectory_.waypoints[0].point.x);
  y.push_back(global_trajectory_.waypoints[0].point.y);
  sl.push_back(length);

  for(int i=1; i < global_trajectory_.waypoints.size(); i++)
  {
    geometry_msgs::Point wp = global_trajectory_.waypoints[i].point;
    geometry_msgs::Point prev_wp = global_trajectory_.waypoints[i-1].point;
    double step = sqrt(pow(prev_wp.x-wp.x, 2) + pow(prev_wp.y-wp.y,2));
    if (step > 0.0)
    {
      length += step;
      x.push_back(global_trajectory_.waypoints[i].point.x);
      y.push_back(global_trajectory_.waypoints[i].point.y);
      sl.push_back(length);
    }
  }
  global_path_len_ = length;
  sx_.set_points(sl,x);
  sy_.set_points(sl,y);
}

// Returns arc-length and lateral offset difference b/t an object and ego
// Need to consider the size of the object
std::tuple<double, double> DynamicPlanning::locateObjectFromEgo(obj_msgs::Obj obj)
{
  return (std::make_tuple(obj.s - ego_.s, abs(obj.lat_offset - ego_.lat_offset)));
}

/**
 * Get the variable length of the candidate paths by the vehicle speed 
 *
 * @param None
 * @return desirable path length
 */
double DynamicPlanning::getPathLength()
{
  // double k = 0.666;
  // double length = k * max_path_len_ * (ego_speed_ / ref_speed_);
  // double k = 4.0;
  double length = ego_speed_ / max_decel_ * ego_speed_ + min_path_len_;

  // printf("Path length by speed: %f\n", length);
  // length by obstacles
  for (int i=0; i < obs_list_.size(); i++)
  {
    if(obs_list_[i].index >= ego_.index && obs_list_[i].type == OType::STATIC)
    {
      double arc_diff = obs_list_[i].s - ego_.s;
      if (arc_diff < length && arc_diff <= max_path_len_)
      {
        length = arc_diff;
      }
    }
  }
  // printf("Path length by obstacle: %f\n", length);

  if (length < min_path_len_)
  {
    length = min_path_len_;
  }
  else if (length > max_path_len_)
  {
    length = max_path_len_;
  }
  // printf("Path length: %f\n", length);
  return length;
}

/**
 * Find the waypoint index that corresponds to a certain distance from an index
 *
 * @param spline curve of the global path
 * @param start index
 * @param distance from the start index 
 * @return end index
 */
int DynamicPlanning::distanceToIndex(tk::spline s, int idx, double dist)
{
  double s_start = s.m_x[idx];
  for(int i = idx; i < s.m_x.size(); i++)
  {
    if (s.m_x[i] > s_start + dist)
    {
      // printf("Looking at index %d\n", i);
      return i;
    }
  }
  // printf("No index to look for\n");
  return (s.m_x.size()-1);
}

void DynamicPlanning::findOptimalPath()
{
  double min_cost = 10000.0;
  int min_index = 0;
  bool found = false;

  for(int i=0; i<candidates_.size();i++)
  {
    double cost = wsm_*candidates_[i].smc + wg_*candidates_[i].gc + ws_*candidates_[i].sc + wd_*candidates_[i].dc;
    if (cost < min_cost && R_[i] != 1.0)
    {
      min_cost = cost;
      min_index = i;
      found = true;
    }
    // printf("%d cost: %3f, %3f, %3f %3f --> %3f, %3f\n",
    // i,
    // wsm_*candidates_[i].smc,
    // wg_*candidates_[i].gc,
    // ws_*candidates_[i].sc,
    // wd_*candidates_[i].dc, cost, R_[i]);
  }

  if (!found)
  {
    opt_path_ = path_size_;
    candidates_[opt_path_].throttle = -max_decel_;
  }
  else
  {
    opt_path_ = min_index;
  }
  // printf("Optimal path: %d\n", opt_path_);
}

void DynamicPlanning::setTargetSpeed(double target_speed)
{
  // Publish target speed
  std_msgs::Float64 msg;
  msg.data = target_speed;
  ego_speed_pub_.publish(msg);
}

void DynamicPlanning::setTargetThrottle(double target_acc)
{
  // publish target throttle
  std_msgs::Float64 msg;
  msg.data = target_acc;
  ego_acc_pub_.publish(msg);
}

/*
 * Generate trajectories with different offsets but same length
 * 
 * @param None
 * @return None
 */
void DynamicPlanning::generateDynamicPath()
{
  path_len_ = getPathLength();
  double tangent_angle = getTangentAngle();
  int end_index = distanceToIndex(sx_, ego_.index, path_len_);
  // printf("Path length: %f\n", path_len_);

  // Generate candidate path in s-p coordinate
  s_start_ = sx_.m_x[ego_.index];
  s_end_ = sx_.m_x[end_index];

  // // Generate candidates
  // clock_t start = clock();
  // generateCandidates(offset_, end_index, tangent_angle);
  // convertPathToCartesian();
  // float gc_time = (float)(clock() - start)/CLOCKS_PER_SEC;
  // ROS_WARN("GC: %0.3f s", gc_time);

  // // Calculate costs
  // start = clock();
  // setGlobalCost();
  setObstacleCost();
  // findOptimalPath(); 
  // double target_speed = getTargetSpeed(opt_path_);
  // double target_throttle = getTargetThrottle(opt_path_);
  // setTargetSpeed(target_speed);
  // setTargetThrottle(target_throttle);
  // float ps_time = (float)(clock() - start)/CLOCKS_PER_SEC;
  // ROS_WARN("Cost: %0.3f s", ps_time);

  // outCSV(std::make_tuple(gc_time, ps_time), "/home/euigon/sim_ws/src/autonomous-car/simulation/simulation/src/50.csv");

  // publishReferencePath(candidates_[opt_path_].points);
  // visualizePathCandidate(opt_path_);
}

std::string DynamicPlanning::getLinkID(obj_msgs::Obj obj)
{
  geometry_msgs::Point position = obj.pose.position;

  std::string link_id = global_trajectory_.waypoints[obj.index].CLID;
  assert(!link_id.empty());
  path_msgs::Link clink = links_.at(link_id);
  int clink_idx = searchLinkIndex(position, clink);
  // printf("Link id: %s, index: %d\n", link_id.c_str(), clink_idx);
  double cdist = getDistance(position.x,
                            position.y,
                            clink.geometry[clink_idx].x - offsetX_,
                            clink.geometry[clink_idx].y - offsetY_);

  if (cdist >= locate_range_)
  {
    return "";
  }


  if (!clink.RLID[0].empty())
  {
    path_msgs::Link rlink = links_.at(clink.RLID[0]);
    int rlink_idx = searchLinkIndex(position, rlink);
    double rdist = getDistance(position.x,
                              position.y,
                              rlink.geometry[rlink_idx].x - offsetX_,
                              rlink.geometry[rlink_idx].y - offsetY_);
    // printf("rdist: %f\n", rdist);
    if (rdist < cdist)
    {
      // printf("AT RIGHT LANE\n");
      link_id = clink.RLID[0];
    }
    // else printf("AT CURRENT LANE\n");
  }

  if (!clink.LLID[0].empty())
  {
    path_msgs::Link llink = links_.at(clink.LLID[0]);
    int llink_idx = searchLinkIndex(position, llink);
    double ldist = getDistance(position.x,
                              position.y,
                              llink.geometry[llink_idx].x - offsetX_,
                              llink.geometry[llink_idx].y - offsetY_);
    // printf("ldist: %f\n", ldist);
    if (ldist < cdist)
    {
      // printf("AT LEFT LANE\n");
      link_id = clink.LLID[0];
    }
    // else printf("AT CURRENT LANE\n");
  }

  return link_id;
}

double DynamicPlanning::getBezierSpeed()
{
  int start_point = 0;
  bool is_bezier = false;
  double length = 0.0;
  for(int i = ego_.index; i < global_trajectory_.waypoints.size()-1; i++)
  {
    path_msgs::Waypoint curr_wp = global_trajectory_.waypoints[i];
    path_msgs::Waypoint next_wp = global_trajectory_.waypoints[i+1];
    // Waypoint is within bezier curve
    if (curr_wp.type == 1)
    {
      if (!is_bezier)
      {
        start_point = i;
        is_bezier = true;
      }
      if (next_wp.type == 1)
      {
        length += getDistance(curr_wp.point.x, curr_wp.point.y, next_wp.point.x, next_wp.point.y);
      }
      else
      {
        break;
      }
    }
  }
  // printf("Bezier length: %f\n", length);
  // Set speed if bezier is found
  if (is_bezier)
  {
    double distance = sx_.m_x[start_point] - sx_.m_x[ego_.index];
    double speed = global_trajectory_.waypoints[start_point].speed*1000.0/3600.0 / 1.5;
    // double speed_diff = ego_speed_ - speed;
    double speed_diff = (ego_speed_ - speed);
    double steady_state_time = 6.0;
    double threshold = 20.0;
    if (speed_diff > 0)
    {
      threshold = steady_state_time * speed_diff;
    }
    // printf("Bezier threshold: %f, distance: %f\n", threshold, distance);
    if (distance < threshold)
    {
      // printf("Bezier speed: %f\n", speed);
      return speed;
    }
    else
    {
      return ref_speed_;
    }
  }
  else
  {
    return ref_speed_;
  }
}


double DynamicPlanning::getSpeedLimit()
{
  path_msgs::Link current_link = links_.at(ego_.link_id);
  // Currently not at the turn, take next link's speed
  // printf("Current link type: %d\n", current_link.road_type);
  if (current_link.road_type != RType::TURN && current_link.road_type != RType::ATYPICAL_TURN)
  {
    for(int i=ego_.index; i < global_trajectory_.waypoints.size(); i++)
    {
      if (global_trajectory_.waypoints[i].CLID == searchNextLink(ego_.index))
      {
        path_msgs::Link next_link = links_.at(global_trajectory_.waypoints[i].CLID);
        // printf("%s\n", global_trajectory_.waypoints[i].CLID.c_str());
        double distance = sx_.m_x[i] - sx_.m_x[ego_.index];
        double speed = global_trajectory_.waypoints[i].speed * 1000.0 / 3600.0;
        // double speed_diff = ego_speed_ - speed;
        // double steady_state_time = 4.0;
        double speed_diff = (ego_speed_ - speed);
        // printf("Ego speed: %f, target speed: %f\n", ego_speed_, speed);
        double steady_state_time = 8.0;
        double threshold = 20.0;

        if (speed_diff > 0)
        {
          threshold = steady_state_time * speed_diff;
        }
        if (next_link.road_type == RType::ATYPICAL_TURN)
        {
          threshold = 15.0;
        }
        else if (next_link.road_type == RType::TURN)
        {
          threshold = 50.0;
        }
        // printf("Threshold %f, distance: %f\n", threshold, distance);
        if (distance < threshold)
        {
          // printf("Speed: %f\n", speed);
          return speed;
        }
        else
        {
          break;
        }
      }
    }
  }
  // At the turn, take current link's speed
  else
  {
    // printf("Following current link's speed\n");
    return current_link.speed * 1000/3600.0;
  }
  // printf("Following current link's speed\n");
  return current_link.speed * 1000/3600.0;
}

double DynamicPlanning::getCurvatureSpeed(int index)
{
  return sqrt(lat_acc_limit_ / (candidates_[index].max_k));
}

double DynamicPlanning::getTargetThrottle(int index)
{
  return candidates_[index].throttle;
}

double DynamicPlanning::getTargetSpeed(int index)
{
  std::vector<double> speeds;
  // Speed based on curvature
  double curv_v = getCurvatureSpeed(index);
  // Speed based on the static cost of a path (avoidance)
  // double avoid_v = getAvoidSpeed(index);
  // sign limit
  double limit_v = getSpeedLimit();
  // Speed for bezier curve
  double bezier_v = getBezierSpeed();
  // cut-in speed if we goes into the global path from outside
  // double cutin_v = getCutInSpeed();
  speeds.push_back(curv_v);
  // speeds.push_back(avoid_v);
  speeds.push_back(limit_v);
  speeds.push_back(bezier_v);
  // speeds.push_back(cutin_v);
  double min_speed = *min_element(speeds.begin(), speeds.end());
  // printf("curv_v: %f\nbezier_v: %f\nlimit_v: %f\n min: %f\n", curv_v, bezier_v,limit_v,min_speed);
  return min_speed;
}

/**
 * Generate path candidates by different offset
 *
 * @param offset
 * @param end index
 * @param tangent angle to the global path 
 * @return None
 */
void DynamicPlanning::generateCandidates(double min_offset, int end_index, double tangent_angle)
{
  candidates_.clear();
  // int count_s = 7;
  // int count_m = 5;
  // int count_l = 3;
  // double inter_s = min_offset;
  // double inter_m = min_offset * 4;
  // double inter_l = min_offset * 8;
  int count_s = 35;
  int count_m = 0;
  int count_l = 0;
  double inter_s = min_offset;
  double inter_m = min_offset;
  double inter_l = min_offset;

  double coverage = count_s * inter_s + count_m * inter_m + count_l * inter_l;
  double remainder = coverage;
  for(int i = 0; i < 2*path_size_+1; i++)
  {
    if (i < count_l)
    {
      candidates_.push_back(generatePathCandidate(remainder, end_index, tangent_angle));
      remainder -= inter_l;
    }
    else if (i < count_l + count_m)
    {
      candidates_.push_back(generatePathCandidate(remainder, end_index, tangent_angle));
      remainder -= inter_m;
    }
    else if (i < 2*count_s + count_m + count_l)
    {
      candidates_.push_back(generatePathCandidate(remainder, end_index, tangent_angle));
      remainder -= inter_s;
    }
    else if (i < 2*(count_s + count_m) + count_l)
    {
      candidates_.push_back(generatePathCandidate(remainder, end_index, tangent_angle));
      remainder -= inter_m;
    }
    else if (i <= 2*(count_s + count_m + count_l))
    {
      candidates_.push_back(generatePathCandidate(remainder, end_index, tangent_angle));
      remainder -= inter_l;
    }
  }
}

/**
 * Generate a path candidate with an offset, end index and tangent angle
 *
 * @param offset
 * @param end index
 * @param tangent angle to the global path 
 * @return candidate path
 */
Candidate DynamicPlanning::generatePathCandidate(double offset, int safety_index, double tangent_angle)
{
  Candidate cand;
  tk::spline sp;
  std::vector<double> p(2); std::vector<double> s(2);
  p[0] = ego_.lat_offset;
  p[1] = ego_.lat_offset + offset;
  s[0] = s_start_;
  s[1] = s_end_;
  sp.set_boundary(tk::spline::first_deriv, tan(tangent_angle), tk::spline::first_deriv, 0, false);
  sp.set_points(s,p);
  cand.sp = sp;
  return cand;
}

/**
 * Convert path candidates in s-p coordinate to cartesian, and
 * at the same time, calculate smoothness cost by integrating 
 * the squared curvature along the path
 * 
 * @param None
 * @return None
 */
void DynamicPlanning::convertPathToCartesian()
{
  double cx = ego_.pose.position.x;
  double cy = ego_.pose.position.y;

  // Convert from s-p to cartesian
  for(int i=0; i < candidates_.size(); i++)
  {
    // clock_t start = clock();
    tk::spline cand_sp = candidates_[i].sp;
    double point_x = cx;
    double point_y = cy;
    double yaw = yaw_;
    double curv_cost = 0.0;
    double max_k = 0.0;
    double sum_k = 0.0;

    for(double s0 = s_start_; s0 <= s_end_; s0 = s0 + ds_)
    {
      double x0 = sx_(s0); double y0 = sy_(s0);
      double x1 = sx_.deriv(1,s0); double y1 = sy_.deriv(1,s0);
      double x2 = sx_.deriv(2,s0); double y2 = sy_.deriv(2,s0);
      double k0 = (x1*y2-x2*y1) / pow(x1*x1 + y1*y1, 1.5);
      sum_k += k0;
      double p0 = cand_sp(s0); double p1 = cand_sp.deriv(1,s0); double p2 = cand_sp.deriv(2,s0);
      double A = sqrt(p1*p1 + (1-p0*k0)*(1-p0*k0));
      double B = ((1-p0*k0) > 0 ? 1 : -1);
      if (1-p0*k0 == 0) {B=0;}
      double k = B/A * (k0 + ((1-p0*k0)*p2 + k0*p1*p1)/(A*A));

      // Insert point
      geometry_msgs::Point point;
      point.x = point_x;
      point.y = point_y;
      candidates_[i].points.push_back(point);

      // Propagate to next step
      point_x = point_x + A*B*cos(yaw)*ds_;
      point_y = point_y + A*B*sin(yaw)*ds_;
      yaw = yaw + A*B*k*ds_;

      // Find max k
      if (abs(k) > max_k)
      {
        max_k = abs(k);
      }
      // Smoothness cost
      curv_cost += k*k*ds_;
    }
    candidates_[i].max_k = max_k;
    candidates_[i].smc = curv_cost;
    // float cand_time = (float)(clock() - start)/CLOCKS_PER_SEC;
    // ROS_WARN("cand time: %.6fs \n", cand_time);
  }
}


void DynamicPlanning::visualizePathCandidate(int optimal_index)
{

  // Visualize path candidates
  for(int i=0; i < candidates_.size(); i++)
  {
    visualization_msgs::Marker rviz_path;
    rviz_path.header.frame_id = map_frame_;
    rviz_path.header.stamp = ros::Time::now();
    rviz_path.action = visualization_msgs::Marker::ADD;
    rviz_path.pose.orientation.w = 1.0;
    rviz_path.id = i;
    rviz_path.type = visualization_msgs::Marker::LINE_STRIP;
    rviz_path.scale.x = 0.05;
    rviz_path.color.r = 0.0;
    rviz_path.color.g = 1.0;
    rviz_path.color.b = 0.0;
    rviz_path.color.a = 1.0;
    //rviz_path.lifetime = ros::Duration(0.2);

    // No way to go
    if (R_[i] >= 0.5)
    {
      rviz_path.color.r = 1.0;
      rviz_path.color.g = 0.0;
      rviz_path.color.b = 0.0;
    }

    for(int j=0; j < candidates_[i].points.size(); j++)
    {
      // geometry_msgs::Point point;
      // point.x = std::get<0> (candidates_[i].points[j]);
      // point.y = std::get<1> (candidates_[i].points[j]);
      rviz_path.points.push_back(candidates_[i].points[j]);
    }

    if (i == optimal_index)
    {
      rviz_path.scale.x = 0.3;
      rviz_path.color.r = 0.0;
      rviz_path.color.g = 0.0;
      rviz_path.color.b = 1.0;
    }
    publishers_.at(i).publish(rviz_path);
  }
}

void DynamicPlanning::publishReferencePath(std::vector<geometry_msgs::Point> path)
{
  visualization_msgs::Marker rviz_path;
  rviz_path.header.frame_id = map_frame_;
  rviz_path.header.stamp = ros::Time::now();
  rviz_path.action = visualization_msgs::Marker::ADD;
  rviz_path.pose.orientation.w = 1.0;
  rviz_path.id = 999;
  rviz_path.type = visualization_msgs::Marker::LINE_STRIP;
  rviz_path.scale.x = 0.2;
  rviz_path.color.r = 0.0;
  rviz_path.color.g = 0.0;
  rviz_path.color.b = 1.0;
  rviz_path.color.a = 1.0;

  for(int i = 0; i < path.size(); i++)
  {
    rviz_path.points.push_back(path[i]);
  }
  ref_path_pub_.publish(rviz_path);
}

void DynamicPlanning::printObjStatus()
{
  // printf("==============================%d Vehicles===========================\n", obs_list_.size());
  for(int i = 0; i < obs_list_.size(); i++)
  {
    obj_msgs::Obj obj = obs_list_[i];
    // printf("Veh %d, type: %d, index %d, arc %f, lat offset %f m, x: %f, y: %f\n", obj.id, obj.type, obj.index, obj.s, obj.lat_offset, obj.pose.position.x, obj.pose.position.y);
  }
  // printf("================================================================\n");
}

bool DynamicPlanning::isSafeLane(path_msgs::Lane lane)
{
  return (lane.lane_code != LCode::BARRIER &&
    lane.lane_code != LCode::CENTER &&
    lane.lane_code != LCode::UTURN &&
    lane.lane_code != LCode::NO_LANE_CHANGE &&
    lane.lane_code != LCode::NO_PARKING &&
    lane.lane_code != LCode::NO_STOPPING);
}

std::string DynamicPlanning::searchNextLink(int index)
{
  std::string link_id = global_trajectory_.waypoints[index].CLID;
  std::string llink_id = links_.at(link_id).LLID[0];
  std::string rlink_id = links_.at(link_id).RLID[0];

  // 물체 위치로 부터 30 m 뒤까지만 next link를 체크한다
  for(int i = index; i < global_trajectory_.waypoints.size(); i++)
  {
    std::string searched = global_trajectory_.waypoints[i].CLID;
    if (searched != link_id && searched != llink_id && searched != rlink_id)
    {
      return searched;
    }
  }
  return "";
}

int DynamicPlanning::searchTrajIndex(obj_msgs::Obj obj, path_msgs::Trajectory traj, double locate_range)
{
  double x = obj.pose.position.x;
  double y = obj.pose.position.y;
  int min_idx = 0;
  double min_dist = 99999;
  int start_idx = 0;
  int end_idx = 0;

  start_idx = ego_.index;

  if (ego_.index + 1000 >= global_trajectory_.waypoints.size())
  {
    end_idx = global_trajectory_.waypoints.size() - 1;
  }
  else
  {
    end_idx = ego_.index + 1000;
  }

  path_msgs::Trajectory temp;
  // printf("start_idx: %d, end_idx: %d, size: %d\n", start_idx, end_idx, global_trajectory_.waypoints.size());
  for (int i = start_idx; i <= end_idx; i++)
  {
    // printf("x: %f, y: %f\n", x, y);
    // printf("traj_x: %f, traj_y: %f\n", traj.waypoints[i].point.x, traj.waypoints[i].point.y);
    if (i < global_trajectory_.waypoints.size())
    {
      temp.waypoints.push_back(global_trajectory_.waypoints[i]);

      double dist = getDistance(x, y, traj.waypoints[i].point.x, traj.waypoints[i].point.y);
      // printf("id: %d, dist: %f\n", obj.id, dist);
      if (dist < min_dist)
      {
        min_dist = dist;
        min_idx = i;
      }
    }
    else
    {
      break;
    }
  }

  test_path_pub_.publish(temp);

  if (min_dist < locate_range_) {
    // printf("obj id: %d\n", obj.id);
    // printf("min_dist: %f, min_idx: %d\n", min_dist, min_idx);
    return min_idx;
  } else {
    // printf("obj id: %d\n", obj.id);
    // printf("min_dist: %f, min_idx: %d\n", min_dist, min_idx);
    return -1;
  }
}

int DynamicPlanning::searchLinkIndex(geometry_msgs::Point point, path_msgs::Link link)
{
  int min_idx = 0;
  double min_dist = getDistance(point.x, point.y, link.geometry[0].x - offsetX_, link.geometry[0].y - offsetY_);

  for (int i = 1; i < link.geometry.size(); i++){
    double dist = getDistance(point.x, point.y, link.geometry[i].x - offsetX_, link.geometry[i].y - offsetY_);
    if (dist < min_dist){
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

int DynamicPlanning::searchLinkIndexByLength(path_msgs::Link link, int from_idx, int to_idx, double refer_length)
{

    double length = 0;
    int index = 0;

    if (to_idx > from_idx){
        for(int i = from_idx; i < to_idx; i++){
            length += getDistance(link.geometry[i].x, link.geometry[i].y, link.geometry[i+1].x, link.geometry[i+1].y);
            if(length >= refer_length){
                index = i;
                break;
            }
        }
    }
    else if(from_idx > to_idx){
        for(int i = from_idx; i > to_idx; i--){
            length += getDistance(link.geometry[i].x, link.geometry[i].y, link.geometry[i-1].x, link.geometry[i-1].y);
            if(length >= refer_length){
                index = i;
                break;
            }
        }
    }

    return index;
}

double DynamicPlanning::calcLength(path_msgs::Link link, int from_idx, int to_idx)
{
  double length = 0;

  if (to_idx > from_idx){
      for(int i = from_idx; i < to_idx; i++){
          length += getDistance(link.geometry[i].x, link.geometry[i].y, link.geometry[i+1].x, link.geometry[i+1].y);
      }
  }
  else if(from_idx > to_idx){
      for(int i = from_idx; i > to_idx; i--){
          length += getDistance(link.geometry[i].x, link.geometry[i].y, link.geometry[i-1].x, link.geometry[i-1].y);
      }
  }
  else{
      length = 0;
  }

  return length;
}

/**
 * Calculate angle difference between the two angles
 *
 * @param two angles to substract
 * @return angle difference
 */
double DynamicPlanning::getAngleDiff(double first_angle, double second_angle)
{
  double diff = first_angle - second_angle;
  if(abs(diff) >= PI)
  {
    double sign = (diff >= 0 ? 1.0 : -1.0);
    diff = diff - (sign * 2 * PI);
  }
  return diff;
}

double DynamicPlanning::getDistance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}



obj_msgs::Obj DynamicPlanning::getEllipse(obj_msgs::Obj obj, geometry_msgs::Point center, double scale, bool elongated)
{
  obj_msgs::Obj ellipse = obj;
  geometry_msgs::Point upper_mid;
  geometry_msgs::Point bottom_mid;
  geometry_msgs::Point right_mid;
  geometry_msgs::Point left_mid;
  upper_mid.x = (obj.ul.x + obj.ur.x) / 2.0;
  upper_mid.y = (obj.ul.y + obj.ur.y) / 2.0;
  bottom_mid.x = (obj.bl.x + obj.br.x) / 2.0;
  bottom_mid.y = (obj.bl.y + obj.br.y) / 2.0;
  right_mid.x = (obj.ur.x + obj.br.x) / 2.0;
  right_mid.y = (obj.ur.y + obj.br.y) / 2.0;
  left_mid.x = (obj.ul.x + obj.bl.x) / 2.0;
  left_mid.y = (obj.ul.y + obj.bl.y) / 2.0;
  double ub_axis = getDistance(upper_mid.x, upper_mid.y, bottom_mid.x, bottom_mid.y) / 2.0;
  double lr_axis = getDistance(right_mid.x, right_mid.y, left_mid.x, left_mid.y) / 2.0;

  // Enlarge
  double a = ub_axis;
  double b = lr_axis;
  double angle = atan2(upper_mid.y - obj.pose.position.y, upper_mid.x - obj.pose.position.x);

  // if (obj.type == OType::STATIC)
  // {
  //   double a_prime;
  //   double b_prime;
  //   b_prime = b + width_ / 2.0 + 0.3;
  //   a_prime = a / b * b_prime;
  //   a_prime *= scale;
  //   ellipse.a = a_prime;
  //   ellipse.b = b_prime;
  // }
  // else
  // {
  //   ellipse.a = a;
  //   ellipse.b = b;
  // }


  double a_prime;
  double b_prime;
  b_prime = b + (width_ / 2.0);
  a_prime = a / b * b_prime;
  a_prime *= scale;
  ellipse.a = a_prime;
  ellipse.b = b_prime;

  ellipse.alpha = angle;
  ellipse.pose.position.x = center.x;
  ellipse.pose.position.y = center.y;
  return ellipse;
}