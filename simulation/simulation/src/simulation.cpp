#include "simulation/simulation.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "simulation/MPC.h"
#define PI 3.14159

double deg2rad(double x) {return x*PI/180.0;}
double rad2deg(double x) {return x*180.0/PI;}

// convert from map coordinate to car coordinates
void map2car(double px, double py, double psi, const vector<double>& ptsx_map, const vector<double>& ptsy_map,
             Eigen::VectorXd & ptsx_car, Eigen::VectorXd & ptsy_car){

  for(size_t i=0; i< ptsx_map.size(); i++){
    double dx = ptsx_map[i] - px;
    double dy = ptsy_map[i] - py;
    ptsx_car[i] = dx * cos(-psi) - dy * sin(-psi);
    ptsy_car[i] = dx * sin(-psi) + dy * cos(-psi);
  }
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}


std::tuple<double, double, double, double, double, double> getPose(obj_msgs::Obj obj)
{
  double x = obj.pose.position.x;
  double y = obj.pose.position.y;
  double psi = tf::getYaw(obj.pose.orientation);
  double v = obj.speed;
  double steer_angle = obj.steer_angle;
  double throttle = obj.throttle;

  return std::make_tuple(x, y, psi, v, steer_angle, throttle);
}

void setPose(obj_msgs::Obj &obj, double x, double y, double psi, double v, double steer_angle, double throttle)
{
  obj.pose.position.x = x;
  obj.pose.position.y = y;

  tf::Quaternion quat;
  quat.setRPY(0, 0, psi);
  obj.pose.orientation.x = quat.x();
  obj.pose.orientation.y = quat.y();
  obj.pose.orientation.z = quat.z();
  obj.pose.orientation.w = quat.w();

  obj.speed = v;
  obj.steer_angle = steer_angle;
  obj.throttle = throttle;
}

void outCSV(const std::vector<std::tuple<double, double, double>>data, std::string filename)
{
  ofstream f;
  f.open(filename);

  for (int i = 0; i < data.size(); ++i)
  {
    f << std::get<0>(data[i]) << "," << std::get<1>(data[i]) << "," << std::get<2>(data[i]) << std::endl;
  }
  printf("CSV output complete\n");
}


Simulation::Simulation()
: nh_(), private_nh_("~")
{
  // parameters
  private_nh_.param<double>("initial_x", px_, 4.0);
  private_nh_.param<double>("initial_y", py_, 4.0);
  private_nh_.param<double>("initial_yaw", psi_, 4.0);
  private_nh_.param<double>("initial_speed", v_, 4.0);
  private_nh_.param<double>("max_speed_obs", max_speed_obs_, 4.0);
  private_nh_.param<double>("max_time_seq", max_time_seq_, 4.0);
  private_nh_.param<std::string>("filename", filename_, "filename");

  // publishers and subscribers
  obs_pub_ = private_nh_.advertise<obj_msgs::ObjList>("/obj_info", 1);
  mpc_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("/mpc_pose", 1);
  past_traj_pub_ = private_nh_.advertise<nav_msgs::Path>("/ego_path", 1);
  traj_pub_ = private_nh_.advertise<visualization_msgs::Marker>("/rviz/traj", 1);
  vehicle_state_pub_ = private_nh_.advertise<control_msgs::VehicleState>("/vehicle_state", 1);
  ref_path_sub_ = private_nh_.subscribe("/ref_path", 1, &Simulation::refPathCallback, this);
  target_speed_sub_ = private_nh_.subscribe("/Target_Velocity", 1, &Simulation::targetSpeedCallback, this);
  target_acc_sub_ = private_nh_.subscribe("/target_throttle", 1, &Simulation::targetThrottleCallback, this);

  // global variables in initialization
  path_received_ = false;
  speed_received_ = false;
  steer_angle_ = 0.0; throttle_ = 0.0;
  time_seq_ = 0.0; latency_ = 0.05;
}

Simulation::~Simulation()
{
}

void Simulation::insertObs(obj_msgs::ObjList& obslist, obj_msgs::Obj obs)
{
  for(int i = 0; i < obslist.objlist.size(); i++)
  {
    if(obslist.objlist[i].id == obs.id)
    {
      obslist.objlist.erase(obslist.objlist.begin() + i);
    }
  }
  obslist.objlist.push_back(obs);
}

void Simulation::publishState(obj_msgs::Obj obj)
{
  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time::now();
  ps.header.frame_id = "map";
  ps.pose = obj.pose;
  past_trajectory_.poses.push_back(ps);
  
  traj_.points.push_back(ps.pose.position);
  // publish current pose to dynamic planning
  mpc_pub_.publish(ps);

  // publish speed and acceleration to dynamic planning
  control_msgs::VehicleState msg;
  msg.v_ego = obj.speed;
  msg.a_x = obj.throttle;
  vehicle_state_pub_.publish(msg);

  printf("seq: %f - x: %f, y: %f, psi: %f, v: %f, a: %f, ts: %f\n", time_seq_, obj.pose.position.x, obj.pose.position.y, tf::getYaw(obj.pose.orientation), obj.speed, obj.throttle, target_speed_);
}

void Simulation::solveMPC(std::vector<geometry_msgs::Point> path, obj_msgs::Obj& obj)
{
  MPC mpc;
 
  vector<double> ptsx;
  vector<double> ptsy;

  for(int i = 0; i < path.size(); i++)
  {
    ptsx.push_back(path[i].x);
    ptsy.push_back(path[i].y);
  }

  double dx = obj.pose.position.x;
  double dy = obj.pose.position.y;
  double dpsi = tf::getYaw(obj.pose.orientation);
  double v = obj.speed;
  double steer_angle = obj.steer_angle;
  double throttle = obj.throttle;

  Eigen::VectorXd ptsx_car(ptsx.size());
  Eigen::VectorXd ptsy_car(ptsy.size());
  map2car(dx, dy, dpsi, ptsx, ptsy, ptsx_car, ptsy_car);

  // compute coefficients
  auto coeffs = polyfit(ptsx_car, ptsy_car, 3);

  // car state
  Eigen::VectorXd state(6); // {x, y, psi, v, cte, epsi}

  double Lf = 2.67;
  dx = 0 + v*cos(0)*latency_;
  dy = 0 + v*sin(0)*latency_;
  dpsi = 0 - v / Lf * steer_angle * latency_;
  double epsi = 0 - atan(coeffs[1]) - v / Lf * steer_angle * latency_;
  double cte = polyeval(coeffs, 0) - 0 + v * sin(0-atan(coeffs[1])) * latency_;
  v += throttle * latency_;
  state << dx, dy, dpsi, v, cte, epsi;


  // set target speed and target throttle
  double target_speed;
  double target_throttle;
  if (obj.type == 0) // EGO
  {
    target_speed = target_speed_;
    target_throttle = target_throttle_;
  }
  else // Obstacles
  {
    target_speed = max_speed_obs_;
    target_throttle = 0.0;
  }

  // MPC solve
  auto vars = mpc.Solve(state, coeffs, target_speed, target_throttle, throttle);

  // transform position from baselink to map frame
  tf::Transform baselink2map;
  tf::Quaternion b2m_quat = tf::Quaternion(obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w);
  baselink2map.setRotation(b2m_quat);
  baselink2map.setOrigin(tf::Vector3(obj.pose.position.x, obj.pose.position.y, 0.0));

  tf::Transform baselink_pose;
  tf::Quaternion b_quat;
  b_quat.setRPY(0, 0, dpsi);
  baselink_pose.setRotation(b_quat);
  baselink_pose.setOrigin(tf::Vector3(dx, dy, 0.0));

  tf::Transform map_pose;
  map_pose = baselink2map * baselink_pose;
  obj.pose.position.x = map_pose.getOrigin().x();
  obj.pose.position.y = map_pose.getOrigin().y();
  obj.pose.orientation.x = map_pose.getRotation().x();
  obj.pose.orientation.y = map_pose.getRotation().y();
  obj.pose.orientation.z = map_pose.getRotation().z();
  obj.pose.orientation.w = map_pose.getRotation().w();

  // set MPC solve results
  obj.steer_angle = -vars[0] / deg2rad(mpc.max_steer);
  obj.throttle = vars[1];
  obj.speed = v;
  obj.velocity.x = obj.speed*cos(tf::getYaw(obj.pose.orientation));
  obj.velocity.y = obj.speed*sin(tf::getYaw(obj.pose.orientation));
  printf("x: %f, y: %f, psi: %f, v: %f\n", obj.pose.position.x, obj.pose.position.y, tf::getYaw(obj.pose.orientation), obj.speed);
  printf("Steer value: %f, throttle_value: %f\n", obj.steer_angle, obj.throttle);
}

void Simulation::targetSpeedCallback(std_msgs::Float64 msg)
{
  target_speed_ = msg.data;
  speed_received_ = true;
}

void Simulation::targetThrottleCallback(std_msgs::Float64 msg)
{
  target_throttle_ = msg.data;
}

void Simulation::refPathCallback(visualization_msgs::Marker marker)
{
  ref_path_ = marker.points;
  path_received_ = true;
}

void Simulation::publishPastTrajectory()
{
  past_trajectory_.header.stamp = ros::Time::now();
  past_trajectory_.header.frame_id = "map";
  past_traj_pub_.publish(past_trajectory_);

  // Trajectory visualization
  traj_.header.frame_id = "map";
  traj_.header.stamp = ros::Time::now();
  traj_.action = visualization_msgs::Marker::ADD;
  traj_.pose.orientation.w = 1.0;
  traj_.id =100;
  traj_.type = visualization_msgs::Marker::POINTS;
  traj_.scale.x = 0.3;
  traj_.scale.y = 0.3;
  traj_.color.r = 178.0/255;
  traj_.color.g = 34.0/255;
  traj_.color.b = 34.0/255;
  traj_.color.a = 1.0;
  traj_pub_.publish(traj_);
}

std::vector<geometry_msgs::Point> Simulation::getObsPath(std::string filename)
{
  std::vector<geometry_msgs::Point> path;

  YAML::Node config = YAML::LoadFile(filename);
  const YAML::Node& points = config["points"];
  for (YAML::const_iterator it = points.begin(); it != points.end(); ++it)
  {
    const YAML::Node& point = *it;
    geometry_msgs::Point p;
    p.x = point["x"].as<double>();
    p.y = point["y"].as<double>();
    path.push_back(p);
  }
  return path;
}

std::vector<geometry_msgs::Point> Simulation::getObsPath(obj_msgs::Obj obs)
{
  std::vector<geometry_msgs::Point> path;
  double psi = tf::getYaw(obs.pose.orientation);

  for(int i = 0; i < 2000; i++)
  {
    geometry_msgs::Point point;
    point.x = obs.pose.position.x + i * 0.5 * cos(psi);
    point.y = obs.pose.position.y + i * 0.5 * sin(psi);
    path.push_back(point); 
  }

  return path;
}

void Simulation::run()
{
  ros::Rate loop_rate(10);

  // vectors for recording (timestamp, acceleration, speed)
  std::vector<std::tuple<double, double, double>> data;

  obj_msgs::ObjList obslist;
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped ts;

  // create static obstacles
  // length, width, x, y, heading, velocity, type
  // obslist.objlist.push_back(createObstacle(3.5, 2.7, 203.651, -30.3418, 6.16358, 0.0, 2, 2));
  // obslist.objlist.push_back(createObstacle(3.5, 2.7, 219.166, -28.9435, 6.16358, 0.0, 2, 3));
  // obslist.objlist.push_back(createObstacle(3.5, 2.7, 29.3186, -8.2268, 6.16358, 0.0, 2, 4));
  // obslist.objlist.push_back(createObstacle(3.5, 2.7, 67.7311, -10.2494, 6.16358, 0.0, 2, 5));
  // obslist.objlist.push_back(createObstacle(3.5, 2.7, 115.899, -19.0269, 6.16358, 0.0, 2, 6));
  // obslist.objlist.push_back(createObstacle(3.5, 2.7, 35.0535, -8.77307, 6.16358, 0.0, 2, 2));

  // create dynamic obstacle
  // obj_msgs::Obj dynamic_obs = createObstacle(3.5, 2.7, -18.2489, -2.19789, 6.16358, max_speed_obs_, 1, 1);
  // obj_msgs::Obj dynamic_obs = createObstacle(3.5, 2.7, -52.2285, 5.15011, 6.16358, max_speed_obs_, 1, 1);

  // create path for dynamic obstacle

  // ego와 relative 속도 차이로 구분... --> if approaching, no path elongation, o.w static and path elongation

  // create ego
  obj_msgs::Obj ego;
  ego.type = 0;
  setPose(ego, px_, py_, psi_, v_, steer_angle_, throttle_);

  while (ros::ok())
  {
    if (time_seq_ > max_time_seq_) // end of simulation
    {
      std::string root = "/home/euigon/sim_ws/src/autonomous-car/simulation/simulation/src/";
      std::string path = root + filename_;
      outCSV(data, path);
      break;
    }
    else // start simulation
    {
      if (!path_received_ || !speed_received_)
      {
        printf("Waiting for reference path...\n");
        publishState(ego);
      }
      else
      {
       
        // std::vector<geometry_msgs::Point> obs_path = getObsPath(dynamic_obs);
        solveMPC(ref_path_, ego);
        // solveMPC(obs_path, dynamic_obs);

        // speed difference check
        // checkDynamic(ego, dynamic_obs);
        // insertObs(obslist, dynamic_obs);
        publishState(ego);
        data.push_back(std::make_tuple(time_seq_, ego.speed, ego.throttle));
        time_seq_ += latency_;
      }

      publishPastTrajectory();
      obs_pub_.publish(obslist);
    }
    ros::spinOnce();

    loop_rate.sleep(); 
  }
}

void Simulation::checkDynamic(obj_msgs::Obj ego, obj_msgs::Obj& obs)
{
  geometry_msgs::Vector3 ego_vel;
  geometry_msgs::Vector3 obs_vel;

  ego_vel.x = ego.speed*cos(tf::getYaw(ego.pose.orientation));
  ego_vel.y = ego.speed*sin(tf::getYaw(ego.pose.orientation));
  obs_vel.x = obs.speed*cos(tf::getYaw(obs.pose.orientation));
  obs_vel.y = obs.speed*sin(tf::getYaw(obs.pose.orientation));

  double rel_speed = sqrt((ego_vel.x-obs_vel.x)*(ego_vel.x-obs_vel.x)+(ego_vel.y-obs_vel.y)*(ego_vel.y-obs_vel.y));
  printf("Relative speed: %f\n", rel_speed);
  if (rel_speed < 0.5)
  {
    printf("Dynamic object is too slow...\n");
    obs.type = 2;
  }
  else
  {
    obs.type = 1;
  } 
}


obj_msgs::Obj Simulation::createObstacle(double length, double width, double x, double y, double heading, double speed, int type, int id)
{
  obj_msgs::Obj obs;
  obs.id = id;
  obs.type = type;
  double h = sqrt((length/2.0)*(length/2.0) + (width/2.0)*(width/2.0));
  double beta = atan2(length/2.0, width/2.0);
  double alpha = PI / 2 + heading - beta;

  // Set obstacle's box
  obs.ur.x = x + h*cos(heading) + width/2.0 * sin(alpha);
  obs.ur.y = y + h*sin(heading) - width/2.0 * cos(alpha);

  obs.ul.x = x + h*cos(heading) - width/2.0 * sin(alpha);
  obs.ul.y = y + h*sin(heading) + width/2.0 * cos(alpha);

  obs.br.x = x - h*cos(heading) + width/2.0 * sin(alpha);
  obs.br.y = y - h*sin(heading) - width/2.0 * cos(alpha);

  obs.bl.x = x - h*cos(heading) - width/2.0 * sin(alpha);
  obs.bl.y = y - h*sin(heading) + width/2.0 * cos(alpha);

  // set position
  obs.pose.position.x = x;
  obs.pose.position.y = y;

  // set orientation
  tf::Quaternion quat;
  quat.setRPY(0, 0, heading);
  obs.pose.orientation.x = quat.x();
  obs.pose.orientation.y = quat.y();
  obs.pose.orientation.z = quat.z();
  obs.pose.orientation.w = quat.w();

  // Set velocity
  obs.velocity.x = speed * cos(heading);
  obs.velocity.y = speed * sin(heading);
  obs.speed = speed;

  return obs;
}