/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/
#include<iostream>
#include<fstream>
#include<limits>
#include<vector>
#include<sys/time.h>
#include"frenet/frenet.h"

#define SIM_LOOP 50
#define MAX_SPEED  50.0 / 3.6  // maximum speed [m/s]
#define MAX_ACCEL  5.0  // maximum acceleration [m/ss]
#define MAX_CURVATURE  1.0  // maximum curvature [1/m]
#define MAX_ROAD_WIDTH  3.5  // maximum road width [m]
#define D_ROAD_W  0.1  // road width sampling length [m]
#define DT  0.2  // time tick [s]
#define MAXT  3.0  // max prediction time [s]
#define MINT  0.3  // min prediction time [s]
#define TARGET_SPEED  30.0 / 3.6  // target speed [m/s]
#define D_T_S  5.0 / 3.6  // target speed sampling length [m/s]
#define N_S_SAMPLE  5  // sampling number of target speed
#define ROBOT_RADIUS  0.3  // robot radius [m]

#define KJ  0.1
#define KT  0.1
#define KD  1.0
#define KLAT  1.0
#define KLON  1.0


#define PI 3.141592653

float sum_of_power(std::vector<float> value_list){
  float sum = 0;
  for(float item:value_list){
    sum += item*item;
  }
  return sum;
};

Vec_Path calc_frenet_paths(
    float c_speed, float c_d, float c_d_d, float c_d_dd, float s0){
  std::vector<FrenetPath> fp_list;
  for(float di=-1*MAX_ROAD_WIDTH; di<MAX_ROAD_WIDTH; di+=D_ROAD_W){
    for(float Ti=MINT; Ti<MAXT; Ti+=DT){
      FrenetPath fp;
      QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
      for(float t=0; t<Ti; t+=DT){
        // printf("QuinticPolynomial: Ti: %f, t: %f\n", Ti, t);
        fp.t.push_back(t);
        fp.d.push_back(lat_qp.calc_point(t));
        fp.d_d.push_back(lat_qp.calc_first_derivative(t));
        fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
        fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
      }
      // printf("QuinticPolynomial: fpt: %d\n", fp.t.size());
      // printf("QuinticPolynomial: fpd: %d\n", fp.d.size());
      // printf("QuinticPolynomial: fpdd: %d\n", fp.d_d.size());
      // printf("QuinticPolynomial: fpddd: %d\n", fp.d_dd.size());
      // printf("QuinticPolynomial: fpdddd: %d\n", fp.d_ddd.size());

      for(float tv=TARGET_SPEED - D_T_S * N_S_SAMPLE;
          tv < TARGET_SPEED + D_T_S * N_S_SAMPLE;
          tv+=D_T_S){

        FrenetPath fp_bot = fp;
        // printf("QarticPolynomial: start\n");
        QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);

        fp_bot.max_speed = std::numeric_limits<float>::min();
        fp_bot.max_accel = std::numeric_limits<float>::min();
        // printf("QarticPolynomial: fpt: %d\n", fp.t.size());

        for(float t_:fp.t){
          fp_bot.s.push_back(lon_qp.calc_point(t_));
          fp_bot.s_d.push_back(lon_qp.calc_first_derivative(t_));
          fp_bot.s_dd.push_back(lon_qp.calc_second_derivative(t_));
          fp_bot.s_ddd.push_back(lon_qp.calc_third_derivative(t_));
          if(fp_bot.s_d.back() > fp_bot.max_speed){
            fp_bot.max_speed = fp_bot.s_d.back();
          }
          if(fp_bot.s_dd.back() > fp_bot.max_accel){
            fp_bot.max_accel = fp_bot.s_dd.back();
          }
        }
        // printf("QarticPolynomial: s: %d\n", fp_bot.s.size());


        float Jp = sum_of_power(fp.d_ddd);
        float Js = sum_of_power(fp_bot.s_ddd);
        // printf("QarticPolynomial: fp_bot sd size: %d\n", fp_bot.s_d.size());
        // printf("QarticPolynomial: fp_bot d size: %d\n", fp_bot.d.size());

        float ds = (TARGET_SPEED - fp_bot.s_d.back());

        fp_bot.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_bot.d.back(), 2);
        fp_bot.cv = KJ * Js + KT * Ti + KD * ds;
        fp_bot.cf = KLAT * fp_bot.cd + KLON * fp_bot.cv;

        fp_list.push_back(fp_bot);
        // printf("QarticPolynomial: push_back\n");
      }
    }
  }
  return fp_list;
};

void calc_global_paths(Vec_Path & path_list, Spline2D csp){
  for (Vec_Path::iterator path_p=path_list.begin(); path_p!=path_list.end();path_p++){

    double prev_x;
    double prev_y;

    for(unsigned int i=0; i<path_p->s.size(); i++){
      if (path_p->s[i] >= csp.s.back()){
        break;
      }
      std::array<float, 2> poi = csp.calc_postion(path_p->s[i]);
      float iyaw = csp.calc_yaw(path_p->s[i]);
      float di = path_p->d[i];
      float x = poi[0] + di * std::cos(iyaw + M_PI/2.0);
      float y = poi[1] + di * std::sin(iyaw + M_PI/2.0);
      path_p->x.push_back(x);
      path_p->y.push_back(y);
      prev_x = poi[0];
      prev_y = poi[1];
    }

    // printf("path_x size: %d\n", path_p->x.size());

    // if (path_p->x.size() == 1){
    //   float dx = path_p->x[0] - prev_x;
    //   float dy = path_p->y[0] - prev_y;
    //   path_p->yaw.push_back(std::atan2(dy, dx));
    //   path_p->ds.push_back(std::sqrt(dx * dx + dy * dy));
    // }
    // else{
      for(int i=0; i<path_p->x.size()-1; i++){
        float dx = path_p->x[i + 1] - path_p->x[i];
        float dy = path_p->y[i + 1] - path_p->y[i];
        path_p->yaw.push_back(std::atan2(dy, dx));
        path_p->ds.push_back(std::sqrt(dx * dx + dy * dy));
      }
    // }

    path_p->yaw.push_back(path_p->yaw.back());
    path_p->ds.push_back(path_p->ds.back());

    path_p->max_curvature = std::numeric_limits<float>::min();
    for(int i=0; i<path_p->x.size()-1; i++){
      path_p->c.push_back((path_p->yaw[i+1]-path_p->yaw[i])/path_p->ds[i]);
      if(path_p->c.back() > path_p->max_curvature){
        path_p->max_curvature = path_p->c.back();
      }
    }
  }
};

bool check_collision(FrenetPath path, const std::vector<std::array<float, 2>> ob){
  for(auto point:ob){
    for(unsigned int i=0; i<path.x.size(); i++){
      float dist = std::pow((path.x[i] - point[0]), 2) + std::pow((path.y[i] - point[1]), 2);
      // printf("obstacle dist: %f\n", dist);
      if (dist <= ROBOT_RADIUS * ROBOT_RADIUS){
        return false;
      }
    }
  }
  return true;
};

Vec_Path check_paths(Vec_Path path_list, const std::vector<std::array<float, 2>> ob){
  Vec_Path output_fp_list;
  // printf("check paths fp_list: %d\n", path_list.size());
  for(FrenetPath path:path_list){
    // printf("max speed: %f, max_accel: %f, max_curv: %f\n", path.max_speed, path.max_accel, path.max_curvature);
    if (path.max_speed < MAX_SPEED && path.max_accel < MAX_ACCEL && path.max_curvature < MAX_CURVATURE && check_collision(path, ob)){
      output_fp_list.push_back(path);
    }
  }
  printf("Check paths: %d\n", output_fp_list.size());
  return output_fp_list;
};

FrenetPath frenet_optimal_planning(
  Spline2D csp, float s0, float c_speed,
  float c_d, float c_d_d, float c_d_dd, std::vector<std::array<float, 2>> ob, std::vector<float> &data){

  clock_t start = clock();
  Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
  float cfp = (float)(clock() - start)/CLOCKS_PER_SEC;
  ROS_WARN("CFP: Execution time: %0.3f s", cfp);
  start = clock();
  calc_global_paths(fp_list, csp);
  float cgp = (float)(clock() - start)/CLOCKS_PER_SEC;
  ROS_WARN("CGP: Execution time: %0.3f s", cgp);
  start = clock();
  Vec_Path save_paths = check_paths(fp_list, ob);
  float sp = (float)(clock() - start)/CLOCKS_PER_SEC;
  ROS_WARN("SP: Execution time: %0.3f s", sp);
  data.push_back(cfp+cgp+sp);

  float min_cost = std::numeric_limits<float>::max();
  FrenetPath final_path;
  for(auto path:save_paths){
    if (min_cost >= path.cf){
      min_cost = path.cf;
      final_path = path;
    }
  }
  return final_path;
};
  

void Frenet::publishState(float x, float y, float v)
{
  printf("Publishing current state -- x: %f, y: %f, v: %f\n", x, y, v);

  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time::now();
  ps.header.frame_id = "map";
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  tf::Quaternion quat;
  quat.setRPY(0, 0, 0);
  ps.pose.orientation.x = quat.x();
  ps.pose.orientation.y = quat.y();
  ps.pose.orientation.z = quat.z();
  ps.pose.orientation.w = quat.w();
  pose_pub_.publish(ps);
}

void Frenet::publishLocalPath(std::vector<std::array<float, 2>> ltraj){
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

	// printf("local traj size: %d\n", ltraj.size());
	for(int i = 0; i < ltraj.size(); i++){
		geometry_msgs::Point point;
		point.x = ltraj[i][0];
		point.y = ltraj[i][1];
		marker.points.push_back(point);
	}
  	local_path_pub_.publish(marker);
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

void Frenet::globalCallback(const path_msgs::Trajectory traj_msg)
{
  wx_.clear(); wy_.clear();
  if (traj_msg.waypoints.size() > 0)
    global_initialized_ = true;
  else
    global_initialized_ = false;

  for(int i = 0; i < traj_msg.waypoints.size(); i++)
  {
    if (i % 2 == 0)
    {
      wx_.push_back(traj_msg.waypoints[i].point.x);
      wy_.push_back(traj_msg.waypoints[i].point.y);
    }
  }
  printf("Global path: %d\n", wx_.size());
  global_initialized_ = true;
}

Frenet::Frenet(): nh_(){
	map_initialized_ = false;
  global_initialized_ = false;
  spline_initialized_ = false;
	ros::Rate loop_rate(10);

  map_sub_ = nh_.subscribe("/adj_lanes", 1, &Frenet::mapInfoCallback, this);
  global_path_sub_ = nh_.subscribe("/test_path", 1, &Frenet::globalCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mpc_pose", 1);
  local_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/local_path", 1);

  std::vector<float> r_x;
  std::vector<float> r_y;
  std::vector<float> ryaw;
  std::vector<float> rcurvature;
  std::vector<float> rs;
  
  Spline2D global_spline = Spline2D(std::vector<float>({}), std::vector<float>({}));
  
  float c_speed = 0.1/3.6;
  float c_d = 0.0;
  float c_d_d = 0.0;
  float c_d_dd = 0.0;
  float s0 = 0.0;

  while(ros::ok())
  {
    // Parse global path spline
    if (global_initialized_ && !spline_initialized_){
      global_spline = Spline2D(wx_, wy_);
      for(float i=0; i<global_spline.s.back(); i+=0.1){
        std::array<float, 2> point_ = global_spline.calc_postion(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        ryaw.push_back(global_spline.calc_yaw(i));
        rcurvature.push_back(global_spline.calc_curvature(i));
        rs.push_back(i);
      }
      spline_initialized_ = true;
    }
    else if (map_initialized_ && spline_initialized_)
    {
      local_map_.clear(); //test
      printf("Start of simulation\n");
      std::vector<float> data;
      for(int i=0; i<SIM_LOOP; i++){
        FrenetPath final_path = frenet_optimal_planning(
        global_spline, s0, c_speed, c_d, c_d_d, c_d_dd, local_map_, data);
        // printf("Frenet path calculated\n");
        s0 = final_path.s[1];
        c_d = final_path.d[1];
        c_d_d = final_path.d_d[1];
        c_d_dd = final_path.d_dd[1];
        c_speed = final_path.s_d[1];
        // publish current pose
        // printf("final path size: %d\n", final_path.x.size());
        publishState(final_path.x.front(), final_path.y.front(), c_speed);
        // publish local path
        std::vector<std::array<float, 2>> ltraj;
        for (int j=0; j < final_path.x.size(); j++){
          ltraj.push_back({final_path.x[j], final_path.y[j]});
        }
        publishLocalPath(ltraj);

        if (std::pow((final_path.x[1] - r_x.back()), 2) + std::pow((final_path.y[1]-r_y.back()), 2) <= 1.0){
          break; 
        }
      }
      std::string root = "/home/euigon/sim_ws/src/autonomous-car/planning/frenet/src/";
      std::string path = root + "wo_lanes.csv";
      outCSV(data, path);
      break;
    } else {
      publishState(-5.61911, -3.91944, c_speed);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

Frenet::~Frenet(){}


void Frenet::mapInfoCallback(const path_msgs::Map map_msg)
{

  path_msgs::Map map_info = map_msg;

  local_map_.clear();

  for(int i = 0; i < map_info.lanes.size(); i++)
  { 
    path_msgs::Lane lane = map_info.lanes[i];
    for(int j=0; j < lane.geometry.size(); ++j){
      float x = lane.geometry[j].x - map_info.OffsetMapX;
      float y = lane.geometry[j].y - map_info.OffsetMapY;
      local_map_.push_back({x, y});
    }
  }
  printf("***Map information received***\n");

  map_initialized_ = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frenet_node");
  Frenet node;
  return 0;
}
