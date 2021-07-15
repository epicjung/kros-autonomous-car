/*************************************************************************
	> File Name: frenet_path.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Tue Apr  9 17:08:22 2019
 ************************************************************************/

#ifndef _FRENET_PATH_H
#define _FRENET_PATH_H

#include<iostream>
#include<vector>
#include<array>
#include<string>

class FrenetPath{
public:
  float cd = 0.0;
  float cv = 0.0;
  float cf = 0.0;

  std::vector<float> t;
  std::vector<float> d;
  std::vector<float> d_d;
  std::vector<float> d_dd;
  std::vector<float> d_ddd;
  std::vector<float> s;
  std::vector<float> s_d;
  std::vector<float> s_dd;
  std::vector<float> s_ddd;

  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> yaw;
  std::vector<float> ds;
  std::vector<float> c;

  float max_speed;
  float max_accel;
  float max_curvature;
};

using Vec_Path=std::vector<FrenetPath>;

#endif