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
#include"cpprobotics_types.h"

namespace cpprobotics{

class FrenetPath{
public:
  float cd = 0.0;
  float cv = 0.0;
  float cf = 0.0;

  Vec_f t;
  Vec_f d;
  Vec_f d_d;
  Vec_f d_dd;
  Vec_f d_ddd;
  Vec_f s;
  Vec_f s_d;
  Vec_f s_dd;
  Vec_f s_ddd;

  Vec_f x;
  Vec_f y;
  Vec_f yaw;
  Vec_f ds;
  Vec_f c;

  float max_speed;
  float max_accel;
  float max_curvature;
};

using Vec_Path=std::vector<FrenetPath>;
}
#endif
