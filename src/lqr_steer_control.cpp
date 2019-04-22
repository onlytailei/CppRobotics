/*************************************************************************
	> File Name: lqr_steer_control.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Wed Apr 17 11:48:46 2019
 ************************************************************************/

#include <iostream>
#include <limits>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/time.h>
#include "cubic_spline.h"
#include "motion_model.h"
#include "cpprobotics_types.h"


using namespace cpprobotics;

Vec_f calc_speed_profile(Vec_f rx, Vec_f ry, Vec_f ryaw, float target_speed){
	Vec_f speed_profile(ryaw.size(), target_speed);

	float direction = 1.0;
	for(unsigned int i; i < ryaw.size()-1; i++){
		float dyaw = std::abs(ryaw[i+1] - ryaw[i]);
		float switch_point = (M_PI/4.0<-dyaw) && (dyaw<M_PI/2.0);

		if (switch_point) direction = direction * -1;
		if (direction != 1.0) speed_profile[i]= target_speed * -1;
		else speed_profile[i]= target_speed;

		if (switch_point) speed_profile[i] = 0.0;
	}

	speed_profile[speed_profile.size()-1] = 0.0;
	return speed_profile;
};

void closed_loop_prediction(Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f ck, Vec_f speed_profile, Poi_f goal){
	float T = 500.0;
	float goal_dis = 0.3;
	float stop_speed = 0.05;

	// State()

	float time = 0.0;
	// float 
};

int main(){
	Vec_f wx({0.0, 6.0,  12.5, 10.0, 7.5, 3.0, -1.0});
	Vec_f wy({0.0, -3.0, -5.0,  6.5, 3.0, 5.0, -2.0});

  Spline2D csp_obj(wx, wy);
	Vec_f r_x;
	Vec_f r_y;
	Vec_f ryaw;
	Vec_f rcurvature;
	Vec_f rs;
	for(float i=0; i<csp_obj.s.back(); i+=0.1){
		std::array<float, 2> point_ = csp_obj.calc_postion(i);
		r_x.push_back(point_[0]);
		r_y.push_back(point_[1]);
		ryaw.push_back(csp_obj.calc_yaw(i));
		rcurvature.push_back(csp_obj.calc_curvature(i));
		rs.push_back(i);
	}

	float target_speed = 10.0 / 3.6;
	Vec_f speed_profile = calc_speed_profile(r_x, r_y, ryaw, target_speed);

}