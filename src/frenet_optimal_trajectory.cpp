/*************************************************************************
	> File Name: frenet_optimal_trajectory.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Wed Apr  3 09:52:17 2019
 ************************************************************************/

#include <iostream>
#include <vector>
#include "cubic_spline.h"
#include "frenet_path.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"

int main(){
	Vec_f wx({0.0, 10.0, 20.5, 35.0, 70.5});
	Vec_f wy({0.0, -6.0, 5.0, 6.5, 0.0});
	std::vector<Poi_f> obstcles{
		{{20.0, 10.0}},
		{{30.0, 6.0}},
		{{30.0, 8.0}},
		{{35.0, 8.0}},
		{{50.0, 3.0}}
	};

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

	return 0; 
}