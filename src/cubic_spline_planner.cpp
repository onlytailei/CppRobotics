/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cubic_spline.h"


int main(){
    Vec_f x{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
    Vec_f y{0.7, -6,   5,   6.5, 0.0, 5.0, -2.0};

    Spline2D csp_obj(x, y);
    for(float i=0; i<csp_obj.s.back(); i+=0.1){
      std::array<float, 2> point_ = csp_obj.calc_postion(i);
    }
};
