/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "motion_model.h"

#define L 1.0
#define DS 0.1
#define CONST_V 3.0  // use a const linear velocity here

int main(){
  State state(0, 0, 0, CONST_V);
  MotionModel motion_model(L, DS, state);
  motion_model.generate_trajectory(6, {0.0, 0.1, 0.2, 0.3, 0.4});
}
