/*************************************************************************
	> File Name: motion_model.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Thu Mar 21 16:06:07 2019
 ************************************************************************/

#ifndef _MOTION_MODEL_H
#define _MOTION_MODEL_H

#include <iostream>
#include <cmath>
// #include <vector>
// #include <array>
// #include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

#define L 1.0
#define DS 0.1
#define CONST_V 3.0  // use a const linear velocity here

class State{

  public:
    float x;
    float y;
    float yaw;
    float v = CONST_V;
    State();
    State(float x_, float y_, float yaw_, float v_=CONST_V);

  private:
    update();

};

State::State(float x_, float y_, float yaw_, float v_){
  x = x_;
  y = y_;
  yaw = yaw_;
  v = v_;
};

#endif
