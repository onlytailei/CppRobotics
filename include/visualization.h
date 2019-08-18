/*************************************************************************
	> File Name: visualization.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Sat Mar 30 15:51:30 2019
 ************************************************************************/

#ifndef _VISUALIZATION_H
#define _VISUALIZATION_H


#include<iostream>
#include<random>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

namespace cpprobotics{

cv::Point2i cv_offset(
    float x, float y,
    int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + image_width/2;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
};
}
#endif
