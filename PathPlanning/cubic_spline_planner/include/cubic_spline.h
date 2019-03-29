/*************************************************************************
	> File Name: csv_reader.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Wed Mar 27 12:49:12 2019
 ************************************************************************/

#ifndef _CUBIC_SPLINE_H
#define _CUBIC_SPLINE_H

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Eigen>

class Spline{
  public:
    std::vector<float> a;
    std::vector<float> b;
    std::vector<float> c;
    std::vector<float> d;
    std::vector<float> x;
    std::vector<float> y;

    Spline(std::vector<float>, std::vector<float>){

    }
  private:

};

class Spline2D{
  public:
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> s;

  private:
    Spline2D(std::vector<float>, std::vector<float>){

    }

};

#endif
