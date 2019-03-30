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


using Vec_f=std::vector<float>;

Vec_f vec_diff(Vec_f input){
  Vec_f output;
  for(unsigned int i=1; i<input.size(); i++){
    output.push_back(input[i] - input[i-1]);
  }
  return output;
};

Vec_f cum_sum(Vec_f input){
  Vec_f output;
  float temp = 0;
  for(unsigned int i=0; i<input.size(); i++){
    temp += input[i];
    output.push_back(temp);
  }
  return output;
};

class Spline{
  public:
    const Vec_f x;
    const Vec_f y;
    const int nx;
    const Vec_f h;
    const Vec_f a;
    Vec_f b;
    Eigen::VectorXf c;
    Vec_f d;

    // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i

    Spline(Vec_f x_, Vec_f y_):x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_){
      Eigen::MatrixXf A = calc_A();
      Eigen::VectorXf B = calc_B();
      c = A.colPivHouseholderQr().solve(B);


    };
  private:
    Eigen::MatrixXf calc_A(){
      Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nx, nx);
      A(0, 0) = 1;
      for(int i=0; i<nx-1; i++){
        if (i != nx-2){
          A(i+1, i+1) = 2 * (h[i] + h[i+1]);
        }
        A(i+1, i) = h[i];
        A(i, i+1) = h[i];
      }
      A(0, 1) = 0.0;
      A(nx-1, nx-2) = 0.0;
      A(nx-1, nx-1) = 1.0;
      return A;
    };
    Eigen::VectorXf calc_B(){
      Eigen::VectorXf B = Eigen::VectorXf::Zero(nx);
      for(int i=0; i<nx-2; i++){
        B(i+1) = 3.0*(a[i+2]-a[i+1])/h[i+1] - 3.0*(a[i+1]-a[i])/h[i];
      }
      return B;
    };
};

class Spline2D{
  public:
    Vec_f sx;
    Vec_f sy;
    Vec_f s;

    Spline2D(Vec_f x, Vec_f y){
      s = calc_s(x, y);
      Spline sx = Spline(s, x);
      Spline sy = Spline(s, y);
    };
  private:
    Vec_f calc_s(Vec_f x, Vec_f y){
      Vec_f ds;
      Vec_f out_s{0};
      Vec_f dx = vec_diff(x);
      Vec_f dy = vec_diff(y);

      for(unsigned int i=0; i<x.size(); i++){
        ds.push_back(std::sqrt(dx[i]*dx[i] + dy[i]*dy[i]));
      }

      Vec_f cum_ds = cum_sum(ds);
      out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
      return out_s;
    };
};
#endif
