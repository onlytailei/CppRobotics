/*************************************************************************
	> File Name: quartic_polynomial.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Tue Apr  2 20:50:42 2019
 ************************************************************************/

#ifndef _QUARTIC_POLYNOMIAL_H
#define _QUARTIC_POLYNOMIAL_H

#include<iostream>
#include<vector>
#include<array>
#include<string>
#include<cmath>
#include<Eigen/Eigen>


namespace cpprobotics{

class QuarticPolynomial{
public:
  // current parameter at t=0
  float xs;
  float vxs;
  float axs;

  // parameters at target t=t_j
  float vxe;
  float axe;

  // function parameters
  float a0, a1, a2, a3, a4;

  QuarticPolynomial(){};

  QuarticPolynomial(float xs_, float vxs_, float axs_, float vxe_, float axe_, float T): xs(xs_), vxs(vxs_), axs(axs_), vxe(vxe_), axe(axe_), a0(xs_), a1(vxs_), a2(axs_/2.0){
    Eigen::Matrix2f A;
    A << 3*std::pow(T, 2),   4*std::pow(T,3),
          6*T,								12*std::pow(T,2);
    Eigen::Vector2f B;
    B << vxe - a1 - 2*a2*T,
          axe - 2*a2;

    Eigen::Vector2f c_eigen = A.colPivHouseholderQr().solve(B);
    a3 = c_eigen[0];
    a4 = c_eigen[1];
  };

  float calc_point(float t){
    return a0 + a1*t + a2*std::pow(t, 2) + a3*std::pow(t, 3) + a4*std::pow(t, 4);
  };

  float calc_first_derivative(float t){
    return a1 + 2*a2*t + 3*a3*std::pow(t, 2) + 4*a4*std::pow(t, 3);
  };

  float calc_second_derivative(float t){
    return 2*a2 + 6*a3*t + 12*a4*std::pow(t, 2);
  };

  float calc_third_derivative(float t){
    return 6*a3 + 24*a4*t;
  };

};
}
#endif
