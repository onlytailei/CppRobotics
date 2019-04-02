/*************************************************************************
	> File Name: quintic_polynomial.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Tue Apr  2 20:50:31 2019
 ************************************************************************/

#ifndef _QUINTIC_POLYNOMIAL_H
#define _QUINTIC_POLYNOMIAL_H

#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <Eigen/Eigen>

class QuinticPolynomial{
	public:
		// current parameter at t=0
		float xs;
		float vxs;
		float axs;

		// parameters at target t=t_j
		float xe;
		float vxe;
		float axe;

		QuinticPolynomial(){};

		// polynomial parameters
		QuinticPolynomial(float xs_, float vxs_, float axs, float xe, float vxe, float axe, float T){
			Eigen::Matrix3f A;
			Eigen::Vector3f B;

		}

	private:
};

#endif
