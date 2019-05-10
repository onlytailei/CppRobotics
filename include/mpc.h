/*************************************************************************
	> File Name: mpc.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Mon May  6 19:46:52 2019
 ************************************************************************/

#ifndef _MPC_H
#define _MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Eigen>
#include <vector>

size_t x_start = 0;
size_t y_start = x_start + T;
size_t yaw_start = y_start + T;
size_t v_start = yaw_start + T;
size_t delta_start = v_start + T;
size_t a_start = delta_start + T - 1;


class FG_EVAL{
	public:
		// Eigen::VectorXd coeeffs;
		vector<float> previous_actuations;

		FG_eval(vector<float> previous_actuations, vector<float> x_ref){
			this->previous_actuations = previous_actuations;
		}

		typedef CPPAD_TESTVECTOR(AD<float>) ADVector;

		void operator()(ADvector& fg, const ADvector& vars){
			fg[0] = 0;

			for(int i=0; i<T; i++){
				fg[0] += CppAD::pow(vars[x_start+i] - ref_v, 2);
				fg[0] += CppAD::pow(vars[y_start+i] - ref_v, 2);
				fg[0] += CppAD::pow(vars[yaw_start+i] - ref_v, 2);
				fg[0] += CppAD::pow(vars[v_start+i] - ref_v, 2);
			}

			for(int i=0; i<T-1; i++){
				fg[0] += CppAD::pow(vars[delta_start+i], 2);
				fg[0] += CppAD::pow(vars[a_start+i], 2);
			}

			for(int i=0; i<T-2; i++){
				fg[0] += CppAD::pow(vars[delta_start+i+1] - vars[delta_start+i], 2);
				fg[0] += CppAD::pow(vars[a_start+i+1] - vars[a_start+i], 2);
			}
		}
};


class MPC{

};

#endif
