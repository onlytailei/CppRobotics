/*************************************************************************
	> File Name: model_predictive_control.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Wed Apr 17 11:48:46 2019
 ************************************************************************/

#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/time.h>
#include <Eigen/Eigen>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <nlopt.hpp>
#include "cubic_spline.h"
#include "motion_model.h"
#include "cpprobotics_types.h"

#define NX 4
#define T 5

#define DT 0.1
#define L 0.5
#define KP 1.0
#define MAX_STEER 45.0/180*M_PI

#define MAX_ITER 3
#define DU_TH 0.1

#define N_IND_SEARCH 10
#define MAX_TIME 500.0

#define WB 2.5
#define MAX_SPEED   55.0/3.6
#define MIN_SPEED  -20.0/3.6

#define LENGTH  4.5
#define WIDTH 2.0
#define BACKTOWHEEL 1.0
#define WHEEL_LEN 0.3
#define WHEEL_WIDTH 0.2
#define TREAD 0.7
#define WB 2.5 


using namespace cpprobotics;
// NOTE && TODO check T+1 condition
using M_XREF=Eigen::Matrix<float, NX, T+1>;
using M_DREF=Eigen::Matrix<float, 1, T+1>;

size_t x_start = 0;
size_t y_start = x_start + T;
size_t yaw_start = y_start + T;
size_t v_start = yaw_start + T;
size_t x_ref_start = v_start + T-1;
size_t y_ref_start = x_ref_start + T-1;
size_t yaw_ref_start = y_ref_start + T-1;
size_t v_ref_start = yaw_ref_start + T-1;
size_t delta_start = v_ref_start + T-1;
size_t a_start = delta_start + T-1;

cv::Point2i cv_offset(
    float x, float y, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + 300;
  output.y = image_height - int(y * 100) - image_height/2;
  return output;
};

void update(State& state, float a, float delta){

	if (delta >= MAX_STEER) delta = MAX_STEER;
	if (delta <= - MAX_STEER) delta = - MAX_STEER;

	state.x = state.x + state.v * std::cos(state.yaw) * DT;
	state.y = state.y + state.v * std::sin(state.yaw) * DT;
	state.yaw = state.yaw + state.v / L * std::tan(delta) * DT;
	state.v = state.v + a * DT;

	if (state.v > MAX_SPEED) state.v = MAX_SPEED;
	if (state.v < MIN_SPEED) state.v = MIN_SPEED;

};

M_XREF predict_motion(State state, Vec_f oa, Vec_f od){
	M_XREF xbar = M_XREF::Zero();
	xbar(0, 0) = state.x; 
	xbar(1, 0) = state.y; 
	xbar(2, 0) = state.v; 
	xbar(3, 0) = state.yaw; 
	for(int i=1; i<T+1; i++){
		update(state, oa[i], od[i]);
		xbar(0, i) = state.x; 
		xbar(1, i) = state.y; 
		xbar(2, i) = state.v; 
		xbar(3, i) = state.yaw; 
	}
	return xbar;
};

void linear_mpc_control(M_XREF x_ref, M_XREF xbar, Eigen::Vector4f x0, M_DREF dref){
	nlopt::opt opt(nlopt::LD_MMA, 2);
	// std::vector<>
};

void iterative_linear_mpc_control(M_XREF x_ref, State state, M_DREF d_ref, Vec_f oa, Vec_f od){
	if (oa.size()==0 || od.size()==0){
		oa = std::vector<float>(T, 0.0);
		od = std::vector<float>(T, 0.0);
	}

	for(int i=0; i<MAX_ITER; i++){
		M_XREF x_bar = predict_motion(state, oa, od);
		Eigen::Vector4f x0;
		x0<< state.x, state.y, state.v, state.yaw;
		linear_mpc_control(x_ref, x_bar, x0, d_ref);
	}

};

Vec_f calc_speed_profile(Vec_f rx, Vec_f ry, Vec_f ryaw, float target_speed){
	Vec_f speed_profile(ryaw.size(), target_speed);

	float direction = 1.0;
	for(unsigned int i=0; i < ryaw.size()-1; i++){
		float dx = rx[i+1] - rx[i];
		float dy = ry[i+1] - ry[i];
        float move_direction = std::atan2(dy, dx);

        if (dx != 0.0 && dy != 0.0){
            float dangle = std::abs(YAW_P2P(move_direction - ryaw[i]));
            if (dangle >= M_PI/4.0) direction = -1.0;
            else direction = 1.0;
        }

        if (direction != 1.0) speed_profile[i] = -1 * target_speed;
        else speed_profile[i] = target_speed;

	}
  speed_profile[-1] = 0.0;

	return speed_profile;
};

int calc_nearest_index(State state, Vec_f cx, Vec_f cy, Vec_f cyaw, int pind){
	float mind = std::numeric_limits<float>::max();
	float ind = 0;
	for(unsigned int i=pind; i<pind+N_IND_SEARCH; i++){
		float idx = cx[i] - state.x;
		float idy = cy[i] - state.y;
		float d_e = idx*idx + idy*idy;

		if (d_e<mind){
			mind = d_e;
			ind = i;
		}
	}

	float dxl = cx[ind] - state.x;
	float dyl = cy[ind] - state.y;
	float angle = YAW_P2P(cyaw[ind] - std::atan2(dyl, dxl));
	if (angle < 0) mind = mind * -1;

	return ind;
};


void calc_ref_trajectory(State state, Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f 
ck, Vec_f sp, float dl, int& target_ind, M_XREF& xref, M_DREF& dref){

	xref = Eigen::Matrix<float, NX, T+1>::Zero();
	dref = Eigen::Matrix<float, 1, T+1>::Zero();

	int ncourse = cx.size();

	int ind = calc_nearest_index(state, cx, cy, cyaw, target_ind);
	if (target_ind > ind) ind = target_ind;

	xref(0, 0) = cx[ind];
	xref(1, 0) = cy[ind];
	xref(2, 0) = sp[ind];
	xref(3, 0) = cyaw[ind];
	dref(0, 0) = 0.0;

	float travel = 0.0;

	for(int i=0; i<T+1; i++){
		travel += std::abs(state.v) * DT;
		int dind = (int)std::round(travel/dl);

		if ((ind+dind)<ncourse){
			xref(0, i) = cx[ind + dind];
			xref(1, i) = cy[ind + dind];
			xref(2, i) = sp[ind + dind];
			xref(3, i) = cyaw[ind + dind];
			dref(0, i) = 0.0;
		}else{
			xref(0, i) = cx[ncourse - 1];
			xref(1, i) = cy[ncourse - 1];
			xref(2, i) = sp[ncourse - 1];
			xref(3, i) = cyaw[ncourse - 1];
			dref(0, i) = 0.0;
		}
	}

	target_ind = ind;
};

void smooth_yaw(Vec_f& cyaw){
	for(unsigned int i=0; i<cyaw.size()-1; i++){
		float dyaw = cyaw[i+1] - cyaw[i];

		while (dyaw > M_PI/2.0){
			cyaw[i+1] -= M_PI*2.0;
			dyaw = cyaw[i+1] - cyaw[i];
		}
		while (dyaw < -M_PI/2.0){
			cyaw[i+1] += M_PI*2.0;
			dyaw = cyaw[i+1] - cyaw[i];
		}

	}
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  //vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  Solution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  double delta_prev {0};
  double a_prev {0.1};

};

class FG_EVAL{
	public:
		// Eigen::VectorXd coeeffs;
		M_XREF traj_ref;

		FG_eval(M_XREF traj_ref){
			this->traj_ref = traj_ref;
		}

		typedef CPPAD_TESTVECTOR(AD<float>) ADVector;

		void operator()(ADvector& fg, const ADvector& vars){
			fg[0] = 0;

			for(int i=0; i<T-1; i++){
				fg[0] += CppAD::pow(vars[delta_start+i], 2);
				fg[0] += CppAD::pow(vars[a_start+i], 2);
			}

			for(int i=0; i<T-2; i++){
				fg[0] += CppAD::pow(vars[delta_start+i+1] - vars[delta_start+i], 2);
				fg[0] += CppAD::pow(vars[a_start+i+1] - vars[a_start+i], 2);
			}

			// fix the initial state as a constraint
			fg[1 + x_start] = vars[x_start];
			fg[1 + y_start] = vars[y_start];
			fg[1 + psi_start] = vars[psi_start];
			fg[1 + v_start] = vars[v_start];

 			// The rest of the constraints
    	for (int i = 0; i < N - 1; i++) {
				// The state at time t+1 .
				AD<double> x1 = vars[x_start + i + 1];
				AD<double> y1 = vars[y_start + i + 1];
				AD<double> psi1 = vars[psi_start + i + 1];
				AD<double> v1 = vars[v_start + i + 1];

				// The state at time t.
				AD<double> x0 = vars[x_start + i];
				AD<double> y0 = vars[y_start + i];
				AD<double> psi0 = vars[psi_start + i];
				AD<double> v0 = vars[v_start + i];

				// Only consider the actuation at time t.
				AD<double> delta0 = vars[delta_start + i];
				AD<double> a0 = vars[a_start + i];

				// constraint with the dynamic model
				fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * DT);
				fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * DT);
				fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / WB * DT);
				fg[2 + v_start + i] = v1 - (v0 + a0 * Dt);
				// constraint with the ref traj
				fg[1 + x_ref_start + i] = CppAD::pow(traj_ref(i, 0) - (x0 + v0 * CppAD::cos(psi0) * DT), 2);
				fg[1 + y_ref_start + i] = CppAD::pow(traj_ref(i, 1) - (y0 + v0 * CppAD::sin(psi0) * DT), 2);
				fg[1 + psi_ref_start + i] = 0.5*CppAD::pow((traj_ref(i, 2) - (psi0 + v0 * delta0 / Lf * DT), 2);
				fg[1 + v_ref_start + i] = 0.5*CppAD::pow((traj_ref(i, 3) - (v0 + a0 * Dt), 2);
			}
		}
};

class MPC{
	public:
		MPC();

		Solution Solve(Eigen::Vectoc4f x0, M_REF traj_ref){
			  typedef CPPAD_TESTVECTOR(double) Dvector;
			  double x = x0[0];
				double y = x0[1];
				double psi = x0[2];
				double v = x0[3];

				size_t n_vars = T * 4 + (T - 1) * 2;
				size_t n_constraints = T * 8 - 4;

				Dvector vars(n_vars);
				for (int i = 0; i < n_vars; i++){
					vars[i] = 0.0;
				}

				vars[x_start] = x;
				vars[y_start] = y;
				vars[psi_start] = psi;
				vars[v_start] = v;
				// vars[x_ref_start] = 0;
				// vars[y_ref_start] = 0;
				// vars[psi_ref_start] = 0;
				// vars[v_ref_start] = 0;

	  		// Lower and upper limits for x
				Dvector vars_lowerbound(n_vars);
				Dvector vars_upperbound(n_vars);

				// Set all non-actuators upper and lowerlimits
				// to the max negative and positive values.
				for (int i = delta_start; i < delta_start+T; i++) {
					vars_lowerbound[i] = -MAX_STEER;
					vars_upperbound[i] = MAX_STEER;
				}

				for (int i = delta_start; i < delta_start+T; i++) {
					vars_lowerbound[i] = -MAX_ACCEL;
					vars_upperbound[i] = MAX_ACCEL;
				}

				for (int i = v_start; i < v_start+T; i++) {
					vars_lowerbound[i] = MIN_SPEED;
					vars_upperbound[i] = MAX_SPEED;
				}

			Dvector constraints_lowerbound(n_constraints);
			Dvector constraints_upperbound(n_constraints);
			for (int i = 0; i < n_constraints; i++) {
				constraints_lowerbound[i] = 0;
				constraints_upperbound[i] = 0;
			}
			constraints_lowerbound[x_start] = x;
			constraints_lowerbound[y_start] = y;
			constraints_lowerbound[psi_start] = psi;
			constraints_lowerbound[v_start] = v;

			constraints_upperbound[x_start] = x;
			constraints_upperbound[y_start] = y;
			constraints_upperbound[psi_start] = psi;
			constraints_upperbound[v_start] = v;

			FG_eval fg_eval(traj_ref);

			// options
			std::string options;
			options += "Integer print_level  0\n";
			options += "Sparse  true        forward\n";
			options += "Sparse  true        reverse\n";
			options += "Numeric max_cpu_time          0.05\n";

			// place to return solution
			CppAD::ipopt::solve_result<Dvector> solution;

			// solve the problem
			CppAD::ipopt::solve<Dvector, FG_eval>(
				options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
				constraints_upperbound, fg_eval, solution);

			bool ok = true;
  		ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  		//cout << "ok " << ok << endl;

		}

}

void mpc_simulation(Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f ck, Vec_f speed_profile, Poi_f goal){
	State state(cx[0], cy[0], cyaw[0], 0.0);

	if ((state.yaw - cyaw[0]) >= M_PI) state.yaw -= M_PI * 2.0;
	else if ((state.yaw - cyaw[0]) <= -1.0*M_PI) state.yaw += M_PI * 2.0;

	float time_ = 0.0;
	Vec_f x;
	x.push_back(state.x);
	Vec_f y;
	y.push_back(state.y);
	Vec_f yaw;
	yaw.push_back(state.yaw);
	Vec_f v;
	v.push_back(state.v);
	Vec_f t;
	t.push_back(0.0);

	int target_ind = 0;
	calc_nearest_index(state, cx, cy, cyaw, target_ind);

	smooth_yaw(cyaw);


  cv::namedWindow("lqr_full", cv::WINDOW_NORMAL);
  int count = 0;
	Vec_f x_h;
	Vec_f y_h;


	M_XREF	xref;
	M_DREF	dref;

	while (MAX_TIME >= time_){
		calc_ref_trajectory(state, cx, cy, cyaw, ck, speed_profile, 1.0, target_ind, xref, dref);

		iterative_linear_mpc_control();


		float dx = state.x - goal[0];
		float dy = state.y - goal[1];
		if (std::sqrt(dx*dx + dy*dy) <= goal_dis) {
			std::cout<<("Goal")<<std::endl;
			break;
		}

		x_h.push_back(state.x);
		y_h.push_back(state.y);

		// visualization
		cv::Mat bg(2000, 3000, CV_8UC3, cv::Scalar(255, 255, 255));
		for(unsigned int i=1; i<cx.size(); i++){
			cv::line(
				bg,
				cv_offset(cx[i-1], cy[i-1], bg.cols, bg.rows),
				cv_offset(cx[i], cy[i], bg.cols, bg.rows),
				cv::Scalar(0, 0, 0),
				10);
		}

		for(unsigned int j=0; j< x_h.size(); j++){
			cv::circle(
				bg,
				cv_offset(x_h[j], y_h[j], bg.cols, bg.rows),
				10, cv::Scalar(0, 0, 255), -1);
		}

		cv::putText(
			bg,
			"Speed: " + std::to_string(state.v*3.6).substr(0, 4) + "km/h",
			cv::Point2i((int)bg.cols*0.5, (int)bg.rows*0.1),
			cv::FONT_HERSHEY_SIMPLEX,
			3,
			cv::Scalar(0, 0, 0),
			10);

		// save image in build/bin/pngs
		struct timeval tp;
		gettimeofday(&tp, NULL);
		long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
		std::string int_count = std::to_string(ms);
		cv::imwrite("./pngs/"+int_count+".png", bg);
		// cv::imshow("lqr_full", bg);
		// cv::waitKey(5);
	}
};

int main(){
    Vec_f wx({0.0, 60.0, 125.0,  50.0,  75.0,  30.0, -10.0});
    Vec_f wy({0.0,  0.0,  50.0,  65.0,  30.0,  50.0, -20.0});

  Spline2D csp_obj(wx, wy);
	Vec_f r_x;
	Vec_f r_y;
	Vec_f ryaw;
	Vec_f rcurvature;
	Vec_f rs;
	for(float i=0; i<csp_obj.s.back(); i+=1.0){
		std::array<float, 2> point_ = csp_obj.calc_postion(i);
		r_x.push_back(point_[0]);
		r_y.push_back(point_[1]);
		ryaw.push_back(csp_obj.calc_yaw(i));
		rcurvature.push_back(csp_obj.calc_curvature(i));
		rs.push_back(i);
	}

	float target_speed = 10.0 / 3.6;
	Vec_f speed_profile = calc_speed_profile(r_x, r_y, ryaw, target_speed);

	mpc_simulation(r_x, r_y, ryaw, rcurvature, speed_profile, {{wx.back(), wy.back()}});

}
