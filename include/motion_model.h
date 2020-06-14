/*************************************************************************
	> File Name: motion_model.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Thu Mar 21 16:06:07 2019
 ************************************************************************/

#ifndef _MOTION_MODEL_H
#define _MOTION_MODEL_H

#include<iostream>
#include<vector>
#include<array>
#include<cmath>
#include<cfenv>
#include<Eigen/Eigen>

#define YAW_P2P(angle) std::fmod(std::fmod((angle)+M_PI, 2*M_PI)-2*M_PI, 2*M_PI)+M_PI

namespace cpprobotics{

struct Parameter{
  float distance;
  std::array<float, 3> steering_sequence{{0,0,0}};
  Parameter(float distance_, std::array<float, 3> steering_sequence_){
    distance = distance_;
    steering_sequence = steering_sequence_;
  };
};

struct State{
  float x;
  float y;
  float yaw;
  float v;
  State(float x_, float y_, float yaw_, float v_){
    x = x_;
    y = y_;
    yaw = yaw_;
    v = v_;
  };
};

struct TrajState{
  float x;
  float y;
  float yaw;
  TrajState(float x_, float y_, float yaw_){
    x = x_;
    y = y_;
    yaw = yaw_;
  };
};

using Traj = std::vector<TrajState>;
using StateList = std::vector<TrajState>;
using ParameterList = std::vector<Parameter>;

std::vector<float> quadratic_interpolation(
    std::array<float, 3> x, std::array<float, 3> y){
  Eigen::Matrix3f A;
  Eigen::Vector3f Y;
  A<< std::pow(x[0], 2), x[0], 1,
      std::pow(x[1], 2), x[1], 1,
      std::pow(x[2], 2), x[2], 1;
  Y<<y[0], y[1], y[2];

  Eigen::Vector3f result = A.inverse() * Y;
  float* result_data = result.data();
  std::vector<float> result_array(result_data, result_data+3);
  return result_array;
}

float interp_refer(std::vector<float> para, float x){
  return para[0] * x * x + para[1] * x + para[2];
}


class MotionModel{
public:
  const float base_l;
  const float ds;
  State state;

  MotionModel(float base_l_, float ds_, State state_):
    base_l(base_l_), ds(ds_), state(state_){};
  void update(float v_, float delta, float dt);
  State update(State state_, float delta, float dt);
  Traj generate_trajectory(Parameter);
  TrajState generate_last_state(Parameter);

};

void MotionModel::update(float v_, float delta, float dt){
  state.v = v_;
  state.x = state.x + state.v * std::cos(state.yaw) * dt;
  state.y = state.y + state.v * std::sin(state.yaw) * dt;
  state.yaw = state.yaw + state.v / base_l * std::tan(delta) * dt;
  state.yaw = YAW_P2P(state.yaw);
};

State MotionModel::update(State state_, float delta, float dt){
  state_.x = state_.x + state_.v * std::cos(state_.yaw) * dt;
  state_.y = state_.y + state_.v * std::sin(state_.yaw) * dt;
  state_.yaw = state_.yaw + state_.v / base_l * std::tan(delta) * dt;
  state_.yaw = YAW_P2P(state_.yaw);
  return state_;
};

Traj MotionModel::generate_trajectory(Parameter p){
  float n =  p.distance / ds;
  float horizon = p.distance / state.v;

  // boost::math::cubic_b_spline<float> spline(
  //   p.steering_sequence.data(), p.steering_sequence.size(),
  //   0, horizon/p.steering_sequence.size());
  std::vector<float> spline = quadratic_interpolation(
    {{0, horizon/2, horizon}},
    p.steering_sequence);

  Traj output;
  State state_ = state;

  for(float i=0.0; i<horizon; i+=horizon/n){
    float kp = interp_refer(spline, i);
    state_ = update(state_, kp, horizon/n);
    TrajState xyyaw{state_.x, state_.y, state_.yaw};
    output.push_back(xyyaw);
  }
  return output;
}

TrajState MotionModel::generate_last_state(Parameter p){
  float n = p.distance / ds;
  float horizon = p.distance / state.v;

  // boost::math::cubic_b_spline<float> spline(
  //   p.steering_sequence.data(), p.steering_sequence.size(),
  //   0, horizon/p.steering_sequence.size());
  std::vector<float> spline = quadratic_interpolation(
    {{0, horizon/2, horizon}},
    p.steering_sequence);

  State state_ = state;
  for(float i=0.0; i<horizon; i+=horizon/n){
      float kp = interp_refer(spline, i);
      state_ = update(state_, kp, horizon/n);
  }
  return TrajState{state_.x, state_.y, state_.yaw};
}
}


#endif
