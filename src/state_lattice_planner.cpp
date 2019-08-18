/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/

#include<iostream>
#include<sstream>
#include<string>
#include<vector>
#include<array>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<Eigen/Eigen>
#include"csv_reader.h"
#include"motion_model.h"
#include"trajectory_optimizer.h"

#define L 1.0
#define DS 0.1
#define CONST_V 3.0  // use a const linear velocity here
typedef std::vector<std::vector<float>> Table;

using namespace cpprobotics;

StateList sample_states(std::vector<float> angle_samples,
                        float a_min, float a_max,
                        int d, float p_max, float p_min, int nh){
  StateList states;
  for(float item:angle_samples){
    float a = a_min + ( a_max - a_min ) * item;
    for(int j=0; j<nh; j++){
      float xf = d * std::cos(a);
      float yf = d * std::sin(a);
      float yawf;
      if(nh == 1) yawf = (p_max - p_min)/2.0 + a;
      else yawf = p_min + (p_max - p_min) * j /(nh-1) + a;
      states.push_back(TrajState(xf, yf, yawf));
    }
  }
  return states;
};

StateList calc_uniform_polar_states(int nxy, int nh, int d,
                                    float a_min, float a_max,
                                    float p_min, float p_max){
  std::vector<float> angle_samples;
  for(int i=0; i<nxy; i++){
    angle_samples.push_back(i*1.0/(nxy-1));
  }
  StateList states = sample_states(angle_samples, a_min, a_max, d, p_max, p_min, nh);
  return states;
};

StateList calc_biased_polar_states(float goal_angle, int ns, int nxy,
                                   int nh, int d,
                                   float a_min, float a_max,
                                   float p_min, float p_max){
  std::vector<float> asi;
  std::vector<float> cnav;
  float cnav_max = std::numeric_limits<float>::min();
  float cnav_sum = 0;
  for(int i=0; i<ns-1; i++){
    float asi_sample = a_min + (a_max - a_min)*i/(ns-1);
    asi.push_back(asi_sample);
    float cnav_sample = M_PI - std::abs(asi_sample - goal_angle);
    cnav.push_back(cnav_sample);
    cnav_sum += cnav_sample;
    if (cnav_max < cnav_sample){
      cnav_max = cnav_sample;
    }
  }

  std::vector<float> csumnav;
  float cum_temp = 0;
  for(int i=0; i<ns-1; i++){
    cnav[i] = (cnav_max - cnav[i]) / (cnav_max * ns - cnav_sum);
    cum_temp += cnav[i];
    csumnav.push_back(cum_temp);
  }

  int li = 0;
  std::vector<float> angle_samples;
  for(int i=0; i<nxy; i++){
    for(int j=li; j<ns-1; j++){
      if (j*1.0/ns >= i*1.0/(nxy -1)){
        angle_samples.push_back(csumnav[j]);
        li = j - 1;
        break;
      }
    }
  }

  StateList states = sample_states(angle_samples, a_min, a_max, d, p_max, p_min, nh);
  return states;
};

StateList calc_lane_states(float l_center, float l_heading, float l_width, float v_width, float d, int nxy){
  float xc = std::cos(l_heading) * d + std::sin(l_heading) * l_center;
  float yc = std::sin(l_heading) * d + std::cos(l_heading) * l_center;

  StateList states;
  for(int i=0; i<nxy; i++){
    float delta = -0.5 * (l_width - v_width) + (l_width - v_width) * i / (nxy -1);
    float xf = xc - delta * std::sin(l_heading);
    float yf = yc + delta * std::cos(l_heading);
    states.push_back(TrajState(xf, yf, l_heading));
  }
  return states;
}

Parameter search_nearest_one_from_lookuptable(TrajState target, Table csv_file){

  float min_d = std::numeric_limits<float>::max();
  int min_id = -1;

  for(unsigned int i=0; i<csv_file.size(); i++)
  {
    float dx = target.x - csv_file[i][0];
    float dy = target.y - csv_file[i][1];
    float dyaw = target.yaw - csv_file[i][2];
    float d = std::sqrt(dx * dx + dy * dy + dyaw * dyaw);

    if ( d<min_d ){
      min_id = i;
      min_d = d;
    }
  }
  Parameter best_p(std::sqrt(target.x * target.x + target.y * target.y),
      {{0, csv_file[min_id][4], csv_file[min_id][5]}});
  return best_p;
}

std::vector<Traj> generate_path(StateList states, Table csv_file, float k0=0.0){
  std::vector<Traj> traj_list;
  for(TrajState state:states){
    Parameter   p = search_nearest_one_from_lookuptable(state, csv_file);
    p.steering_sequence[0] = k0;

    // default settings for this scenario
    State init_state(0, 0, 0, CONST_V);
    MotionModel m_model(L, DS, init_state);
    float cost_th_ = 0.1;
    std::vector<float> h_step_{0.5, 0.02, 0.02};
    int max_iter = 100;

    TrajectoryOptimizer traj_opti_obj(m_model, p, state);
    Traj traj = traj_opti_obj.optimizer_traj(max_iter, cost_th_, h_step_, true, true);
    traj_list.push_back(traj);
  }
  return traj_list;
};

std::vector<Traj> uniform_terminal_state_sample_test(Table csv_file){
  float k0 = 0.0;
  int nxy = 5;  // number of position sampling
  int nh = 3;  // number of heading sampling
  int d = 20; // distance to target
  float a_min = -45.0/180 * M_PI; // position sampling min angle
  float a_max = +45.0/180 * M_PI; // position sampling max angle
  float p_min = -45.0/180 * M_PI; // heading sampling min angle
  float p_max = +45.0/180 * M_PI; // heading sampling max angle

  StateList states = calc_uniform_polar_states(nxy, nh, d,
                                               a_min, a_max,
                                               p_min, p_max);

  std::vector<Traj> traj_list = generate_path(states, csv_file, k0);
  return traj_list;
};

std::vector<Traj> biased_terminal_state_sample_test(Table csv_file){
  float k0 = 0.0;
  int nxy = 30;  // number of position sampling
  int nh = 2;  // number of heading sampling
  int d = 20; // distance to target
  float a_min = -45.0/180 * M_PI; // position sampling min angle
  float a_max = +45.0/180 * M_PI; // position sampling max angle
  float p_min = -20.0/180 * M_PI; // heading sampling min angle
  float p_max = +20.0/180 * M_PI; // heading sampling max angle

  int ns = 100;
  float goal_angle = 0.0;
  StateList states = calc_biased_polar_states(goal_angle, ns,
                                              nxy, nh, d,
                                              a_min, a_max,
                                              p_min, p_max);

  std::vector<Traj> traj_list = generate_path(states, csv_file, k0);
  return traj_list;
};

std::vector<Traj> lane_state_sample_test(Table csv_file){
  float k0 = 0.0;
  float l_center = 10.0;
  float l_heading = 90.0/180.0 * M_PI;
  float l_width = 3.0;
  float v_width = 1.0;
  int d = 10;
  int nxy = 5;

  StateList states = calc_lane_states(l_center, l_heading, l_width,
                                      v_width, d, nxy);

  std::vector<Traj> traj_list = generate_path(states, csv_file, k0);
  return traj_list;
};

int main(){
  //uniform_terminal_state_sample_test1();
  std::vector<std::vector<float>> lookup_table;

  std::ifstream file("../../lookuptable.csv");
  CSVIterator loop(file);
  loop++;
  for(; loop != CSVIterator(); ++loop)
  {
    std::vector<float> temp;
    for(int i=0; i<6; i++){
      temp.push_back(std::stod((*loop)[i]));
    }
    lookup_table.push_back(temp);
  }
  std::vector<Traj> traj_list1 = uniform_terminal_state_sample_test(lookup_table);
  std::vector<Traj> traj_list2 = biased_terminal_state_sample_test(lookup_table);
  std::vector<Traj> traj_list3 = lane_state_sample_test(lookup_table);

};
