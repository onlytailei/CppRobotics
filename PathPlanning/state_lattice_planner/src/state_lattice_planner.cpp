/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Eigen>
#include "csv_reader.h"
#include "motion_model.h"
#include "trajectory_optimizer.h"

#define L 1.0
#define DS 0.1
#define CONST_V 3.0  // use a const linear velocity here
typedef std::vector<std::vector<float>> Table;

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

std::vector<Traj> generate_path(StateList states, Table csv_file){
  std::vector<Traj> traj_list;
  for(TrajState state:states){
    Parameter   p = search_nearest_one_from_lookuptable(state, csv_file);

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

std::vector<Traj> uniform_terminal_state_sample_test1(Table csv_file){
  int nxy = 5;  // number of position sampling
  int nh = 3;  // number of heading sampling
  int d = 20; // distance to target
  float a_min = - 45.0/180 * M_PI; // position sampling min angle
  float a_max = + 45.0/180 * M_PI; // position sampling max angle
  float p_min = - 45.0/180 * M_PI; // heading sampling min angle
  float p_max = + 45.0/180 * M_PI; // heading sampling max angle

  StateList states = calc_uniform_polar_states(nxy, nh, d,
                                               a_min, a_max,
                                               p_min, p_max);

  std::vector<Traj> traj_list = generate_path(states, csv_file);
  return traj_list;
};

int main(){
  //uniform_terminal_state_sample_test1();
    std::vector<std::vector<float>> lookup_table;

    std::ifstream file("../lookuptable.csv");
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
    std::vector<Traj> traj_list = uniform_terminal_state_sample_test1(lookup_table);

};
