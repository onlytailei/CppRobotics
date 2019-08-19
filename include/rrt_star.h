/*************************************************************************
	> File Name: rrt.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Sat Jul 27 16:46:44 2019
 ************************************************************************/

#ifndef _RRT_STAR_H
#define _RRT_STAR_H

#include<iostream>
#include<limits>
#include<random>
#include<vector>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include"rrt.h"

namespace cpprobotics{


// class RRTStar_Node : public Node{
// public:
//   float cost;
// 	RRTStar_Node(float x_, float y_): Node(x_, y_), cost(0){};
// };

class RRTStar: public RRT{
public:
	RRTStar(Node*, Node*, std::vector<std::vector<float> >, std::vector<float>, float, float, int, int, float);

  std::vector<Node* > planning();
private:
  std::vector<Node* > find_near_nodes(Node* );

  float calc_new_cost(Node*, Node*);

  Node* choose_parent(Node*, std::vector<Node* >);

  float connect_circle_dist;
};

RRTStar::RRTStar(Node* start_, Node* end_, std::vector<std::vector<float> > ob_list_, std::vector<float> rand_area_, float expand_dis_, float path_resolution_, int goal_sample_rate_, int max_iter_, float connect_circle_dist_): RRT(start_, end_, ob_list_, rand_area_, expand_dis_, path_resolution_, goal_sample_rate_, max_iter_), connect_circle_dist(connect_circle_dist_){};

Node* RRTStar::choose_parent(Node* new_node, std::vector<Node* > neighbours){
  Node* output;
  if (neighbours.size()==0){
    return output;
  }

  std::vector<float> costs;
  for (Node* n_:neighbours){
    Node* t_node = steer(n_, new_node);
    if (t_node && CollisionCheck(t_node)){
      costs.push_back(calc_new_cost(n_, new_node));
    }else{
      costs.push_back(std::numeric_limits<float>::max());
    }
  }
  // min_costs

};

std::vector<Node* > RRTStar::planning(){
	node_list.push_back(start);
  for(int i=0; i<max_iter; i++){
		std::vector<float> rnd;
		if (goal_dis(goal_gen)>goal_sample_rate){
			float rand_x = area_dis(goal_gen);
			float rand_y = area_dis(goal_gen);
			rnd.push_back(rand_x);
			rnd.push_back(rand_y);
		}else{
			rnd.push_back(end->x);
			rnd.push_back(end->y);
		}

    Node* rnd_node = new Node(rnd[0], rnd[1]);
		Node* nearest_node =  GetNearestNode(rnd);

    Node* new_node = steer(nearest_node, rnd_node, expand_dis);
    
		if (!CollisionCheck(new_node)) continue;
      
    std::vector<Node* > neighbour_nodes=find_near_nodes(new_node);
    // choose parent  

  return node_list;

  }
};

std::vector<Node* > RRTStar::find_near_nodes(Node* new_node){
  std::vector<Node* > output;
  int nnode = node_list.size() + 1;
  float r = connect_circle_dist * std::sqrt(std::log(nnode)/nnode);
  for(Node* n_:node_list){
    if ((n_->x-new_node->x)*(n_->x-new_node->x) + (n_->y-new_node->y)*(n_->y-new_node->y)<r*r){
      output.push_back(n_);
    }
    return output;
  }
};

float RRTStar::calc_new_cost(Node* from_node, Node* to_node){
  std::vector<float> dist_angle = calc_distance_and_angle(from_node, to_node);
  return from_node->cost + dist_angle[0];
};

}
#endif
