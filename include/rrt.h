/*************************************************************************
	> File Name: rrt.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Sat Jul 27 16:46:44 2019
 ************************************************************************/

#ifndef _RRT_H
#define _RRT_H

#include<iostream>
#include<limits>
#include<random>
#include<vector>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

namespace cpprobotics{

class Node{
public:
	float x;
	float y;
  std::vector<float> path_x;
  std::vector<float> path_y;
	Node* parent;
  float cost;

	Node(float x_, float y_): x(x_), y(y_), parent(NULL), cost(0){};
};


class RRT{
public:
	RRT(Node*, Node*, std::vector<std::vector<float> >, 
		  std::vector<float>, float, float, int, int);

	std::vector<Node*> planning();
	
	Node* GetNearestNode(const std::vector<float>);
	
	bool CollisionCheck(Node*);

protected:

  Node* steer(Node* , Node*, float extend_length);

	Node* start;
	Node* end;
	const float expand_dis;
	const float path_resolution;
	const int goal_sample_rate;
	const int max_iter;
	const std::vector<std::vector<float> > ob_list;

	std::vector<float> rand_area;
	std::vector<Node*> node_list;

	std::random_device goal_rd;
  std::mt19937 goal_gen;
  std::uniform_int_distribution<int> goal_dis;
	
	std::random_device area_rd;
  std::mt19937 area_gen;
  std::uniform_real_distribution<float> area_dis;

  static std::vector<float> calc_distance_and_angle(Node*, Node*);
};

RRT::RRT(Node* start_, Node* end_, std::vector<std::vector<float> > ob_list_, std::vector<float> rand_area_, float expand_dis_=1.0, float path_resolution_=1.0, int goal_sample_rate_=5, int max_iter_=500 ): 
		start(start_), end(end_), ob_list(ob_list_), expand_dis(expand_dis_), path_resolution(path_resolution_), goal_sample_rate(goal_sample_rate_), max_iter(max_iter_), goal_gen(goal_rd()), goal_dis(std::uniform_int_distribution<int>(0, 100)), area_gen(area_rd()), area_dis(std::uniform_real_distribution<float>(rand_area_[0], rand_area_[1])), rand_area(rand_area_){};

std::vector<Node*> RRT::planning(){

	//visualization
	cv::namedWindow("rrt", cv::WINDOW_NORMAL);
	int count=0;
	int img_size = (int)(rand_area[1] - rand_area[0]);
	int img_reso = 50;
  cv::Mat bg(img_size * img_reso, img_size * img_reso,
             CV_8UC3, cv::Scalar(255,255,255));

	cv::circle(bg, cv::Point((int)((start->x-rand_area[0])*img_reso), (int)((start->y-rand_area[0])*img_reso)), 20, cv::Scalar(0,0,255), -1);
	cv::circle(bg, cv::Point((int)((end->x-rand_area[0])*img_reso), (int)((end->y-rand_area[0])*img_reso)), 20, cv::Scalar(255,0,0), -1);
	for(auto item:ob_list){
      cv::circle(bg, cv::Point((int)((item[0]-rand_area[0])*img_reso), (int)((item[1]-rand_area[0])*img_reso)), (int)item[2] * img_reso, cv::Scalar(0,0,0), -1);
	}


	node_list.push_back(start);
	while(true){
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

		Node* nearest_node =  GetNearestNode(rnd);

		float theta = std::atan2(rnd[1]-nearest_node->y, rnd[0]-nearest_node->x); 

		Node* new_node = new Node(nearest_node->x+expand_dis*std::cos(theta), nearest_node->y+expand_dis*std::sin(theta));
		new_node->parent = nearest_node;

		if (!CollisionCheck(new_node)) continue;

		node_list.push_back(new_node);

		//visualization
		cv::line(
			bg, 
			cv::Point((int)((new_node->x-rand_area[0])*img_reso), (int)((new_node->y-rand_area[0])*img_reso)), 
			cv::Point((int)((nearest_node->x-rand_area[0])*img_reso), (int)((nearest_node->y-rand_area[0])*img_reso)),
			cv::Scalar(0,255,0), 10);

		// std::string int_count = std::to_string(count);
		// cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
		count++;
		cv::imshow("rrt", bg);
		cv::waitKey(5);

		if (std::sqrt(std::pow((new_node->x - end->x), 2) + std::pow((new_node->y - end->y), 2)) <= expand_dis){
			std::cout<<"find path"<<std::endl;
			break;
		}
	}

	std::vector<Node*> path;
	path.push_back(end);
	Node* temp = node_list.back();
	while (temp->parent != NULL){
		//visualization
		cv::line(
			bg, 
			cv::Point((int)((temp->x-rand_area[0])*img_reso), (int)((temp->y-rand_area[0])*img_reso)), 
			cv::Point((int)((temp->parent->x-rand_area[0])*img_reso), (int)((temp->parent->y-rand_area[0])*img_reso)),
			cv::Scalar(255,0,255), 10);

		path.push_back(temp);
		temp = temp->parent;
	}

	//viosualization
	// std::string int_count = std::to_string(count);
	// cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
	cv::imshow("rrt", bg);
	cv::waitKey(5);

	path.push_back(start);

	return path;
};

Node* RRT::GetNearestNode(const std::vector<float> rnd){
	int min_id = -1;
	float min_distance = std::numeric_limits<float>::max();

	for(int i=0; i<node_list.size(); i++){
		float dist = std::pow((node_list[i]->x-rnd[0]), 2) + std::pow((node_list[i]->y-rnd[1]), 2);
		if (dist <  min_distance){
			min_distance = dist;
			min_id = i;
		}
	}

	return node_list[min_id];
};

bool RRT::CollisionCheck(Node* node){
	for(std::vector<float> item:ob_list){
		if (std::sqrt(std::pow((item[0] - node->x), 2) + std::pow((item[1] - node->y), 2)) <= item[2]) return false; 
	}
	return true;
}

Node* RRT::steer(Node* from_node, Node* to_node, float extend_length=std::numeric_limits<float>::max()){
  Node * new_node = new Node(from_node->x, from_node->y);
  std::vector<float> dist_angle = calc_distance_and_angle(new_node, to_node);

  new_node->path_x.push_back(new_node->x); 
  new_node->path_y.push_back(new_node->y);

  if (extend_length > dist_angle[0]){
    extend_length = dist_angle[0];
  }

  int n_expand = std::floor(extend_length/path_resolution);

  for(int i=0; i<n_expand; i++){
    new_node->x += path_resolution * std::cos(dist_angle[1]);
    new_node->y += path_resolution * std::sin(dist_angle[1]);
    new_node->path_x.push_back(new_node->x);
    new_node->path_y.push_back(new_node->y);
  }

  std::vector<float> dist_angle_new = calc_distance_and_angle(new_node, to_node);

  if (dist_angle_new[0] <= path_resolution){
    new_node->x = to_node->x;
    new_node->y = to_node->y;
    new_node->path_x.back()=to_node->x;
    new_node->path_y.back()=to_node->y;
  }

  new_node->parent = from_node;
  return new_node;

};

std::vector<float> RRT::calc_distance_and_angle(Node* from_node, Node* to_node){
  float dx = to_node->x - from_node->x;
  float dy = to_node->y - from_node->y;
  float d = std::sqrt(dx*dx + dy*dy);
  float theta = std::atan2(dy, dx);
  return {d, theta};
};

}
#endif
