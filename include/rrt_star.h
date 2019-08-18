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


class RRTStar_Node : public Node{
public:
  float cost;
	RRTStar_Node(float x_, float y_): Node(x_, y_), cost(0){};
};

class RRTStar: public RRT{
public:
	RRTStar(Node*, Node*, std::vector<std::vector<float> >, std::vector<float>, float, int, int, float);
private:
  float connect_circle_dist;
};

RRTStar::RRTStar(Node* start_, Node* end_, std::vector<std::vector<float> > ob_list_, std::vector<float> rand_area_, float expand_dis_, int goal_sample_rate_, int max_iter_, float connect_circle_dist_): RRT(start_, end_, ob_list_, rand_area_, expand_dis_, goal_sample_rate_, max_iter_), connect_circle_dist(connect_circle_dist_){};

}
#endif
