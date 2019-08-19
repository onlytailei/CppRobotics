/*************************************************************************
	> File Name: rrt.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Sat Jul 27 16:45:17 2019
 ************************************************************************/

#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include"rrt_star.h"

using namespace std;
using namespace cpprobotics;

int main(){
	std::vector<vector<float> > obstacle_list{
		{5, 5, 1},
		{3, 6, 2},
		{3, 8, 2},
		{3, 10, 2},
		{7, 5, 2},
		{9, 5, 2}};

	Node* start = new Node(0.0, 0.0);
	Node* goal = new Node(6.0, 9.0);

	std::vector<float> rand_area{-2, 15};

	RRTStar rrt_star(start, goal, obstacle_list, rand_area, 0.5, 1.0, 5, 500, 50.0);


	std::vector<Node*> path = rrt_star.planning();

}

