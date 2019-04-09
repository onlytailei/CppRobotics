/*************************************************************************
	> File Name: frenet_optimal_trajectory.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Wed Apr  3 09:52:17 2019
 ************************************************************************/

#include <iostream>
#include <vector>
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"

int main(){
	std::vector<float> wx({0.0, 10.0, 20.5, 35.0, 70.5});
	std::vector<float> wy({0.0, -6.0, 5.0, 6.5, 0.0});
	std::vector<std::array<float, 2>> obstcles{
		{{20.0, 10.0}},
		{{30.0, 6.0}},
		{{30.0, 8.0}},
		{{35.0, 8.0}},
		{{50.0, 3.0}}
	};

	return 0; 
}