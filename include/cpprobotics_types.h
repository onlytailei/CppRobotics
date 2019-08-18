/*************************************************************************
	> File Name: cpprobotics_types.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Mon Apr 22 13:25:12 2019
 ************************************************************************/

#ifndef _CPPROBOTICS_TYPES_H
#define _CPPROBOTICS_TYPES_H

#include<iterator>
#include<vector>
#include<array>
#include<string>
#include<iostream>

namespace cpprobotics{

using Vec_f=std::vector<float>;
using Poi_f=std::array<float, 2>;
using Vec_Poi=std::vector<Poi_f>;

};

#endif
