#ifndef _OUT_ASTAR_H_
#define _OUT_ASTAR_H_
#include "Assignment1.h"

#pragma once
class out_astar
{
public:
	out_astar(vector<pair<int ,int>> path, float route_cost, string result_out);
	~out_astar();
private:
	ofstream astar_out;
};


#endif // !_OUT_ASTAR_H_
