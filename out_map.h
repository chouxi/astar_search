#ifndef _OUT_MAP_H_
#define _OUT_MAP_H_

#pragma once
#include "Assignment1.h"
class out_map
{
public:
	out_map(vector<vector<map_str>> map, vector<pair<int, int>> start_goal, vector<pair<int, int>> hard_center, string map_file, int row, int col);
	~out_map();
private:
	ofstream out_m_file;
	void output_map(vector<vector<map_str>> map, vector<pair<int, int>> start_goal, vector<pair<int, int>> hard_center, int row, int col);
};


#endif // !_OUT_MAP_H_
