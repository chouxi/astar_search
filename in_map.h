#ifndef _IN_MAP_H_
#define _IN_MAP_H_
#include "Assignment1.h"

#pragma once
class in_map
{
public:
	in_map(vector<vector<map_str>>& origin, string map_file, vector<pair<int, int>>& start_goal);
	~in_map();
private:
	ifstream map_in;
};

#endif // !_IN_MAP_H_
