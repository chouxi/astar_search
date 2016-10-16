#ifndef _ASSIGNMENT1_H_
#define _ASSIGNMENT1_H_
#pragma once
#include <iostream>
#include <vector>
#include <set>
#include <string>
#include <fstream>
#include <sstream>
using namespace std;

typedef struct _map_str_ {
	char type;
	char highway_num;
	float gn = -1;
	float hn = -1;
	pair<int, int> parent;
}map_str;

class Assignment1
{
public:
	int row;
	int col;
	float route_cost;
	bool did_astar;
	string map_file;
	string result_file;
	vector<vector<map_str>> origin_map;
	vector<pair<int, int>> hard_center;
	vector<pair<int, int>> start_goal;
	vector<pair<int, int>> path;
	Assignment1(int row, int col, string map_file, string result_file) {
		this->row = row;
		this->col = col;
		this->map_file = map_file;
		this->result_file = result_file;
	}
	~Assignment1(){}
};

#endif