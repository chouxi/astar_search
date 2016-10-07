#ifndef _H_FUNCS_H_
#define _H_FUNCS_H_
#include <algorithm>
#pragma once
using namespace std;
class h_funcs {
public:
	virtual float calc_heuristic(pair<int, int> x, pair<int, int> goal) = 0;
};

class Manhattan_dis_high :public h_funcs{
public:
	Manhattan_dis_high() {}
	~Manhattan_dis_high() {}
	float h_funcs::calc_heuristic(pair<int, int> x, pair<int, int > goal) {
		return (abs(goal.first - x.first) + abs(goal.second - x.second)) * 0.25;
	}
};

class Manhattan_dis :public h_funcs {
public:
	Manhattan_dis() {}
	~Manhattan_dis() {}
	float h_funcs::calc_heuristic(pair<int, int> x, pair<int, int > goal) {
		return (abs(goal.first - x.first) + abs(goal.second - x.second));
	}
};

class Euclidean_dis :public h_funcs {
public:
	Euclidean_dis() {};
	~Euclidean_dis() {};
	float h_funcs::calc_heuristic(pair<int, int> x, pair<int, int > goal) {
		return (sqrt(pow((goal.first - x.first),2) + pow((goal.second - x.second),2)));
	}
};

class Diagonal_dis :public h_funcs {
public:
	Diagonal_dis() {};
	~Diagonal_dis() {};
	float h_funcs::calc_heuristic(pair<int, int> x, pair<int, int > goal) {
		int dis_x = abs(x.first - goal.first);
		int dis_y = abs(x.second - goal.second);
		int max_dis = max(dis_x, dis_y);
		int min_dis = min(dis_x, dis_y);
		return (min_dis * 1.4 + max_dis - min_dis);
	}
};

class Diagonal_dis_high :public h_funcs {
public:
	Diagonal_dis_high() {};
	~Diagonal_dis_high() {};
	float h_funcs::calc_heuristic(pair<int, int> x, pair<int, int > goal) {
		int dis_x = abs(x.first - goal.first);
		int dis_y = abs(x.second - goal.second);
		int max_dis = max(dis_x, dis_y);
		int min_dis = min(dis_x, dis_y);
		return (min_dis * 1.4 + (max_dis - min_dis) * 0.25);
	}
};
#endif // !_H_FUNCS_H_

