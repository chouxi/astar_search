#ifndef _A_STAR_H_
#define _A_STAR_H_
#include <set>
#include "Assignment1.h"
#include "heuritic.h"
#include "uniform_heuristic.h"
#include "astar_heuristic.h"
#include "weight_astar_heuristic.h"
#include "min_heap.h"

#pragma once
typedef struct _fringe_str_ {
	pair<int, int> s;
	float g_h;
}fringe_str;
class astar
{
public:
	astar(vector<vector<map_str>>&, vector<pair<int, int>> start_goal, vector<pair<int, int>>& path, float& route_cost, float weight, h_funcs *hf);
	~astar();
	int get_closed_size();
private:
	/*typedef struct _cmp_ {
		bool operator()(const fringe_str a, const fringe_str b) {
			return a.g_h < b.g_h;
		}
	}cmp;
	set<fringe_str, cmp> fringe;
	set<fringe_str>::iterator find_fringe(pair<int, int> s);*/
	typedef struct _cmp_{
		bool operator()(const fringe_str a, const fringe_str b) {
			return a.g_h < b.g_h;
		}	
	}cmp;
	typedef struct _find_{
		bool operator()(const fringe_str a, const fringe_str b) {
			return (a.s.first == b.s.first && a.s.second == b.s.second);
		}	
	}find_str;
	min_heap<fringe_str, find_str, cmp> fringe;
	set<pair<int, int>> closed;
	bool goastar(vector<vector<map_str>>& origin, pair<int, int> start, pair<int, int> goal);
//	float heuristic(pair<int, int> x, pair<int, int> goal);
	float move_cost(vector<vector<map_str>>& origin, pair<int, int> a, pair<int, int> b);
	void update_vertex(vector<vector<map_str>>& origin, pair<int, int> s, pair<int, int> s_next, pair<int, int> goal);
	void output_path(vector<vector<map_str>>& origin, vector<pair<int, int>>& path,pair<int, int> start, pair<int, int> goal);
	heuristic *hn;
};
#endif // !_A_STAR_H_

