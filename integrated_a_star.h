#ifndef _INTEGRATED_A_STAR_H_
#define _INTEGRATED_A_STAR_H_

#include "Assignment1.h"
#include "heuritic.h"
#include "weight_astar_heuristic.h"
#include "min_heap.h"
#pragma once
class integrated_a_star
{
public:
	integrated_a_star(vector<vector<map_str>>& origin_map, vector<pair<int, int>> start_goal, vector<pair<int, int>>& path, float& route_cost, float weight_1, float weight_2);
	~integrated_a_star();
	int get_closed_size();
private:
	typedef struct _open_str_ {
		pair<int, int> s;
		float g_h;
	}open_node;
	typedef struct _cmp_{
		bool operator()(const open_node a, const open_node b) {
			return a.g_h < b.g_h;
		}	
	}cmp;
	typedef struct _find_{
		bool operator()(const open_node a, const open_node b) {
			return (a.s.first == b.s.first && a.s.second == b.s.second);
		}	
	}find_str;
	float w_1;
	float w_2;
	typedef struct _intg_a_str_ {
		min_heap<open_node, find_str, cmp> open_list;
		heuristic *hn;
	}intg_astar_str;
	vector<set<pair<int, int>>> close_list;
	vector<intg_astar_str> intg_astar_vec;
	void initial(vector<vector<map_str>>& origin_map);
	bool go_astar(vector<vector<map_str>>& origin, pair<int, int> start, pair<int, int> goal);
	void expand_state(vector<vector<map_str>>& origin, pair<int, int> s, pair<int, int> goal);
	void output_path(vector<vector<map_str>>& origin, vector<pair<int, int>>& path,pair<int, int> start, pair<int, int> goal);
	float move_cost(vector<vector<map_str>>& origin, pair<int, int> a, pair<int, int> b);
	float key(vector<vector<map_str>>& origin, pair<int, int> a, pair<int, int> goal, int );
};


#endif // !_INTEGRATED_A_STAR_H_
