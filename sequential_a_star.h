#ifndef _SEQUENTIAL_A_STAR_H_
#define _SEQUENTIAL_A_STAR_H_
#include "Assignment1.h"
#include "heuritic.h"
#include "weight_astar_heuristic.h"
#include "min_heap.h"


#pragma once
class sequential_a_star
{
public:
	sequential_a_star(vector<vector<map_str>>& origin_map, vector<pair<int, int>> start_goal, vector<pair<int, int>>& path, float& route_cost, float weight_1, float weight_2);
	~sequential_a_star();
	int get_closed_size();
private:
	typedef struct _seq_a_star_ {
		vector<float> g;
		vector<pair<int, int>> parent;
	}seq_astar_map;
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
	typedef struct _seq_a_str_ {
		set<pair<int, int>> close_list;
		min_heap<open_node, find_str, cmp> open_list;
		heuristic *hn;
	}seq_astar_str;
	vector<seq_astar_str> seq_astar_vec;
	vector<vector<seq_astar_map>> seq_astar_map_vec;
	void initial(vector<vector<map_str>>& origin_map);
	bool go_astar(vector<vector<map_str>>& origin, pair<int, int> start, pair<int, int> goal, int& index);
	void expand_state(vector<vector<map_str>>& origin, pair<int, int> s, pair<int, int> goal, int index);
	void output_path(int index, vector<pair<int, int>>& path,pair<int, int> start, pair<int, int> goal);
	float move_cost(vector<vector<map_str>>& origin, pair<int, int> a, pair<int, int> b);
	float key(pair<int, int> a, pair<int, int> goal, int );
};

#endif // !_SEQUENTIAL_A_STAR_H_
