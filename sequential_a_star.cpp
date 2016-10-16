#include "stdafx.h"
#include "sequential_a_star.h"


sequential_a_star::sequential_a_star(vector<vector<map_str>>& origin_map, vector<pair<int, int>> start_goal, vector<pair<int, int>>& path, float& route_cost, float weight_1, float weight_2)
{
	if (!origin_map.size() || !origin_map[0].size()) {
		cout << "no map generated" << endl;
		return;
	}
	if (start_goal.size() < 2) {
		cout << "no start_goal generated" << endl;
		return;
	}
	this->w_1 = weight_1;
	this->w_2 = weight_2;
	initial(origin_map);
	cout << "initial succeed" << endl;
	int index = 0;
	if (go_astar(origin_map, start_goal[0], start_goal[1], index)) {
		output_path(index, path, start_goal[0], start_goal[1]);
		route_cost = seq_astar_map_vec[start_goal[1].first][start_goal[1].second].g[index];
		for (int i = 0; i < path.size(); i++)
			cout << path[i].first << "," << path[i].second << endl;
	}
	else {
		cout << "no path found" << endl;
	}
}


sequential_a_star::~sequential_a_star()
{
}

void sequential_a_star::initial(vector<vector<map_str>>& origin_map) {
	seq_astar_vec.resize(HEURISTIC_NUMBER);
	seq_astar_vec[0].hn = new weight_astar_heuristic (w_1, new Manhattan_dis_high() );
	seq_astar_vec[1].hn = new weight_astar_heuristic (w_1, new Manhattan_dis() );
	seq_astar_vec[2].hn = new weight_astar_heuristic (w_1, new Euclidean_dis() );
	seq_astar_vec[3].hn = new weight_astar_heuristic (w_1, new Diagonal_dis() );
	seq_astar_vec[4].hn = new weight_astar_heuristic (w_1, new Diagonal_dis_high() );
	seq_astar_map init;
	init.g.resize(HEURISTIC_NUMBER, FLT_MAX);
	init.parent.resize(HEURISTIC_NUMBER, pair<int, int>{-1, -1});
	vector<seq_astar_map> tmp(origin_map[0].size(), init);
	seq_astar_map_vec.resize(origin_map.size(), tmp);
}

bool sequential_a_star::go_astar(vector<vector<map_str>>& origin, pair<int, int> start, pair<int, int> goal, int & index)
{
	for (int i = 0; i < HEURISTIC_NUMBER; i++) {
		open_node tmp;
		seq_astar_map_vec[start.first][start.second].g[i] = 0;
		tmp.g_h = key(start, goal, i);
		tmp.s = start;
		seq_astar_vec[i].open_list.push_heap(tmp);
	}
	while (!(seq_astar_vec[0].open_list.is_empty())) {
		for (int i = 1; i < HEURISTIC_NUMBER; i++) {
			if (!(seq_astar_vec[0].open_list.is_empty()) && !(seq_astar_vec[i].open_list.is_empty())) {
				if (seq_astar_vec[i].open_list.top().g_h <= w_2 * seq_astar_vec[0].open_list.top().g_h) {
					if (seq_astar_map_vec[goal.first][goal.second].g[i] <= seq_astar_vec[i].open_list.top().g_h) {
						index = i;
						return true;
					}
					else {
						open_node tmp = seq_astar_vec[i].open_list.pop_heap();
						expand_state(origin, tmp.s, goal, i);
						seq_astar_vec[i].close_list.insert(tmp.s);
					}
				}
				else {
					if (seq_astar_map_vec[goal.first][goal.second].g[0] <= seq_astar_vec[0].open_list.top().g_h) {
						index = 0;
						return true;
					}
					else {
						open_node tmp = seq_astar_vec[0].open_list.pop_heap();
						expand_state(origin, tmp.s, goal, 0);
						seq_astar_vec[0].close_list.insert(tmp.s);
					}
				}
			}
		}
	}
	return false;
}

void sequential_a_star::expand_state(vector<vector<map_str>>& origin, pair<int, int> s, pair<int, int> goal, int index)
{
	open_node tmp;
	tmp.s = s;
	seq_astar_vec[index].open_list.remove_heap(tmp);
	for (int i = s.first - 1; i <= s.first + 1 && i < origin.size(); i++) {
		for (int j = s.second - 1; j <= s.second + 1 && j < origin[0].size(); j++) {
			if (i < 0) continue;
			if (j < 0) continue;
			if (i == s.first && j == s.second) continue;
			open_node s_prime;
			s_prime.s = { i, j };
			float cost = move_cost(origin, s, { i, j });
			if (cost < 0) continue;
			if (seq_astar_map_vec[i][j].g[index] > seq_astar_map_vec[s.first][s.second].g[index] + cost) {
				seq_astar_map_vec[i][j].g[index] = seq_astar_map_vec[s.first][s.second].g[index] + cost;
				seq_astar_map_vec[i][j].parent[index] = s;
				if (seq_astar_vec[index].close_list.find({ i, j }) == seq_astar_vec[index].close_list.end()) {
					open_node ins;
					ins.g_h = key({ i,j }, goal, index);
					ins.s = { i,j };
					if(seq_astar_vec[index].open_list.find_heap(ins) >= 0)
						seq_astar_vec[index].open_list.remove_heap(ins);
					seq_astar_vec[index].open_list.push_heap(ins);
				}
			}
		}
	}
}


void sequential_a_star::output_path(int index, vector<pair<int, int>>& path, pair<int, int> start, pair<int, int> goal)
{
	path.clear();
	pair<int, int> tmp = goal;
	while (tmp != start) {
		path.push_back(tmp);
		tmp = seq_astar_map_vec[tmp.first][tmp.second].parent[index];
		cout << tmp.first << "," << tmp.second << endl;
	}
	path.push_back(start);
	reverse(path.begin(), path.end());
}

float sequential_a_star::move_cost(vector<vector<map_str>>& origin, pair<int, int> a, pair<int, int> b) {
	if (origin[b.first][b.second].type == '0')
		return -1;
	int diff_x = a.first - b.first;
	int diff_y = a.second - b.second;
	char type_a = origin[a.first][a.second].type;
	char type_b = origin[b.first][b.second].type;
	if (diff_x == 0 || diff_y == 0) {
		if ((type_a == '1' || type_a == 'a')&& type_b == '1') return 1;
		if ((type_a == '2' || type_a == 'b') && type_b == '2') return 2;
		if (((type_a == '1' || type_a == 'a') && type_b == '2') || ((type_a == '2' || type_a == 'b') && type_b == '1')) return 1.5;
		if (type_b == 'a') {
			switch (type_a) {
			case '1':
				return 1;
			case '2':
				return 1.5;
			case 'a':
				return 0.25;
			case 'b':
				return 0.325;
			}
		}
		if (type_b == 'b') {
			switch (type_a) {
			case '1':
				return 1.5;
			case '2':
				return 2;
			case 'a':
				return 0.325;
			case 'b':
				return 0.5;
			}
		}
	}
	else {
		if ((type_a == '1' || type_a == 'a') && (type_b == '1' || type_b == 'a')) return 1.4;
		if ((type_a == '2' || type_a == 'b') && (type_b == '2' || type_b == 'b')) return 2.8;
		if (((type_a == '1' || type_a == 'a') && (type_a == '2' || type_a == 'b')) || ((type_a == '2' || type_a == 'b') && (type_b == '1' || type_b == 'a'))) return (1.4 + 2.8) / 2;
	}
}

float sequential_a_star::key(pair<int, int> a, pair<int, int> goal, int index)
{
	return seq_astar_map_vec[a.first][a.second].g[index] + seq_astar_vec[index].hn->heuristic_func(a, goal);
}