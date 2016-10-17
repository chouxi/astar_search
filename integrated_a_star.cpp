#include "stdafx.h"
#include "integrated_a_star.h"
integrated_a_star::integrated_a_star(vector<vector<map_str>>& origin_map, vector<pair<int, int>> start_goal, vector<pair<int, int>>& path, float & route_cost, float weight_1, float weight_2)
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
	cout << "init su" << endl;
	int index = 0;
	if (go_astar(origin_map, start_goal[0], start_goal[1])) {
		output_path(origin_map, path, start_goal[0], start_goal[1]);
		route_cost = origin_map[start_goal[1].first][start_goal[1].second].gn;
		cout << route_cost << endl;
		/*for (int i = 0; i < path.size(); i++)
			cout << path[i].first << "," << path[i].second << endl;*/
	}
	else {
		cout << "no path found" << endl;
	}
}

integrated_a_star::~integrated_a_star()
{
	for (auto vec : intg_astar_vec)
		delete vec.hn;
}

void integrated_a_star::initial(vector<vector<map_str>>& origin_map)
{
	close_list.resize(2,set<pair<int,int>>());
	intg_astar_vec.resize(HEURISTIC_NUMBER);
	intg_astar_vec[0].hn = new weight_astar_heuristic (w_1, new Manhattan_dis_high() );
	intg_astar_vec[1].hn = new weight_astar_heuristic (w_1, new Manhattan_dis() );
	intg_astar_vec[2].hn = new weight_astar_heuristic (w_1, new Euclidean_dis() );
	intg_astar_vec[3].hn = new weight_astar_heuristic (w_1, new Diagonal_dis() );
	intg_astar_vec[4].hn = new weight_astar_heuristic (w_1, new Diagonal_dis_high() );
	for (int i = 0; i < origin_map.size(); i++) {
		for (int j = 0; j < origin_map[0].size(); j++) {
			origin_map[i][j].gn = FLT_MAX;
			origin_map[i][j].parent = { -1,-1 };
		}
	}
}

bool integrated_a_star::go_astar(vector<vector<map_str>>& origin, pair<int, int> start, pair<int, int> goal)
{
	origin[start.first][start.second].gn = 0;
	for (int i = 0; i < HEURISTIC_NUMBER; i++) {
		open_node tmp;
		tmp.g_h = key(origin, start, goal, i);
		tmp.s = start;
		intg_astar_vec[i].open_list.push_heap(tmp);
	}
	while (!(intg_astar_vec[0].open_list.is_empty())) {
		for (int i = 1; i < HEURISTIC_NUMBER; i++) {
			if (!(intg_astar_vec[0].open_list.is_empty())) {
				if ( !(intg_astar_vec[i].open_list.is_empty()) && intg_astar_vec[i].open_list.top().g_h <= w_2 * intg_astar_vec[0].open_list.top().g_h) {
					if (origin[goal.first][goal.second].gn <= intg_astar_vec[i].open_list.top().g_h)
						return true;
					else {
						open_node tmp_1 = intg_astar_vec[i].open_list.pop_heap();
						expand_state(origin, tmp_1.s, goal);
						close_list[1].insert(tmp_1.s);
					}
				}
				else {
					if (origin[goal.first][goal.second].gn <= intg_astar_vec[0].open_list.top().g_h)
						return true;
					else {
						open_node tmp_0 = intg_astar_vec[0].open_list.pop_heap();
						expand_state(origin, tmp_0.s, goal);
						close_list[0].insert(tmp_0.s);
					}
				}
			}
		}
	}
	return false;
}

void integrated_a_star::expand_state(vector<vector<map_str>>& origin, pair<int, int> s, pair<int, int> goal)
{
	open_node tmp;
	tmp.s = s;
	for (int i = 0; i < HEURISTIC_NUMBER; i++) {
		if (intg_astar_vec[i].open_list.find_heap(tmp) >= 0)
			intg_astar_vec[i].open_list.remove_heap(tmp);
	}
	for (int i = s.first - 1; i <= s.first + 1 && i < origin.size(); i++) {
		for (int j = s.second - 1; j <= s.second + 1 && j < origin[0].size(); j++) {
			if (i < 0) continue;
			if (j < 0) continue;
			if (i == s.first && j == s.second) continue;
			open_node s_prime;
			s_prime.s = { i, j };
			float cost = move_cost(origin, s, { i, j });
			if (cost < 0) continue;
			if (origin[i][j].gn > origin[s.first][s.second].gn + cost) {
				origin[i][j].gn = origin[s.first][s.second].gn + cost;
				origin[i][j].parent = s;
				if (close_list[0].find({ i, j }) == close_list[0].end()) {
					open_node ins;
					ins.g_h = key(origin, { i,j }, goal, 0);
					ins.s = { i,j };
					if (intg_astar_vec[0].open_list.find_heap(ins) >= 0)
						intg_astar_vec[0].open_list.remove_heap(ins);
					intg_astar_vec[0].open_list.push_heap(ins);
					if (close_list[1].find({ i, j }) == close_list[1].end()) {
						for (int k = 1; k < HEURISTIC_NUMBER; k++) {
							if (key(origin, { i, j }, goal, k) <= w_2 * key(origin, { i, j }, goal, 0)) {
								open_node ins_iad;
								ins_iad.g_h = key(origin, { i,j }, goal, k);
								ins_iad.s = { i,j };
								if(intg_astar_vec[k].open_list.find_heap(ins_iad) >= 0)
									intg_astar_vec[k].open_list.remove_heap(ins_iad);
								intg_astar_vec[k].open_list.push_heap(ins_iad);
							}
						}
					}
				}
			}
		}
	}
}

void integrated_a_star::output_path(vector<vector<map_str>>& origin,vector<pair<int, int>>& path, pair<int, int> start, pair<int, int> goal)
{
	path.clear();
	pair<int, int> tmp = goal;
	while (tmp != start) {
		path.push_back(tmp);
		tmp = origin[tmp.first][tmp.second].parent;
	}
	path.push_back(start);
	reverse(path.begin(), path.end());
}

float integrated_a_star::move_cost(vector<vector<map_str>>& origin, pair<int, int> a, pair<int, int> b)
{
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

float integrated_a_star::key(vector<vector<map_str>>& origin, pair<int, int> a, pair<int, int> goal, int index)
{
	return origin[a.first][a.second].gn + intg_astar_vec[index].hn->heuristic_func(a, goal);
}
int integrated_a_star::get_closed_size() {
	set <pair<int, int>> close_list_res;
	for (int i = 0; i < 2; i++) {
		for (auto it = close_list[i].begin(); it != close_list[i].end(); it++) {
			close_list_res.insert(*it);
		}
	}
	return close_list_res.size();
}
