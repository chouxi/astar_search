#include "stdafx.h"
#include "astar.h"


astar::astar(vector<vector<map_str>>& origin, vector<pair<int,int>> start_goal , vector<pair<int ,int>>& path, float& route_cost, float weight, h_funcs *hf)
{
	if (!weight) hn = new uniform_heuristic(hf);
	else if (weight == 1) hn = new astar_heuristic(hf);
	else if (weight > 1) hn = new  weight_astar_heuristic(weight, hf );
	else {
		cout << "illigeal weight has been input";
		return;
	}
	if (start_goal.size()< 2 || !origin.size()) {
		cout << "no map generate or read" << endl;
		return;
	}
	//cout << "start" << start_goal[0].first << "," << start_goal[0].second << endl;
	//cout << "goal" << start_goal[1].first << "," << start_goal[1].second << endl;
	if (goastar(origin, start_goal[0], start_goal[1])) {
		output_path(origin, path, start_goal[0], start_goal[1]);
		route_cost = origin[start_goal[1].first][start_goal[1].second].gn;
		//for (auto p : path)
		//	cout << p.first << "," << p.second << endl;
	}
	else {
		cout << "no path found" << endl;
	}
}


astar::~astar()
{
}
bool astar::goastar(vector<vector<map_str>>& origin, pair<int,int> start, pair<int, int> goal) {
	origin[start.first][start.second].gn = 0;
	origin[start.first][start.second].parent = start;
	fringe_str start_f;
	float start_h = hn->heuristic_func(start, goal);
	start_f.g_h = origin[start.first][start.second].gn + start_h;
	start_f.s = start;
	origin[start.first][start.second].hn = start_h;
	fringe.push_heap(start_f);
	while (!fringe.is_empty()) {
		fringe_str tmp = fringe.pop_heap();
		if (goal == tmp.s)
			return true;
		closed.insert(tmp.s);
		for (int i = tmp.s.first - 1; i <= tmp.s.first + 1 && i < origin.size() ; i++) {
			for (int j = tmp.s.second - 1; j <= tmp.s.second + 1 && j < origin[0].size() ; j++) {
				if (i < 0) continue;
				if (j < 0) continue;
				if (i == tmp.s.first && j == tmp.s.second) continue;
				if (closed.find({ i, j }) == closed.end()) {
					fringe_str find_tmp;
					find_tmp.s = { i,j };
					if (fringe.find_heap(find_tmp) < 0) {
						origin[i][j].gn = INT_MAX;
						origin[i][j].parent = { -1, -1 };
					}
					update_vertex(origin, tmp.s, { i,j }, goal);
				}
			}
		}
	}
	return false;
}
/*
set<fringe_str>::iterator astar::find_fringe(pair<int, int> s) {
	for (auto it = fringe.begin(); it != fringe.end(); it++) {
		if ((*it).s == s)
			return it;
	}
	return fringe.end();
}
*/
void astar::update_vertex(vector<vector<map_str>> & origin, pair<int, int> s, pair<int, int> s_next, pair<int ,int> goal) {
	float cost = move_cost(origin, s, s_next);
	if (cost < 0) return;
	if (origin[s.first][s.second].gn + cost < origin[s_next.first][s_next.second].gn) {
		origin[s_next.first][s_next.second].gn = origin[s.first][s.second].gn + cost;
		origin[s_next.first][s_next.second].parent = s;
		/*auto it = find_fringe(s_next);
		if (it != fringe.end()) {
			fringe.erase(it);
		}*/
		fringe_str tmp;
		tmp.s = s_next;
		fringe.remove_heap(tmp);
		float tmp_h = hn->heuristic_func(s_next, goal);
		tmp.g_h = origin[s_next.first][s_next.second].gn + tmp_h;
		origin[s_next.first][s_next.second].hn = tmp_h;
		fringe.push_heap(tmp);
	}
}

float astar::move_cost(vector<vector<map_str>>& origin, pair<int, int> a, pair<int, int> b) {
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

void astar::output_path(vector<vector<map_str>>& origin, vector<pair<int, int>>& path, pair<int, int> start, pair<int, int> goal) {
	path.clear();
	pair<int, int> tmp = goal;
	while (tmp != start) {
		path.push_back(tmp);
		tmp = origin[tmp.first][tmp.second].parent;
	}
	path.push_back(start);
	reverse(path.begin(), path.end());
}
int astar::get_closed_size() {
	return closed.size();
}