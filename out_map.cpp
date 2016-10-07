#include "stdafx.h"
#include "out_map.h"


out_map::out_map(vector<vector<map_str>> map, vector<pair<int, int>> start_goal,vector<pair<int, int>> hard_center, string map_file, int row, int col)
{
	out_m_file.open(map_file);
	if (!map.size() || !start_goal.size() || !hard_center.size()) {
		cout << "no map in memory" << endl;
		return;
	}
	if (!out_m_file.is_open()) {
		cout << "cannot open map file" << endl;
		return;
	}
	output_map(map, start_goal, hard_center, row, col);
}


out_map::~out_map()
{
	out_m_file.close();
}

void out_map::output_map(vector<vector<map_str>> map, vector<pair<int, int>> start_goal, vector<pair<int, int>> hard_center,int row, int col) {
	for (int i = 0; i < start_goal.size(); i++)
		out_m_file << start_goal[i].first << "," << start_goal[i].second << endl;
	for (int i = 0; i < hard_center.size(); i++)
		out_m_file << hard_center[i].first << "," << hard_center[i].second << endl;
	out_m_file << row << "," << col << endl;
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			map_str tmp = map[i][j];
			if (tmp.highway_num == '0') {
				out_m_file << tmp.type;
			}
			else {
				out_m_file << tmp.type << tmp.highway_num;
			}
			if(j < col - 1)
				out_m_file << ",";
		}
		out_m_file << endl;
	}
}