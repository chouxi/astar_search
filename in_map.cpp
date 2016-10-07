#include "stdafx.h"
#include "in_map.h"


in_map::in_map(vector<vector<map_str>>& origin, string map_file, vector<pair<int, int>>& start_goal)
{
	map_in.open(map_file);
	if (map_in.is_open()) {
		if (!start_goal.size()) {
			start_goal.push_back({0,0});
			start_goal.push_back({0,0});
		}
		origin.clear();
		int count = 0;
		char points_string[2][1024];
		char trash[1024];
		while (count < 11) {
			count++;
			if (count == 1) map_in.getline(points_string[0], 1024);
			else if (count == 2) map_in.getline(points_string[1], 1024);
			else
				map_in.getline(trash, 1024);
		}
		for (int i = 0; i < 2; i++) {
			char points[4];
			int j = 0;
			while (points_string[i][j] != ',') {
				points[j] = points_string[i][j];
				j++;
			}
			j++;
			start_goal[i].first = atoi(points);
			memset(points, '\0', 3);
			int x = 0;
			while (points_string[i][j] != '\0') {
				points[x] = points_string[i][j];
				j++;
				x++;
			}
			start_goal[i].second = atoi(points);
			memset(points, '\0', 3);
		}
		cout << start_goal[0].first << ", " << start_goal[0].second << endl;
		cout << start_goal[1].first << ", " << start_goal[1].second << endl;
		while (!map_in.eof()) {
			vector<map_str> tmp;
			char file_c;
			do {
				map_in.get(file_c);
				map_str tmp_str;
				int count = 0;
				while (file_c != ',' && file_c != '\n' ) {
					if (!count) {
						tmp_str.type = file_c;
						tmp_str.highway_num = '0';
						count++;
					}
					else {
						tmp_str.highway_num = file_c;
					}
					map_in.get(file_c);
				}
				tmp.push_back(tmp_str);
			} while (file_c != '\n');
			if (tmp.size() == 1) break;
			origin.push_back(tmp);
		}
	}
	origin[start_goal[0].first][start_goal[0].second].highway_num = 's';
	origin[start_goal[1].first][start_goal[1].second].highway_num = 'e';
}


in_map::~in_map()
{
	map_in.close();
}
