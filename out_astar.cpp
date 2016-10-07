#include "stdafx.h"
#include "out_astar.h"


out_astar::out_astar(vector<pair<int, int>> path, float route_cost, string astar_file)
{
	astar_out.open(astar_file);
	if (astar_out.is_open()) {
		if (path.size() && route_cost >= 0) {
			astar_out << route_cost << endl;
			for (int i = 0; i < path.size(); i++) {
				astar_out << "(" << path[i].first << "," << path[i].second << ")" << endl;
			}
		}
	}
}


out_astar::~out_astar()
{
	astar_out.close();
}
