#ifndef _ASTAR_HEURISTIC_H_
#define _ASTAR_HEURISTIC_H_
#include "heuritic.h"
#pragma once
class astar_heuristic :public heuristic {
public:
	astar_heuristic(h_funcs *hf) :hf(hf){}
	~astar_heuristic() {}
	float heuristic::heuristic_func(pair<int,int> x, pair<int,int> goal) {
		return hf->calc_heuristic(x, goal);
	}
private:
	h_funcs *hf;
};

#endif // !_ASTAR_HEURISTIC_H_

