#ifndef _WEIGHT_ASTAR_HEURISTIC_H_
#define _WEIGHT_ASTAR_HEURISTIC_H_
#include "heuritic.h"
#pragma once
class weight_astar_heuristic : public heuristic {
public:
	weight_astar_heuristic(float w, h_funcs *hf) :hf(hf), weight(w) {}
	~weight_astar_heuristic() {}
	float heuristic::heuristic_func(pair<int,int> x, pair<int,int> goal) {
		return hf->calc_heuristic(x, goal) * weight;
	}
private:
	h_funcs *hf;
	float weight;
};

#endif // !_WEIGHT_ASTAR_HEURISTIC_H_

