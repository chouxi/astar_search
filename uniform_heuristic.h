#ifndef _UNIFORM_HEURISTIC_H_
#define _UNIFORM_HEURISTIC_H_
#include "heuritic.h"
#pragma once
class uniform_heuristic : public heuristic {
public:
	uniform_heuristic(h_funcs * hf) :hf(hf){}
	~uniform_heuristic() {}
	float heuristic::heuristic_func(pair<int,int> x, pair<int,int> goal) {
		return 0;
	}
private:
	h_funcs *hf;
};

#endif // !_UNIFORM_HEURISTIC_H_


