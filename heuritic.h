#ifndef _HEURISTIC_H_
#define _HEURISTIC_H_
#include "Assignment1.h"
#include "h_funcs.h"
#pragma once
class heuristic {
public:
	virtual float heuristic_func(pair<int, int> x, pair<int, int> goal) = 0;
};
#endif // !_HEURISTIC_H_

