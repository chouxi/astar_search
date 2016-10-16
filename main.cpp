#include "stdafx.h"
#include "ai_map.h"
#include "astar.h"
#include "Assignment1.h"
#include "out_map.h"
#include "in_map.h"
#include "out_astar.h"
#include "cell_info.h"
#include "sequential_a_star.h"
#include <ctime>
#include "sys/timeb.h"
#define MAP_NUM 5
#define S_G_NUM 10
int main(int argc, char *argv[])
{
	cout << "0:test" << endl;
	cout << "1:run" << endl;
	cout << "Please input your choice:";
	int ru;
	scanf_s("%d", &ru);
	switch (ru) {
	case 0: {
		typedef struct _log_info_ {
			float _time;
			int _length;
			float _cost;
			int _expand;
		}log_info;
		float weight[4] = { 0, 1, 1.25, 2};
		vector<h_funcs *> hf;
		vector<vector<vector<log_info>>> storage(4,vector<vector<log_info>>(5, vector<log_info>(50)));
		hf.push_back(new Manhattan_dis());
		hf.push_back(new Manhattan_dis_high());
		hf.push_back(new Euclidean_dis());
		hf.push_back(new Diagonal_dis());
		hf.push_back(new Diagonal_dis_high());
		vector<string> hf_str;
		hf_str.push_back("Manhatton_dis");
		hf_str.push_back("Manhatton_dis_high");
		hf_str.push_back("Euclidean_dis");
		hf_str.push_back("Diagonal_dis");
		hf_str.push_back("Diagonal_dis_high");
		for (int i = 0; i < MAP_NUM; i++) {
			stringstream ss;
			ss << "./maps/map_" << i << ".txt";
			string map_file = ss.str(); 
			Assignment1 *a1 = new Assignment1(120, 160, map_file, "./result.txt");
			ai_map *m = new ai_map(a1->row, a1->col, a1->origin_map, a1->start_goal, a1->hard_center, S_G_NUM);
			out_map * out = new out_map(a1->origin_map, a1->start_goal, a1->hard_center, a1->map_file, a1->row, a1->col);
			delete out;
			stringstream log_s;
			log_s << "./logs/log_" << i << ".txt";
			string log_file = log_s.str();
			ofstream log_out(log_file);
			cout << "======================================" << endl;
			cout << "map" << i << endl;
			for (int j = 0; j < S_G_NUM * 2; j += 2) {
				log_out << "result" << j << endl;
				cout << "result" << j << endl;
				vector<pair<int, int>> tmp;
				tmp.push_back(a1->start_goal[j]);
				tmp.push_back(a1->start_goal[j + 1]);
				for (int w = 0; w < 4; w++) {
					log_out << "weight:\t" << weight[w] << endl;
					cout << "weight:\t" << weight[w] << endl;
					for (int h = 0; h < hf.size(); h++) {
						struct timeb start_t, end_t;
						ftime(&start_t);
						astar *start = new astar(a1->origin_map, tmp, a1->path, a1->route_cost, weight[w], hf[h]);
						ftime(&end_t);
						int expand = start->get_closed_size();
						delete start;
						stringstream result_s;
						result_s << "./results/result_" << i <<"_" << j<< ".txt";
						string result_file = result_s.str();
						out_astar *o_start = new out_astar(a1->path, a1->route_cost, result_file);
						delete o_start;
						log_out << "function:\t" << hf_str[h] << endl;
						log_out << "cost:\t" << a1->route_cost << endl;
						log_out << "length:\t" << a1->path.size() << endl;
						log_out << "runtime:\t" << (end_t.time - start_t.time)* 1000 + (end_t.millitm - start_t.millitm) << "ms" << endl;
						log_out << "------" << endl;
						cout << "function:\t" << hf_str[h] << endl;
						cout << "cost:\t" << a1->route_cost << endl;
						cout << "length:\t" << a1->path.size() << endl;
						cout << "runtime:\t" << (end_t.time - start_t.time)* 1000 + (end_t.millitm - start_t.millitm) << "ms" << endl;
						cout << "------" << endl;
						log_info tmp;
						tmp._time = (end_t.time - start_t.time) * 1000 + (end_t.millitm - start_t.millitm);
						tmp._cost = a1->route_cost;
						tmp._length = a1->path.size();
						tmp._expand = expand;
						storage[w][h][i*MAP_NUM + j] = tmp;
					}
					log_out << "=======" << endl;
					cout << "=======" << endl;
				}
				cout << "-------------------------------------" << endl;
			}
			log_out.close();
			delete m;
		}
		ofstream avg_log("./avg_log.txt");
		for (int i = 0; i < 4; i++) {
			avg_log << "weight:" << weight[i] << endl;
			for (int j = 0; j < 5; j++) {
				avg_log << "heuristic:" << hf_str[j] << endl;
				double _time = 0;
				double _cost = 0;
				double _length = 0;
				double _expand = 0;
				for (int k = 0; k < 50; k++) {
					_time += storage[i][j][k]._time;
					_cost += storage[i][j][k]._cost;
					_length += storage[i][j][k]._length;
					_expand += storage[i][j][k]._expand;
				}
				avg_log <<"cost:\t" << (double)(_cost / 50.0) << endl;
				avg_log << "length:\t" << (double)(_length / 50.0) << endl;
				avg_log << "runtime:\t" << (double)(_time / 50.0) << "ms" << endl;
				avg_log << "expand:\t" << (double)(_expand /50.0) << endl;
			}
		}
		avg_log.close();
		break;
	}
	case 1: {
		Assignment1 *a1 = new Assignment1(120, 160, "./map.txt", "./result.txt");
		a1->did_astar = false;
		while (1) {
			cout << "Please input your choice:" << endl;
			cout << "1. Create new map" << endl;
			cout << "2. Read map from file" << endl;
			cout << "3. Execute A* (WA* etc)" << endl;
			cout << "4. Save path in file" << endl;
			cout << "5. Save map in file" << endl;
			cout << "6. Info of cells" << endl;
			cout << "7. Execute Squential A*" << endl;
			cout << "8. Exit" << endl;
			cout << "Enter your choice:";
			int choice;
			scanf_s("%d", &choice);
			switch (choice) {
			case 1: {
				ai_map *m = new ai_map(a1->row, a1->col, a1->origin_map, a1->start_goal, a1->hard_center, 1);
				delete m;
				a1->did_astar = false;
				break;
			}
			case 2: {
				in_map * in = new in_map(a1->origin_map, a1->map_file, a1->start_goal);
				delete in;
				break;
			}
			case 3: {
				float weight;
				cout << "0: uniform_cost_search" << endl;
				cout << "1: standard A* search" << endl;
				cout << "more than 1: weight A* search" << endl;
				cout << "please input the kind or weight you want to calc heuristic:";
				scanf_s("%f", &weight);
				h_funcs *hf = new Manhattan_dis_high();
				astar *start = new astar(a1->origin_map, a1->start_goal, a1->path, a1->route_cost, weight, hf);
				delete start;
				delete hf;
				a1->did_astar = true;
				break;
			}
			case 4: {
				out_astar *o_start = new out_astar(a1->path, a1->route_cost, a1->result_file);
				delete o_start;
				break;
			}
			case 5: {
				out_map * out = new out_map(a1->origin_map, a1->start_goal, a1->hard_center, a1->map_file, a1->row, a1->col);
				delete out;
				break;
			}
			case 6: {
				if (!(a1->did_astar)) {
					cout << "Didn't do astar yet!" << endl;
					break;
				}
				int x, y;
				cout << "Input x:";
				scanf_s("%d", &x);
				cout << "Input y:";
				scanf_s("%d", &y);
				cell_info *ci = new cell_info(a1->origin_map, x, y, a1->row, a1->col);
				delete ci;
				break;
			}
			case 7: {
				float weight1;
				float weight2;
				cout << "Input weight 1:" << endl;
				scanf_s("%f", &weight1);
				cout << "Input weight 2:" << endl;
				scanf_s("%f", &weight2);
				sequential_a_star *seq_star = new sequential_a_star(a1->origin_map, a1->start_goal, a1->path, a1->route_cost, weight1, weight2);
				delete seq_star;
				a1->did_astar = true;
				break;
			}
			case 8:
				exit(0);
			default:
				break;
			}
		}
	}
	}
	return 0;
}