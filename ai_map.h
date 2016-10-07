#ifndef AI_MAP_H
#define AI_MAP_H
#include "Assignment1.h"
#include <algorithm>
//random algorithm dependency
#include "Windows.h"
#include "Winbase.h"
#include "BaseTsd.h"
#include "WinDef.h"
#include "WinNT.h"
#include "Wincrypt.h"
#include <assert.h>
using namespace std;
class ai_map
{
public:
    ai_map(int row, int col, vector<vector<map_str>> &origin_map, vector<pair<int,int>>& start_goal, vector<pair<int,int>>& hard_center, int goal_count);
    ~ai_map();
private:
    int col;
    int row;
	int goal_count;
    vector<vector<map_str>> origin_map;
	vector<pair<int, int>> hard_center;
	vector<pair<int, int>> start_goal;
    vector<vector<map_str>> highway_map;
    vector<vector<map_str>> highway_tmp_map;
    void start_generate();
    void generate_unblock();
    void generate_hard();
    bool generate_highway();
    bool generate_one_highway(int);
	bool check_highway(int, int, int);
	bool diff_direction(int start_x, int start_y, int direction, pair<int,int> & dest, int );
	bool move_next(int direction, pair<int, int> & dest, int& count, int );
    void generate_block();
    void generate_start_goal();
	bool check_bound(pair<int,int>&);
    //int my_random(int start, int end);
	int my_random();
};

#endif // AI_MAP_H