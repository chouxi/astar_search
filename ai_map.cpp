#include "stdafx.h"
#include "ai_map.h"
ai_map::ai_map(int row, int col, vector<vector<map_str>> & origin_map,vector<pair<int,int>>& start_goal, vector<pair<int,int>>& hard_center, int count)
{
	origin_map.clear();
    this->col = col;
    this->row = row;
	this->goal_count = count;
	start_generate();
	origin_map = this->origin_map;
	start_goal = this->start_goal;
	hard_center = this->hard_center;
}
ai_map::~ai_map(){
}

void  ai_map::start_generate(){
    generate_unblock();
    generate_hard();
	
	highway_map = highway_tmp_map = origin_map;
	while (!generate_highway()) {
		highway_map = highway_tmp_map = origin_map;
		cout << "generate all highways again!" << endl;
	}
	cout << "generate success" << endl;
	origin_map = highway_map;
	highway_map.clear();
	highway_tmp_map.clear();
	generate_block();
	generate_start_goal();
}

void ai_map::generate_unblock(){
    for(int i = 0; i < row; i ++){
        vector<map_str> tmp;
        for(int j = 0; j < col; j++){
			map_str tmp_str;
			tmp_str.type = '1';
			tmp_str.highway_num = '0';
            tmp.push_back(tmp_str);
        }
        origin_map.push_back(tmp);
    }
}

void ai_map::generate_hard(){
    int count = 0;
    bool same_flag= false;
    //random center of hard squre
    while(count < 8){
        int x = my_random()%(row - 30);
        int y = my_random()%(col - 30);
		//cout << x << " " << y << endl;
        for(auto p: hard_center){
            if(p.first == x && p.second == y){
                same_flag = true;
                break;
            }
        }
        if(!same_flag){
            count ++;
            hard_center.push_back({x,y});
            same_flag = false;
        }
    }
    //generate random blocks
    for(int v = 0; v < hard_center.size(); v ++){
        int start_row = hard_center[v].first;
        int start_col = hard_center[v].second;
        int range_row = 30 + start_row;
        int range_col = 30 + start_col;
        start_row = 0 ? start_row < 0 : start_row;
        start_col = 0 ? start_col < 0 : start_col;
        for(int i = start_row; i <= range_row && i < row; i ++){
            for(int j = start_col; j <= range_col && j < col; j ++){
                if(my_random()%1000 > 500)
                    origin_map[i][j].type = '2';
            }
        }
    }
}
bool ai_map::move_next(int direction, pair<int, int> & dest, int& count, int high_num) {
	for (int i = 0; i < 20; i++) {
		switch (direction)
		{
		case 0:
			dest.second++;
			break;
		case 1:
			dest.second--;
			break;
		case 2:
			dest.first--;
			break;
		case 3:
			dest.first++;
			break;
		default:
			break;
		}
		count++;
		if (!check_highway(dest.first, dest.second, high_num)) return false;
		if (check_bound(dest) && count >= 99) return true;
	}
	return true;
}
bool ai_map::check_highway(int x, int y, int high_num) {
	if (x < 0 || x > row - 1) return false;
	if (y < 0 || y > col - 1) return false;
	char tmp_c = highway_tmp_map[x][y].type;
	if (tmp_c == 'a' || tmp_c == 'b')
		return false;
	else if (tmp_c == '1') {
		highway_tmp_map[x][y].type = 'a';
		highway_tmp_map[x][y].highway_num = high_num + '0';
	}
	else if (tmp_c == '2') {
		highway_tmp_map[x][y].type = 'b';
		highway_tmp_map[x][y].highway_num = high_num + '0';
	}
	return true;
}
bool ai_map::diff_direction(int start_x, int start_y, int direction, pair<int,int> &dest, int high_num) {
	if (direction == 0) {
		if (start_y > row - 20) return false;
		for (int i = start_y; i < start_y + 20; i++)
			if (!check_highway(start_x, i, high_num)) return false;
		dest = { start_x, start_y + 19 };
	}
	else if (direction == 1) {
		if (start_y < 20) return false;
		for (int i = start_y; i > start_y - 20; i--)
			if (!check_highway(start_x, i, high_num)) return false;
		dest = { start_x, start_y - 19 };
	}
	else if (direction == 2) {
		if (start_x < 20) return false;
		for (int i = start_x; i > start_x - 20; i--)
			if (!check_highway(i, start_y, high_num)) return false;
		dest = { start_x - 19, start_y };
	}
	else {
		if (start_x > row - 20) return false;
		for (int i = start_x; i < start_x + 20; i++)
			if (!check_highway(i, start_y, high_num)) return false;
		dest = { start_x + 19, start_y };
	}
}
bool ai_map::generate_highway() {
	int count = 0;
	int high_num = 1;
	while(high_num <= 4) {
		if (count >= 99) return false;
		if (!generate_one_highway(high_num)) {
			highway_tmp_map = highway_map;
			count++;
			//cout << "highway number " << high_num << "again" << endl;
		}
		else{
			count = 0;
			high_num++;
			highway_map = highway_tmp_map;
		}
	}
	return true;
}
bool ai_map::check_bound(pair<int, int> & point) {
	if (point.first == 0 || point.first == row || point.second == 0 || point.second == col)
		return true;
	return false;
}
bool ai_map::generate_one_highway(int high_num) {
	int bound = my_random() % 1000;
	//int direction = my_random() % 900;
	int chosen_dir = 0;//0:right 1:left 2:up 3:down
	pair<int, int> dest = {0,0};
	//up
	if (bound < 250) {
		int start_y = 1 + my_random() % (col - 3);
		int start_x = 0;
		/*if (direction < 300) {
			chosen_dir = 0;
			if (!diff_direction(start_x, start_y, chosen_dir, dest)) return false;
		}
		else if(direction >= 300 && direction < 600){*/
			chosen_dir = 3;
			if (!diff_direction(start_x, start_y, chosen_dir, dest, high_num)) return false;
		/*}
		else {
			chosen_dir = 1;
			if (!diff_direction(start_x, start_y, chosen_dir, dest)) return false;
		}*/
	}
	//left
	else if (bound >= 250 && bound < 500) {
		int start_y = 0;
		int start_x = 1 + my_random() % (row - 3);
		//if (direction < 300) {
			chosen_dir = 0;
			if (!diff_direction(start_x, start_y, chosen_dir, dest, high_num)) return false;
		/*}
		else if(direction >= 300 && direction < 600){
			chosen_dir = 2;
			if (!diff_direction(start_x, start_y, chosen_dir, dest)) return false;
		}
		else {
			chosen_dir = 3;
			if (!diff_direction(start_x, start_y, chosen_dir, dest)) return false;
		}*/
	}
	//down
	else if (bound >= 500 && bound < 750) {
		int start_y = 1 + my_random() % (col - 3);
		int start_x = row - 1;
		/*if (direction < 300) {
			chosen_dir = 0;
			if (!diff_direction(start_x, start_y, chosen_dir, dest)) return false;
		}
		else if(direction >= 300 && direction < 600){*/
			chosen_dir = 2;
			if (!diff_direction(start_x, start_y, chosen_dir, dest, high_num)) return false;
		/*}
		else {
			chosen_dir = 1;
			if (!diff_direction(start_x, start_y, chosen_dir, dest)) return false;
		}*/
	}
	// right
	else if (bound >= 750 && bound < 1000) {
		int start_y = col - 1;
		int start_x = 1 + my_random() % (row - 3);
		//if (direction < 300) {
			chosen_dir = 1;
			if (!diff_direction(start_x, start_y, chosen_dir, dest, high_num)) return false;
		/*}
		else if(direction >= 300 && direction < 600){
			chosen_dir = 2;
			if (!diff_direction(start_x, start_y, chosen_dir, dest)) return false;
		}
		else {
			chosen_dir = 3;
			if (!diff_direction(start_x, start_y, chosen_dir, dest)) return false;
		}*/
	}
	//generater the last next stpes and the remain steps.
	int count = 0;
	while(1){
		int next_dir = my_random() % 1000;
		if (next_dir >= 100 && next_dir < 900) {
			if (!move_next(chosen_dir, dest, count, high_num)) return false;
		}
		//left relative to chosen
		else if(next_dir < 100){
			switch (chosen_dir) {
			case 0:
				chosen_dir = 2;
				break;
			case 1:
				chosen_dir = 3;
				break;
			case 2:
				chosen_dir = 1;
				break;
			case 3:
				chosen_dir = 0;
				break;
			}
			if (!move_next(chosen_dir, dest, count, high_num)) return false;
		}
		//right relative to chosen
		else {
			switch (chosen_dir) {
			case 0:
				chosen_dir = 3;
				break;
			case 1:
				chosen_dir = 2;
				break;
			case 2:
				chosen_dir = 0;
				break;
			case 3:
				chosen_dir = 3;
				break;
			}
			if (!move_next(chosen_dir, dest, count, high_num)) return false;
		}
		if (check_bound(dest) && count >= 99)
			return true;
	}
	return true;
}
void ai_map::generate_block() {
	for (int i = 0; i < 3840;) {
		int x = my_random() % row;
		int y = my_random() % col;
		if (origin_map[x][y].type == 'a' || origin_map[x][y].type == 'b') continue;
		origin_map[x][y].type = '0';
		i++;
	}
}
void ai_map::generate_start_goal() {
	vector<pair<int, int>> sample;
	for (int i = 0; i < 20; i++) {
		for (int j = 0; j < 20; j++) {
			if (origin_map[i][j].type != '0')
				sample.push_back({ i, j });
		}
		for (int j = col - 20; j < col; j++) {
			if (origin_map[i][j].type != '0')
				sample.push_back({ i, j });
		}
	}
	for (int i = row - 20; i < row; i++) {
		for (int j = 0; j < 20; j++) {
			if (origin_map[i][j].type != '0')
				sample.push_back({ i, j });
		}
		for (int j = col - 20; j < col; j++) {
			if (origin_map[i][j].type != '0')
				sample.push_back({ i, j });
		}
	}
	int count = 0;
	while (1) {
		if (goal_count == count)
			return;
		int start = my_random() % sample.size();
		int goal = my_random() % sample.size();
		long square = pow(abs(sample[start].first - sample[goal].first), 2) + pow(abs(sample[start].second - sample[goal].second), 2);
		long distance = sqrt(square);
		//cout << distance << endl;
		if (distance >= 100) {
			origin_map[sample[start].first][sample[start].second].highway_num = 's';
			origin_map[sample[goal].first][sample[goal].second].highway_num = 'e';
			start_goal.push_back({ sample[start].first, sample[start].second });
			start_goal.push_back({ sample[goal].first, sample[goal].second });
			sample.erase(sample.begin() + start);
			sample.erase(sample.begin() + goal);
			count++;
		}
	}
}
/*int random_helper(int i) {
	//srand(time(NULL));
	return rand() % i;
}
//random number [start, end]
int ai_map::my_random(int start, int end){
    int v_len = end - start + 1;
    vector<int> v(v_len);
    for(int i = start, j = 0; i <= end; i ++, j ++)
        v[j] = i;
    random_shuffle(v.begin(), v.end(),random_helper);
    return (v[random_helper(rand())%v_len]);
}*/
//generate random number by using crypt knowledge
//https://www.zhihu.com/question/20397465
int ai_map::my_random() {
	int rnum = 0;
#if defined _MSC_VER
#if defined _WIN32_WCE
	CeGenRandom(sizeof(int), (PBYTE)&rnum);
#else
	HCRYPTPROV hProvider = 0;
	const DWORD dwLength = sizeof(int);
	BYTE pbBuffer[dwLength] = {};
	DWORD result = ::CryptAcquireContext(&hProvider, 0, 0, PROV_RSA_FULL, CRYPT_VERIFYCONTEXT | CRYPT_SILENT);
	assert(result);
	result = ::CryptGenRandom(hProvider, dwLength, pbBuffer);
	rnum = *(int*)pbBuffer;
	assert(result);
	::CryptReleaseContext(hProvider, 0);
#endif
#elif defined __GNUC__
	int fd = open("/dev/urandom", O_RDONLY);
	if (fd != -1) {
		(void)read(fd, (void *)&rnum, sizeof(int));
		(void)close(fd);
	}
#endif
	if (rnum < 0) rnum = 0 - rnum;
	return rnum;
}