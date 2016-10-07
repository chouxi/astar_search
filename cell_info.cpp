#include "stdafx.h"
#include "cell_info.h"


cell_info::cell_info(vector<vector<map_str>> origin, int x, int y, int row, int col)
{
	map_str cell = origin[x][y];
	if (x < 0 || x >= row || y < 0 || y >= col) {
		cout << "Please input legal x,y " << row << "," << col << endl;
		return;
	}
	if (cell.gn < 0 || cell.hn < 0) {
		cout << "No f, g, or h info in this cell" << endl;
		return;
	}
	cout << "fn:" << cell.gn + cell.hn << endl;
	cout << "gn:" << cell.gn << endl;
	cout << "hn:" << cell.hn << endl;
}


cell_info::~cell_info()
{
}
