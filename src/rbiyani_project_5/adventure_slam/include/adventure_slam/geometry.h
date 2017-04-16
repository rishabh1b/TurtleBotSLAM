#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std;

struct mypoint
{
    int x;
    int y;
};

ostream& operator<< (ostream &out, const mypoint &point){
        out<< point.x << ", " << point.y;
        return out;
    }

void convert_to_grid(int &grid_x , int &grid_y, double x, double y, double origin_x, double origin_y, int size_x, int size_y, float resolution);

void convert_to_world(double &x, double &y, int grid_x , int grid_y,  double origin_x, double origin_y, int size_x, int size_y, float resolution);

int to_index(int grid_x, int grid_y, int size_x);

vector<mypoint> bresenham(int x0 , int y0, int x1, int y1);

