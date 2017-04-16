#include "adventure_slam/geometry.h"

using namespace std;

void convert_to_grid(int &grid_x , int &grid_y, double x, double y, double origin_x, double origin_y, int size_x, int size_y, float resolution){
	if(x>=origin_x && y>=origin_y && x< origin_x + size_x * resolution && y< origin_y + size_y * resolution){
		grid_x = floor((x - origin_x)/resolution);
		grid_y = floor((y - origin_y)/resolution);
	}
}

void convert_to_world(double &x, double &y, int grid_x, int grid_y, double origin_x, double origin_y, int size_x, int size_y, float resolution){
	x=grid_x*resolution+origin_x + 0.5*resolution;
	y=grid_y*resolution+origin_y + 0.5*resolution;
}

int to_index(int grid_x, int grid_y, int size_x){
	return grid_y * size_x + grid_x;
}

vector<mypoint> bresenham(int x0 , int y0, int x1, int y1){
	
	int dx,dy;

	dx = x1 - x0;
	dy = y1 - y0;

	bool is_steep = abs(dy) > abs(dx);

	if(is_steep){
		swap(x0,y0);
		swap(x1,y1);
	}

	bool swapped = false;

	if(x0 > x1){
		swap(x0,x1);
		swap(y0,y1);
		swapped = true;
	}

	dx = x1 - x0;
   	dy = y1 - y0;

	double error;
	error = floor(dx / 2.0);

	int ystep = (y0 < y1) ? 1 : -1;
	int y = y0;

	vector<mypoint> a;
	mypoint b;
		

	for (int x= x0; x < x1+1; x++){
		if(is_steep){
			b.x = y;
			b.y = x; 
		}
		else {
			b.x = x;
			b.y = y;
		}
		a.push_back(b);
		error -= abs(dy);
		
		if (error < 0){
            y += ystep;
            error += dx;
		}		
	}

	 if (swapped){
			reverse(a.begin(),a.end());
		}
	
	return a;

}


