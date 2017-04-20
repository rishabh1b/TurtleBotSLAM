#include "adventure_slam/LineMatcher.h"
#include <vector>
class Localizer{

public:
std::vector<LineMatcher::Pair> matched_pairs;

Localizer();

void estimate();

void estimateGlobalPosition(double curr_shift_x, double curr_shift_y);

double shift_x, shift_y, delta_yaw;

//bool rotating;

private:
double getXEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle);
double getYEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle);
};
