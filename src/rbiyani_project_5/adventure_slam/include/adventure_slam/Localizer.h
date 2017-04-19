#include "adventure_slam/LineMatcher.h"
#include <vector>
class Localizer{

public:
std::vector<LineMatcher::Pair> matched_pairs;

Localizer();

void estimateRotation();

void estimateTranslation();

void estimate();

double shift_x, shift_y, delta_yaw;

//bool rotating;

private:
double getXEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle);
double getYEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle);
};
