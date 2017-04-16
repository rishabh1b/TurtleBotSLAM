#include "adventure_slam/LineMatcher.h"
#include <vector>
class Localizer{

public:
std::vector<LineMatcher::Pair> matched_pairs;

Localizer();

void estimateRotation();

void estimateTranslation();

double shift_x, shift_y, delta_yaw;

private:
double getXEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle);
double getYEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle);
};
