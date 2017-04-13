#include "adventure_slam/LineMatcher.h"
#include <vector>
class Localizer{

public:
std::vector<LineMatcher::Pair> matched_pairs;

double estimateRotation();

double estimateTranslation();

};
