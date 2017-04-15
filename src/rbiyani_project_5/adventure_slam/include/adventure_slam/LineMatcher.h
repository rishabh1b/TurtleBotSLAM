#include "adventure_slam/Line.h"
#include <vector>

extern double matching_line_threshold;

class LineMatcher{
public:
	struct Pair{
	Line L1, L2;
	double err;
	Pair(Line l1_, Line l2_){
	L1 = l1_;
	L2 = l2_;
	err = l1_.getError(l2_);
	}
};

 static std::vector<int> BruteForcePairs(std::vector<Line> l_old, std::vector<Line> l_new, std::vector<LineMatcher::Pair> &matched);
         
};
