#include "Line.h"
#include "LineUtils.h"
#include <vector>

class LineMatcher{
public:
	struct Pair{
	Line L1, L2;
	double err;
	Pair(Line l1_, Line l2_){
	L1 = l1_;
	L2 = l2_;
	err = LineUtils::getError(l1_, l2_);
	}

    void findMatchingPairs(std::vector<Line> l_old, std::vector<Line> l_new, std::vector<LineMatcher::Pair> &matched);
         
};
