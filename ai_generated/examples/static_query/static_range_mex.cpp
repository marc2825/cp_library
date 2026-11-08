#include <iostream>
#include <vector>
#include "static_query/static_range_mex.hpp"
int main() {
    cp::StaticRangeMex a(std::vector<int>{0,1,0,3,2});
    std::cout << a.mex(0,5) << " " << a.mex(1,4) << " " << a.mex(2,2) << "\n";
}
