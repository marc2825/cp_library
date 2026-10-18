#include <iostream>
#include "offline_query/kd_tree.hpp"
int main() {
    cp::KdTree<long long> s({{0,0},{4,3},{2,1}});
    std::cout << s.nearest(2,2)->first << "\n";
}
