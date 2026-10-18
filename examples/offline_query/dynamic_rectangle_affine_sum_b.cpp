#include <iostream>
#include "offline_query/dynamic_rectangle_affine_sum.hpp"
int main() {
    cp::DynamicRectangleAffineSum<> s; s.add_point(1,1,3); s.apply(0,0,3,3,2,1); s.add_point(2,2,5);
    std::cout << s.sum(0,0,3,3) << "\n";
}
