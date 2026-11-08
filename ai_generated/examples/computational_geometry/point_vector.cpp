#include <iostream>
#include <vector>
#include "computational_geometry/exact/primitives.hpp"
namespace e=cp::geometry::exact;
int main(){
    e::Vector a{3,1},b{1,2};
    std::cout<<e::to_string(e::dot(a,b))<<' '<<e::to_string(e::cross(a,b))<<'\n';
    std::cout<<static_cast<int>(e::ccw({0,0},a,b))<<'\n';
}
