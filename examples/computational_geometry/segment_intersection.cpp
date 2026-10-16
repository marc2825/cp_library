#include <iostream>
#include <vector>
#include "computational_geometry/exact/primitives.hpp"
namespace e=cp::geometry::exact;
int main(){
    std::cout<<e::segments_intersect({{0,0},{2,2}},{{0,2},{2,0}})<<'\n';
    std::cout<<e::segments_intersect({{0,0},{0,0}},{{1,0},{2,0}})<<'\n';
}
