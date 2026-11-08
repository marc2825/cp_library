#include <iostream>
#include <vector>
#include "computational_geometry/polygon.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    std::vector<e::Point> p{{0,0},{3,0},{3,3},{0,3}};
    for(auto q:std::vector<e::Point>{{1,1},{3,2},{4,1}})
        std::cout<<static_cast<int>(e::point_in_polygon(p,q))<<'\n';
}
