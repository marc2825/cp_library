#include <iostream>
#include <vector>
#include "computational_geometry/rotating_calipers.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    auto h=e::convex_hull({{0,0},{3,0},{3,4},{0,4}});
    auto r=e::convex_diameter(h);
    if(r)std::cout<<e::to_string(r->squared_distance)<<'\n';
}
