#include <iostream>
#include <vector>
#include "computational_geometry/convex_hull.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    auto h=e::convex_hull({{0,0},{2,0},{2,2},{0,2},{1,1},{0,0}});
    for(auto p:h)std::cout<<p.x<<' '<<p.y<<'\n';
}
