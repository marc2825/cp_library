#include <iostream>
#include <vector>
#include "computational_geometry/polygon.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    e::ConvexPolygon p({{0,0},{3,0},{3,3},{0,3},{1,1}});
    std::cout<<static_cast<int>(p.contains({1,1}))<<' '<<static_cast<int>(p.contains({4,1}))<<'\n';
}
