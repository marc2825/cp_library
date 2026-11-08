#include <iostream>
#include <vector>
#include "computational_geometry/exact/primitives.hpp"
namespace e=cp::geometry::exact;
int main(){
    auto r=e::line_intersection(e::Line({0,0},{3,3}),e::Line({0,2},{4,0}));
    if(r.point)std::cout<<r.point->x.str()<<' '<<r.point->y.str()<<'\n';
}
