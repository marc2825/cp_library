#include <iostream>
#include <vector>
#include "computational_geometry/polygon.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    std::vector<e::Point> p{{0,0},{3,0},{0,1}};
    std::cout<<e::to_string(e::signed_area2(p))<<' '<<e::polygon_area(p).str()<<'\n';
}
