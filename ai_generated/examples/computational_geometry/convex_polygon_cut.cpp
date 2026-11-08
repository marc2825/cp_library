#include <iostream>
#include <vector>
#include "computational_geometry/convex_cut.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    std::vector<e::Point> p{{0,0},{2,0},{2,2},{0,2}};
    auto q=e::convex_polygon_cut(p,e::Line({0,1},{3,0}));
    std::cout<<e::polygon_area(q).str()<<'\n';
    q=e::convex_polygon_cut(q,e::Line({1,0},{1,1}));
    std::cout<<e::polygon_area(q).str()<<'\n';
}
