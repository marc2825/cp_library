#include <iostream>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    const e::Line line({0,0},{2,1});
    auto q=e::projection(line,{1,2});
    std::cout<<q.x.str()<<' '<<q.y.str()<<' '<<e::squared_distance(line,e::Point{1,2}).str()<<'\n';
    const f::Line other({0,0},{2,1});
    std::cout<<(f::distance(other,f::Point{1,2})>1)<<'\n';
}
