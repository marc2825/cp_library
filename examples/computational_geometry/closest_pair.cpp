#include <iostream>
#include <vector>
#include "computational_geometry/closest_pair.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    auto r=e::closest_pair({{0,0},{3,4},{1,1},{7,8}});
    if(r)std::cout<<r->first<<' '<<r->second<<' '<<e::to_string(r->squared_distance)<<'\n';
}
