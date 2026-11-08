#include <iostream>
#include <vector>
#include "computational_geometry/angular_sort.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    auto p=e::angular_sort({{-1,0},{0,-1},{0,0},{0,1},{1,0}});
    for(auto q:p)std::cout<<q.x<<' '<<q.y<<'\n';
}
