#include <iostream>
#include "../../range_query/dynamic_nearest_neighbor.hpp"
int main(){
    cp::DynamicNearestNeighbor<3> points;
    int a=points.insert({0,0,0}),b=points.insert({3,0,0});
    std::cout<<points.nearest({2,0,0})->id<<'\n';
    points.move(a,{2,0,0}); points.erase(b);
    auto result=*points.nearest({2,1,0});
    std::cout<<result.id<<' '<<result.squared_distance<<'\n';
}
