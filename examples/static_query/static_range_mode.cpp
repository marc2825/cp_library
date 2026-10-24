#include <iostream>
#include "../../static_query/static_range_mode.hpp"
int main(){
    cp::StaticRangeMode<int> mode({2,1,2,3,1,2});
    auto a=*mode.mode(0,6),b=*mode.mode(1,5);
    std::cout<<a.first<<' '<<a.second<<'\n'<<b.first<<' '<<b.second<<'\n';
}
