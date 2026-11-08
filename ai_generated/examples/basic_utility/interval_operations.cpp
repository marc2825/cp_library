#include <iostream>
#include "basic_utility/interval_operations.hpp"
int main() {
    auto a=cp::merge_intervals<int>({{1,4},{3,6},{6,8}});
    for(auto [l,r]:cp::split_interval(a[0].first,a[0].second,{3,5})) std::cout<<l<<":"<<r<<" ";
    std::cout<<"\n";
}
