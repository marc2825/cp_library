#include <iostream>
#include "../../offline_query/static_range_inversions.hpp"
int main(){
    auto result=cp::static_range_inversions(std::vector<int>{3,1,2,1},{{0,4},{1,4},{2,2}});
    for(auto count:result)std::cout<<count<<' ';
    std::cout<<'\n';
}
