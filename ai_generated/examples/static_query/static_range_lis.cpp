#include <iostream>
#include "../../static_query/static_range_lis.hpp"
int main(){
    cp::StaticRangeLIS<int> lis({3,1,2,2,4,0,5});
    std::cout<<lis.query(0,7)<<' '<<lis.query(1,5)<<' '<<lis.query(3,3)<<'\n';
}
