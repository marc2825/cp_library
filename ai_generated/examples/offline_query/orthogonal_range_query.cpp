#include <iostream>
#include "../../offline_query/orthogonal_range_query.hpp"
int main(){
    using Query=cp::OrthogonalRangeQuery<long long,3>;
    Query query({{0,0,0},{1,2,3},{1,2,3}}, {4,5,6});
    std::cout<<query.count({0,0,0},{2,3,4})<<' '<<query.sum({1,2,3},{2,3,4})<<'\n';
}
