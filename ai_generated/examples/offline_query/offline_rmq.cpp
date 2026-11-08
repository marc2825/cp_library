#include <iostream>
#include "offline_query/offline_rmq.hpp"
int main() {
    auto a=cp::offline_rmq(std::vector<int>{4,2,2,7},{{0,4},{2,4},{1,1}});
    for(int x:a) { std::cout<<x<<" "; } std::cout<<"\n";
}
