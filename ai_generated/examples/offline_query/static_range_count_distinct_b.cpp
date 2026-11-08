#include <iostream>
#include "offline_query/static_range_count_distinct.hpp"
int main() {
    auto a=cp::static_range_count_distinct<int>({1,2,1,3},{{0,4},{1,3},{2,2}});
    std::cout << a[0] << " " << a[1] << " " << a[2] << "\n";
}
