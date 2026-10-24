#include <iostream>
#include "static_query/range_majority.hpp"
int main() {
    cp::RangeMajority<int> a({2,1,2,2,3,1});
    std::cout << a.majority(0,4).value_or(-1) << " " << a.majority(0,6).has_value() << "\n";
    std::cout << a.frequency(1,5,2) << "\n";
}
