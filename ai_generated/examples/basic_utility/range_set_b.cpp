#include <iostream>
#include "basic_utility/range_set.hpp"
int main() {
    cp::RangeSet<int> s; s.insert(1,5); s.erase(2,4);
    std::cout << s.interval_count() << "\n";
}
