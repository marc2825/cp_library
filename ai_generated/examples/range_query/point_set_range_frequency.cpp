#include <iostream>
#include "range_query/point_set_range_frequency.hpp"
int main() {
    cp::PointSetRangeFrequency<int> a({2,1,2,3});
    std::cout << a.frequency(0,4,2) << "\n";
    a.set(1,2); a.set(3,100);
    std::cout << a.frequency(0,3,2) << " " << a.get(3) << "\n";
}
