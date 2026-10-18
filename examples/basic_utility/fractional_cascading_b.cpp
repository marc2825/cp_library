#include <iostream>
#include "basic_utility/fractional_cascading.hpp"
int main() {
    cp::FractionalCascading<int> f({{1,4,8},{2,4,7},{}});
    std::cout << f.lower_bounds(4)[0] << " " << f.lower_bounds(4)[1] << "\n";
}
