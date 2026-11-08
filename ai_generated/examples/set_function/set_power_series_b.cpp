#include <iostream>
#include <vector>
#include "../../set_function/set_power_series.hpp"

int main() {
    auto a = cp::sps_exp(std::vector<int>{0, 2, 3, 5});
    std::cout << a[0] << " " << a[3] << "\n";
}
