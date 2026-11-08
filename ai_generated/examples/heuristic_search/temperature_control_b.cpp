#include <iostream>
#include <vector>
#include "../../heuristic_search/temperature_control.hpp"

int main() {
    auto temperature = cp::calibrate_annealing_temperature({2, 2, 2}, 0.5L);
    std::cout << (temperature > 2 && temperature < 3) << "\n";
}
