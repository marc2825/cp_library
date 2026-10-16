#include <iostream>
#include "linear_recurrence/bostan_mori.hpp"
int main() {
    std::cout << cp::bostan_mori({0, 1}, {1, -1, -1}, 10) << '\n';
    std::cout << cp::linear_recurrence_nth({0, 1}, {1, 1}, 10) << '\n';
}
