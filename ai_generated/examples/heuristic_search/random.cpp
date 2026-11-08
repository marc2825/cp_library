#include <algorithm>
#include <iostream>
#include <vector>
#include "heuristic_search/random.hpp"
int main() {
    cp::Random random(0);
    std::cout << random() << '\n' << random() << '\n';
    std::cout << random.integer(-7, -7) << '\n';
    std::vector<int> order{0, 1, 2, 3};
    random.shuffle(order.begin(), order.end());
    std::sort(order.begin(), order.end());
    std::cout << (order == std::vector<int>{0, 1, 2, 3}) << '\n';
}
