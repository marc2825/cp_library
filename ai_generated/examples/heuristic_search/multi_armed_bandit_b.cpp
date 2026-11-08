#include <iostream>
#include <vector>
#include "../../heuristic_search/multi_armed_bandit.hpp"

int main() {
    cp::MultiArmedBandit bandit(2);
    std::size_t first = bandit.select_ucb1(); bandit.update(first, 1);
    std::cout << first << " " << bandit.select_ucb1() << "\n";
}
