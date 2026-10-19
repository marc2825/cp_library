#include <iostream>
#include <utility>
#include "basic_utility/rollback_array.hpp"
int main() {
    cp::RollbackArray<std::pair<int, int>> state(2, {10, 0});
    auto before = state.snapshot();
    state.modify(0, [](auto& item) { item.first += 7; item.second = 1; });
    std::cout << state[0].first << " " << state[0].second << "\n";
    state.rollback(before);
    std::cout << state[0].first << " " << state[0].second << "\n";
}
