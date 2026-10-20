#include <iostream>
#include "basic_utility/two_pointers.hpp"
int main() {
    std::vector<int> a{2, 1, 3, 1, 2};
    int sum = 0;
    auto windows = cp::two_pointers(a.size(),
        [&](std::size_t i) { return sum + a[i] <= 4; },
        [&](std::size_t i) { sum += a[i]; },
        [&](std::size_t i) { sum -= a[i]; });
    auto [l, r] = windows.longest();
    std::cout << l << ' ' << r << ' ' << windows.count() << '\n';
}
