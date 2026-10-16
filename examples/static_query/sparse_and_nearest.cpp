#include "static_query/sparse_table.hpp"
#include "static_query/monotonic_stack.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>
int minimum(int x, int y) { return std::min(x, y); }
int infinity() { return std::numeric_limits<int>::max(); }
int main() {
    std::vector<int> a{3, 1, 1, 2};
    cp::SparseTable<int, minimum, infinity> table(a);
    std::cout << table.prod(0, 4) << ' ' << table.prod(3, 4) << '\n'; // 1 2
    for (int i : cp::previous_better_indices(a)) std::cout << i << ' '; // -1 -1 -1 2
    std::cout << '\n';
    for (int i : cp::next_better_indices(a, false)) std::cout << i << ' '; // 1 2 -1 -1
    std::cout << '\n';
}
