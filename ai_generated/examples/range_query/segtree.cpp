#include "range_query/segtree.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

int maximum(int a, int b) { return std::max(a, b); }
int identity() { return std::numeric_limits<int>::lowest(); }

int main() {
    cp::Segtree<int, maximum, identity> a(std::vector<int>{2, 7, 1, 5});
    a.set(1, 3);
    std::cout << a.prod(0, 3) << ' ' << a.all_prod() << '\n'; // 3 5
    std::cout << a.max_right(0, [](int x) { return x < 4; }) << '\n'; // 3
}
