#include "range_query/fenwick_tree.hpp"
#include "range_query/range_add_range_sum_fenwick_tree.hpp"
#include <iostream>
#include <vector>

int main() {
    cp::FenwickTree<long long> counts(std::vector<long long>{0, 2, 0, 3});
    counts.add(2, 1);
    std::cout << counts.sum(1, 4) << ' ' << counts.lower_bound(3) << '\n'; // 6 2
    cp::RangeAddRangeSumFenwickTree<long long> a(std::vector<long long>{1, 2, 3, 4});
    a.add(1, 4, 5);
    std::cout << a.sum(0, 4) << ' ' << a.get(2) << '\n'; // 25 8
}
