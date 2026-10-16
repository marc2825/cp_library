#include <iostream>
#include <vector>
#include "offline_query/mo.hpp"
#include "ordered_structure/coordinate_compression.hpp"

int main() {
    std::vector<int> a{4, 1, 4, 2, 1};
    cp::CoordinateCompression<int> values(a);
    for (auto& x : a) x = *values.index(x);
    cp::Mo mo(int(a.size()));
    mo.add_query(0, 3);
    mo.add_query(1, 5);
    mo.add_query(2, 2);
    std::vector<int> count(values.size()), answer(mo.query_count());
    int distinct = 0;
    mo.run([&](int i) { if (count[a[i]]++ == 0) ++distinct; },
           [&](int i) { if (--count[a[i]] == 0) --distinct; },
           [&](int id) { answer[id] = distinct; });
    for (int x : answer) std::cout << x << '\n'; // 2, 3, 0
}
