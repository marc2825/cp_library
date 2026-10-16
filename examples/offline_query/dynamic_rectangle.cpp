#include <iostream>
#include "offline_query/cdq_rectangle_sum.hpp"

int main() {
    cp::CDQRectangleSum<> queries;
    queries.add_query(0, 0, 4, 4);   // id 0: no points yet
    queries.add_point(1, 2, 4);
    queries.add_query(0, 0, 4, 4);   // id 1: 4
    queries.add_point(1, 2, -1);     // signed delta at the same coordinates
    queries.add_point(3, 2, 7);
    queries.add_query(1, 2, 3, 3);   // id 2: right boundary is excluded
    for (auto x : queries.solve()) std::cout << x << '\n'; // 0, 4, 3
}
