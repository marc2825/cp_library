#include <iostream>
#include "static_query/priority_sum.hpp"
int main() {
    cp::PrioritySum<long long> s(2); s.insert(8); s.insert(1); s.insert(3);
    std::cout << s.sum() << "\n";
}
