#include <iostream>
#include "ordered_structure/implicit_treap.hpp"
long long op(long long a, long long b) { return a + b; }
long long e() { return 0; }
int main() {
    cp::ImplicitTreap<long long, op, e> t(std::vector<long long>{1, 2, 3});
    t.reverse(0, 3); t.insert(1, 9); t.erase(2); t.move(0, 1, 2);
    for (auto x : t.to_vector()) std::cout << x << ' ';
    std::cout << t.prod(0, 3) << '\n';
}
