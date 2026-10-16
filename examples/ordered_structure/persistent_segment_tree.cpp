#include <iostream>
#include "ordered_structure/persistent_segment_tree.hpp"
long long op(long long a, long long b) { return a + b; }
long long e() { return 0; }
int main() {
    cp::PersistentSegmentTree<long long, op, e> s(4);
    int a = s.set(0, 1, 5), b = s.set(a, 2, 7), c = s.set(a, 1, 9);
    std::cout << s.prod(a, 0, 4) << ' ' << s.prod(b, 0, 4) << ' ' << s.prod(c, 0, 4) << '\n';
}
