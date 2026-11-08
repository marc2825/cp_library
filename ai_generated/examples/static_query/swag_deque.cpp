#include <iostream>
#include "static_query/swag_deque.hpp"
struct F { long long a, b; };
F op(F left, F right) { return {right.a * left.a, right.a * left.b + right.b}; }
F e() { return {1, 0}; }
int main() {
    cp::SWAGDeque<F, op, e> deque;
    deque.push_back({3, 4}); deque.push_front({2, 1});
    auto f = deque.prod();
    std::cout << f.a * 5 + f.b << '\n'; // (2*x+1), then (3*x+4).
    deque.pop_back();
    f = deque.prod(); std::cout << f.a * 5 + f.b << '\n';
    deque.push_front({0, 7}); // Assignment needs no inverse.
    f = deque.prod(); std::cout << f.a * 5 + f.b << '\n';
    deque.pop_front(); deque.pop_front();
    f = deque.prod(); std::cout << f.a * 5 + f.b << '\n';
}
