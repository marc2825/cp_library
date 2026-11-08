#include "static_query/swag.hpp"
#include <iostream>
struct Affine { long long a, b; };
Affine op(Affine left, Affine right) { return {right.a * left.a, right.a * left.b + right.b}; }
Affine e() { return {1, 0}; }
long long evaluate(Affine f, long long x) { return f.a * x + f.b; }
int main() {
    cp::SWAG<Affine, op, e> queue;
    queue.push({2, 1}); queue.push({3, 4});
    std::cout << evaluate(queue.prod(), 5) << '\n'; // 37 = 3*(2*5+1)+4
    queue.pop();
    std::cout << evaluate(queue.prod(), 5) << '\n'; // 19
    queue.push({0, 7});
    std::cout << evaluate(queue.prod(), 5) << '\n'; // 7
    queue.clear();
    std::cout << evaluate(queue.prod(), 5) << '\n'; // 5 (identity)
}
