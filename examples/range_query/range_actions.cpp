#include "range_query/lazy_segtree.hpp"
#include "range_query/dual_segtree.hpp"
#include <iostream>
#include <vector>

struct Node { long long sum; int length; };
struct Action { long long a, b; }; // x -> a*x+b; add={1,b}, assign={0,b}
Node op(Node x, Node y) { return {x.sum + y.sum, x.length + y.length}; }
Node e() { return {0, 0}; }
Node mapping(Action f, Node x) { return {f.a * x.sum + f.b * x.length, x.length}; }
long long point_mapping(Action f, long long x) { return f.a * x + f.b; }
Action composition(Action f, Action g) { return {f.a * g.a, f.a * g.b + f.b}; }
Action id() { return {1, 0}; }

int main() {
    cp::LazySegtree<Node, op, e, Action, mapping, composition, id> lazy(
        std::vector<Node>{{1, 1}, {2, 1}, {3, 1}, {4, 1}});
    cp::DualSegtree<long long, Action, point_mapping, composition, id> dual(
        std::vector<long long>{1, 2, 3, 4});
    lazy.apply(0, 4, {1, 5}); dual.apply(0, 4, {1, 5}); // add 5
    lazy.apply(1, 3, {0, 2}); dual.apply(1, 3, {0, 2}); // assign 2 after addition
    std::cout << lazy.prod(0, 4).sum << ' ' << dual.get(2) << '\n'; // 19 2
}
