#define PROBLEM "https://judge.yosupo.jp/problem/queue_operate_all_composite"
#include "static_query/swag.hpp"
#include "test_values.hpp"
#include <iostream>
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int q; std::cin >> q;
    cp::SWAG<static_test::Affine, static_test::concatenate_affine, static_test::identity> queue;
    while (q--) {
        int t; std::cin >> t;
        if (t == 0) { static_test::Affine f; std::cin >> f.a >> f.b; queue.push(f); }
        else if (t == 1) queue.pop();
        else { long long x; std::cin >> x; std::cout << static_test::evaluate(queue.prod(), x) << '\n'; }
    }
}
