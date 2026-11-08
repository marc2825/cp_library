#define PROBLEM "https://judge.yosupo.jp/problem/deque_operate_all_composite"
#include "static_query/swag_deque.hpp"
#include "test_values.hpp"
#include <iostream>
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int q; std::cin >> q;
    cp::SWAGDeque<static_test::Affine, static_test::concatenate_affine, static_test::identity> deque;
    while (q--) {
        int t; std::cin >> t;
        if (t <= 1) {
            static_test::Affine f; std::cin >> f.a >> f.b;
            if (t == 0) deque.push_front(f); else deque.push_back(f);
        } else if (t == 2) deque.pop_front();
        else if (t == 3) deque.pop_back();
        else { long long x; std::cin >> x; std::cout << static_test::evaluate(deque.prod(), x) << '\n'; }
    }
}
