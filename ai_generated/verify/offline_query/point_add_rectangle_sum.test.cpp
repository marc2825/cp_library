#define PROBLEM "https://judge.yosupo.jp/problem/point_add_rectangle_sum"
#include <iostream>
#include "offline_query/cdq_rectangle_sum.hpp"

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    int n, q;
    std::cin >> n >> q;
    cp::CDQRectangleSum<> solver;
    for (int i = 0; i < n; ++i) {
        long long x, y, w;
        std::cin >> x >> y >> w;
        solver.add_point(x, y, w);
    }
    for (int i = 0; i < q; ++i) {
        int type;
        std::cin >> type;
        if (type == 0) {
            long long x, y, w;
            std::cin >> x >> y >> w;
            solver.add_point(x, y, w);
        } else {
            long long l, d, r, u;
            std::cin >> l >> d >> r >> u;
            solver.add_query(l, d, r, u);
        }
    }
    for (auto x : solver.solve()) std::cout << x << '\n';
}
