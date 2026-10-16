#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DSL_3_D"
#include "static_query/monotonic_queue.hpp"
#include <iostream>
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, k; std::cin >> n >> k;
    cp::MonotonicQueue<int> queue;
    for (int i = 0; i < n; ++i) {
        int x; std::cin >> x;
        if (queue.size() == k) queue.pop();
        queue.push(x);
        if (queue.size() == k) std::cout << (i == k - 1 ? "" : " ") << queue.best();
    }
    std::cout << '\n';
}
