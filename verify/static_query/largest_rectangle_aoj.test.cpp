#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_3_C"
#include "static_query/monotonic_stack.hpp"
#include <algorithm>
#include <iostream>
#include <vector>
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::cin >> n;
    std::vector<long long> h(n); for (auto& x : h) std::cin >> x;
    auto left = cp::previous_better_indices(h), right = cp::next_better_indices(h);
    long long answer = 0;
    for (int i = 0; i < n; ++i) {
        int r = right[i] == -1 ? n : right[i];
        answer = std::max(answer, h[i] * (r - left[i] - 1));
    }
    std::cout << answer << '\n';
}
