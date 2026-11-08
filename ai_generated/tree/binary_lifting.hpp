#pragma once

#include <cassert>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace cp {

// 一意な遷移を k 回適用。閉路可、-1 は遷移先なし。詳細: docs/tree/binary_lifting.md。
class BinaryLifting {
public:
    std::vector<std::vector<int>> up; // up[b][v] = 2^b 回先。存在しなければ -1。

    explicit BinaryLifting(const std::vector<int>& next,
                           std::uint64_t max_steps = std::numeric_limits<std::uint64_t>::max())
        : max_steps_(max_steps) {
        const int n = static_cast<int>(next.size());
        for (int v : next) { assert(-1 <= v && v < n); (void)v; }
        int levels = 1;
        for (auto k = max_steps; k > 1; k >>= 1) ++levels;
        up.assign(levels, std::vector<int>(n, -1));
        up[0] = next;
        for (int b = 1; b < levels; ++b)
            for (int v = 0; v < n; ++v)
                if (up[b - 1][v] != -1) up[b][v] = up[b - 1][up[b - 1][v]];
    }

    int size() const { return static_cast<int>(up[0].size()); }
    std::uint64_t max_steps() const { return max_steps_; }
    // k=0 は v 自身。前計算上限の超過は out_of_range（release でも確認）。
    int jump(int v, std::uint64_t k) const {
        assert(0 <= v && v < size());
        if (k > max_steps_) throw std::out_of_range("BinaryLifting step limit exceeded");
        for (int b = 0; k && v != -1; ++b, k >>= 1)
            if (k & 1) v = up[b][v];
        return v;
    }

private:
    std::uint64_t max_steps_;
};

} // namespace cp
