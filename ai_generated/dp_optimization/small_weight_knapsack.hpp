#pragma once
#include <algorithm>
#include <map>
#include <stdexcept>
#include <vector>
#include "knapsack_common.hpp"

namespace cp {
// 0-1 knapsack。重さごとに価値を降順化し、凹prefixと各剰余列をmax-plus積。
// D種類の正の重さ: O(n log n + D W log(W+1))時間、O(n+W)領域。
inline KnapsackResult small_weight_knapsack(int capacity, const std::vector<KnapsackItem>& items) {
    if (capacity < 0) throw std::invalid_argument("negative knapsack capacity");
    KnapsackResult result(capacity);
    std::map<int, std::vector<long long>> groups;
    long long free_gain = 0;
    for (const auto& item : items) {
        if (item.weight < 0) throw std::invalid_argument("negative knapsack weight");
        if (!item.weight) { if (item.value > 0) free_gain = detail::dp_add(free_gain, item.value); }
        else if (item.weight <= capacity) groups[item.weight].push_back(item.value);
    }
    result.dp[0] = free_gain;
    for (auto& group : groups) {
        const int weight = group.first;
        auto& values = group.second;
        std::sort(values.begin(), values.end(), std::greater<long long>());
        const int limit = static_cast<int>(std::min(values.size(), static_cast<std::size_t>(capacity / weight)));
        std::vector<long long> prefix(static_cast<std::size_t>(limit) + 1);
        for (int i = 0; i < limit; ++i) prefix[i+1] = detail::dp_add(prefix[i], values[i]);
        const auto previous = result.dp;
        for (int residue = 0; residue < weight && residue <= capacity; ++residue) {
            const int length = (capacity-residue)/weight + 1;
            std::vector<int> columns, rows;
            for (int t = 0; t < length; ++t) if (previous[residue+t*weight]) columns.push_back(t);
            if (columns.empty()) continue;
            // 候補が存在する行だけ探索して、-infinityだけの行のargmaxを使わない。
            std::size_t first = 0;
            for (int t = 0; t < length; ++t) {
                while (first < columns.size() && columns[first] < t-limit) ++first;
                if (first < columns.size() && columns[first] <= t) rows.push_back(t);
            }
            auto solve = [&](auto&& self, int top, int bottom, int left, int right) -> void {
                if (top >= bottom) return;
                const int mid = top + (bottom-top)/2, t = rows[mid];
                const int lo = static_cast<int>(std::lower_bound(columns.begin()+left, columns.begin()+right, t-limit) - columns.begin());
                const int hi = static_cast<int>(std::upper_bound(columns.begin()+lo, columns.begin()+right, t) - columns.begin());
                int best = lo;
                auto score = [&](int c) { return detail::dp_add(*previous[residue+columns[c]*weight], prefix[t-columns[c]]); };
                auto value = score(best);
                for (int c = lo+1; c < hi; ++c) { const auto x = score(c); if (value < x) { value = x; best = c; } }
                result.dp[residue+t*weight] = value;
                self(self, top, mid, left, best+1); self(self, mid+1, bottom, best, right);
            };
            solve(solve, 0, static_cast<int>(rows.size()), 0, static_cast<int>(columns.size()));
        }
    }
    return result;
}
} // namespace cp
