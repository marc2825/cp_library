#pragma once
#include <cassert>
#include <limits>
#include <stdexcept>
#include <vector>
#include "detail/arithmetic.hpp"
#include "../static_query/monotonic_stack.hpp"

namespace cp {
struct HistogramRectangle { long long area = 0; int left = 0, right = 0; long long height = 0; };
struct GridRectangle { long long area = 0; int top = 0, left = 0, bottom = 0, right = 0; };
// 高さは非負。区間は半開。詳細: docs/dp_optimization/largest_rectangle.md。
inline HistogramRectangle largest_rectangle_histogram(const std::vector<long long>& height) {
    for (auto h : height) { assert(h >= 0); (void)h; }
    const auto left = previous_better_indices(height), right = next_better_indices(height);
    HistogramRectangle result; const int n = static_cast<int>(height.size());
    for (int i = 0; i < n; ++i) {
        const int l = left[i] + 1, r = right[i] == -1 ? n : right[i];
        const auto area = detail::dp_multiply(height[i], r - l);
        if (area > result.area) result = {area, l, r, height[i]};
    }
    return result;
}
// blocked=true が障害物。false だけの最大長方形。空・0列も可。
inline GridRectangle largest_rectangle_grid(const std::vector<std::vector<bool>>& blocked) {
    GridRectangle result;
    if (blocked.empty()) return result;
    assert(blocked.size() <= static_cast<std::size_t>(std::numeric_limits<int>::max()));
    const auto width = blocked[0].size(); std::vector<long long> height(width);
    for (int row = 0; row < static_cast<int>(blocked.size()); ++row) {
        if (blocked[row].size() != width) throw std::invalid_argument("rectangle grid must be rectangular");
        for (std::size_t j = 0; j < width; ++j) height[j] = blocked[row][j] ? 0 : height[j] + 1;
        const auto best = largest_rectangle_histogram(height);
        if (best.area > result.area) result = {best.area, row + 1 - static_cast<int>(best.height), best.left, row + 1, best.right};
    }
    return result;
}
} // namespace cp
