#pragma once
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include "../basic_utility/grid.hpp"

namespace cp {
// (行, 列) で指定。上下左右、通行不可は blocked。到達不能は -1。O(H+HW)。
// 空グリッドは空配列、始点が壁ならすべて -1。
inline std::vector<std::vector<int>> grid_bfs(
    const std::vector<std::string>& grid, int row, int col, char blocked = '#') {
    const auto shape = grid_shape(grid);
    int h = shape.rows(), w = shape.cols();
    if (std::int64_t(h) * w > std::numeric_limits<int>::max())
        throw std::length_error("grid BFS cell count exceeds int distance");
    std::vector<std::vector<int>> dist(h, std::vector<int>(w, -1));
    if (w == 0) return dist;
    if (!shape.contains(row,col)) throw std::out_of_range("grid BFS source out of range");
    if (grid[row][col] == blocked) return dist;
    std::queue<std::pair<int, int>> queue;
    dist[row][col] = 0; queue.emplace(row, col);
    while (!queue.empty()) {
        auto [r, c] = queue.front(); queue.pop();
        for (auto [dr,dc] : grid_directions4) {
            int nr = r + dr, nc = c + dc;
            if (!shape.contains(nr,nc)) continue;
            if (dist[nr][nc] != -1 || grid[nr][nc] == blocked) continue;
            dist[nr][nc] = dist[r][c] + 1; queue.emplace(nr, nc);
        }
    }
    return dist;
}
} // namespace cp
