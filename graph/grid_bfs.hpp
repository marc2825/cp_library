#pragma once
#include <cassert>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace cp {
// (行, 列) で指定。上下左右、通行不可は blocked。到達不能は -1。O(HW)。
// 空グリッドは空配列、始点が壁ならすべて -1。
inline std::vector<std::vector<int>> grid_bfs(
    const std::vector<std::string>& grid, int row, int col, char blocked = '#') {
    if (grid.empty()) return {};
    int h = static_cast<int>(grid.size()), w = static_cast<int>(grid[0].size());
    for (const auto& line : grid) { assert(static_cast<int>(line.size()) == w); (void)line; }
    std::vector<std::vector<int>> dist(h, std::vector<int>(w, -1));
    if (w == 0) return dist;
    assert(0 <= row && row < h && 0 <= col && col < w);
    if (grid[row][col] == blocked) return dist;
    constexpr int dr[] = {0, 1, 0, -1}, dc[] = {1, 0, -1, 0};
    std::queue<std::pair<int, int>> queue;
    dist[row][col] = 0; queue.emplace(row, col);
    while (!queue.empty()) {
        auto [r, c] = queue.front(); queue.pop();
        for (int d = 0; d < 4; ++d) {
            int nr = r + dr[d], nc = c + dc[d];
            if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
            if (dist[nr][nc] != -1 || grid[nr][nc] == blocked) continue;
            dist[nr][nc] = dist[r][c] + 1; queue.emplace(nr, nc);
        }
    }
    return dist;
}
} // namespace cp
