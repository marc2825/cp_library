#pragma once

// 旧名だけを残す。include マクロや dx1/dy1 のグローバル定義は不要。
#include "../graph/grid_bfs.hpp"

inline std::vector<std::vector<int>> Grid_BFS(
    const std::vector<std::string>& grid, int row, int col, char blocked = '#') {
    return cp::grid_bfs(grid, row, col, blocked);
}
