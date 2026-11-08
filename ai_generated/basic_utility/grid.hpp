#pragma once
#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cp {
inline constexpr std::array<std::pair<int,int>,4> grid_directions4{{{0,1},{1,0},{0,-1},{-1,0}}};
inline constexpr std::array<std::pair<int,int>,8> grid_directions8{{{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}}};
class GridShape {
    int rows_, cols_;
    template <std::size_t N> std::vector<std::pair<int,int>> neighbors(
        int r, int c, const std::array<std::pair<int,int>,N>& directions) const {
        if (!contains(r,c)) throw std::out_of_range("grid coordinate out of range");
        std::vector<std::pair<int,int>> result;
        for (auto [dr,dc] : directions) if (contains(r+dr,c+dc)) result.emplace_back(r+dr,c+dc);
        return result;
    }
public:
    GridShape(int rows, int cols) : rows_(rows), cols_(cols) {
        if (rows < 0 || cols < 0) throw std::invalid_argument("negative grid dimensions");
    }
    int rows() const { return rows_; }
    int cols() const { return cols_; }
    bool contains(int r, int c) const { return 0 <= r && r < rows_ && 0 <= c && c < cols_; }
    auto neighbors4(int r, int c) const { return neighbors(r,c,grid_directions4); }
    auto neighbors8(int r, int c) const { return neighbors(r,c,grid_directions8); }
};
template <class Row> GridShape grid_shape(const std::vector<Row>& grid) {
    const auto h = grid.size(), w = h ? grid.front().size() : 0;
    if (h > std::size_t(std::numeric_limits<int>::max()) || w > std::size_t(std::numeric_limits<int>::max()))
        throw std::length_error("grid dimensions exceed int");
    for (const auto& row : grid) if (row.size() != w) throw std::invalid_argument("ragged grid");
    return GridShape(static_cast<int>(h), static_cast<int>(w));
}
template <class Row> std::vector<Row> grid_transpose(const std::vector<Row>& grid) {
    const auto shape = grid_shape(grid);
    std::vector<Row> result(shape.cols(), Row(shape.rows(), typename Row::value_type{}));
    for (int r=0;r<shape.rows();++r) for (int c=0;c<shape.cols();++c) result[c][r]=grid[r][c];
    return result;
}
template <class Row> std::vector<Row> grid_rotate_clockwise(std::vector<Row> grid, int turns = 1) {
    grid_shape(grid);
    turns = (turns % 4 + 4) % 4;
    while (turns--) {
        grid = grid_transpose(grid);
        for (auto& row : grid) std::reverse(row.begin(),row.end());
    }
    return grid;
}
} // namespace cp
