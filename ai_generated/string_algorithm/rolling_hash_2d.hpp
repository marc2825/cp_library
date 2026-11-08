#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include "rolling_hash.hpp"

namespace cp {
// byte+1の二変数多項式ハッシュ、二つの素数法。衝突あり。
// docs/string_algorithm/rolling_hash_2d.md
class RollingHash2D {
public:
    using Hash = RollingHash::Hash;
    inline static constexpr auto moduli = RollingHash::moduli;
    inline static constexpr std::uint32_t default_row_base = RollingHash::default_base;
    inline static constexpr std::uint32_t default_column_base = 972663749;
private:
    int height_ = 0, width_ = 0;
    std::uint32_t row_base_, column_base_;
    std::size_t stride_ = 1;
    std::vector<Hash> prefix_, row_power_, column_power_;
    const Hash& cell(int row, int col) const { return prefix_[static_cast<std::size_t>(row) * stride_ + col]; }
public:
    explicit RollingHash2D(const std::vector<std::string>& grid = {}, std::uint32_t row_base = default_row_base,
                          std::uint32_t column_base = default_column_base) : row_base_(row_base), column_base_(column_base) {
        height_ = string_algorithm_detail::checked_size(grid.size());
        width_ = grid.empty() ? 0 : string_algorithm_detail::checked_size(grid.front().size());
        if (row_base < 257 || row_base >= moduli[0] || column_base < 257 || column_base >= moduli[0])
            throw std::invalid_argument("2D hash bases must be in [257,1000000007)");
        for (const auto& row : grid) if (row.size() != static_cast<std::size_t>(width_)) throw std::invalid_argument("2D hash rows must have equal length");
        stride_ = static_cast<std::size_t>(width_) + 1;
        const auto rows = static_cast<std::size_t>(height_) + 1;
        if (rows > std::vector<Hash>().max_size() / stride_) throw std::length_error("2D hash prefix too large");
        prefix_.resize(rows * stride_); row_power_.resize(rows); column_power_.resize(stride_);
        row_power_[0] = column_power_[0] = {1, 1};
        for (int i = 0; i < height_; ++i) for (std::size_t k = 0; k < 2; ++k) row_power_[i + 1][k] = std::uint64_t(row_power_[i][k]) * row_base_ % moduli[k];
        for (int j = 0; j < width_; ++j) for (std::size_t k = 0; k < 2; ++k) column_power_[j + 1][k] = std::uint64_t(column_power_[j][k]) * column_base_ % moduli[k];
        for (int i = 0; i < height_; ++i) for (int j = 0; j < width_; ++j) for (std::size_t k = 0; k < 2; ++k) {
            const auto p = moduli[k];
            const auto up = std::uint64_t(cell(i, j + 1)[k]) * row_base_ % p;
            const auto left = std::uint64_t(cell(i + 1, j)[k]) * column_base_ % p;
            const auto both = std::uint64_t(cell(i, j)[k]) * row_base_ % p * column_base_ % p;
            prefix_[static_cast<std::size_t>(i + 1) * stride_ + j + 1][k] = (up + left + p - both + static_cast<unsigned char>(grid[i][j]) + 1) % p;
        }
    }
    int height() const { return height_; }
    int width() const { return width_; }
    std::uint32_t row_base() const { return row_base_; }
    std::uint32_t column_base() const { return column_base_; }
    Hash get(int top, int bottom, int left, int right) const {
        string_algorithm_detail::range(height_, top, bottom); string_algorithm_detail::range(width_, left, right);
        if (top == bottom || left == right) return {0, 0};
        Hash result{};
        for (std::size_t k = 0; k < 2; ++k) {
            const auto p = moduli[k]; const auto row = row_power_[bottom - top][k], col = column_power_[right - left][k];
            const auto up = std::uint64_t(cell(top, right)[k]) * row % p;
            const auto before = std::uint64_t(cell(bottom, left)[k]) * col % p;
            const auto both = std::uint64_t(cell(top, left)[k]) * row % p * col % p;
            result[k] = (cell(bottom, right)[k] + std::uint64_t(2) * p - up - before + both) % p;
        }
        return result;
    }
    bool equal(const RollingHash2D& other, int top, int bottom, int left, int right,
               int other_top, int other_bottom, int other_left, int other_right) const {
        const auto a = get(top, bottom, left, right), b = other.get(other_top, other_bottom, other_left, other_right);
        if (row_base_ != other.row_base_ || column_base_ != other.column_base_) throw std::invalid_argument("2D hash bases differ");
        return bottom - top == other_bottom - other_top && right - left == other_right - other_left && a == b;
    }
};
} // namespace cp
