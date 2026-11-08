#pragma once
#include <bitset>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>
#include "gauss_jordan.hpp"

namespace cp {
template <std::size_t Bits> struct GF2LinearSystemResult {
    LinearSystemStatus status = LinearSystemStatus::no_solution;
    std::size_t rank = 0;
    std::vector<std::size_t> pivot_columns;
    std::vector<std::bitset<Bits + 1>> reduced; // RHSは指定された変数数の位置。
    std::bitset<Bits> particular;
    std::vector<std::bitset<Bits>> nullspace_basis;
    bool has_solution() const { return status != LinearSystemStatus::no_solution; }
};
namespace linear_algebra_detail {
template <std::size_t Bits> GF2LinearSystemResult<Bits> gf2_reduce(const std::vector<std::bitset<Bits>>& a, const std::vector<int>& b, std::size_t variables) {
    if (variables > Bits || a.size() != b.size()) throw std::invalid_argument("GF(2) dimensions differ");
    GF2LinearSystemResult<Bits> result; result.reduced.resize(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (b[i] != 0 && b[i] != 1) throw std::invalid_argument("GF(2) RHS must be zero or one");
        for (std::size_t j = 0; j < Bits; ++j) {
            if (j >= variables && a[i][j]) throw std::invalid_argument("GF(2) unused high bits must be zero");
            result.reduced[i][j] = a[i][j];
        }
        result.reduced[i][variables] = b[i] != 0;
    }
    for (std::size_t col = 0; col < variables && result.rank < a.size(); ++col) {
        auto pivot = result.rank;
        while (pivot < a.size() && !result.reduced[pivot][col]) ++pivot;
        if (pivot == a.size()) continue;
        std::swap(result.reduced[result.rank], result.reduced[pivot]);
        for (std::size_t row = 0; row < a.size(); ++row)
            if (row != result.rank && result.reduced[row][col]) result.reduced[row] ^= result.reduced[result.rank];
        result.pivot_columns.push_back(col); ++result.rank;
    }
    return result;
}
} // namespace linear_algebra_detail
// bit jは変数jの係数。未使用high bitは0。O(m*r*ceil((Bits+1)/word))消去。
// 特解と全零空間を返す。docs/linear_algebra/gf2_gauss.md
template <std::size_t Bits> GF2LinearSystemResult<Bits> gf2_gauss_jordan(const std::vector<std::bitset<Bits>>& a, const std::vector<int>& b, std::size_t variables = Bits) {
    auto result = linear_algebra_detail::gf2_reduce(a, b, variables);
    for (std::size_t row = result.rank; row < a.size(); ++row) if (result.reduced[row][variables]) return result;
    result.status = result.rank == variables ? LinearSystemStatus::unique : LinearSystemStatus::multiple;
    for (std::size_t i = 0; i < result.rank; ++i) result.particular[result.pivot_columns[i]] = result.reduced[i][variables];
    for (std::size_t col = 0, p = 0; col < variables; ++col) {
        if (p < result.rank && result.pivot_columns[p] == col) { ++p; continue; }
        std::bitset<Bits> v; v[col] = true;
        for (std::size_t i = 0; i < result.rank; ++i) v[result.pivot_columns[i]] = result.reduced[i][col];
        result.nullspace_basis.push_back(v);
    }
    return result;
}
template <std::size_t Bits> std::size_t gf2_rank(const std::vector<std::bitset<Bits>>& a, std::size_t variables = Bits) {
    return linear_algebra_detail::gf2_reduce(a, std::vector<int>(a.size()), variables).rank;
}
} // namespace cp
