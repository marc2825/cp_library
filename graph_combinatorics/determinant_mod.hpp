#pragma once
#include <cassert>
#include <cstdint>
#include <utility>
#include <vector>

namespace cp {
namespace graph_combinatorics_detail {
constexpr bool is_prime_mod(int p) {
    if (p < 2) return false;
    for (int d = 2; d <= p / d; ++d) if (p % d == 0) return false;
    return true;
}
template<int Mod> long long power_mod(long long value, int exponent) {
    long long result = 1;
    for (; exponent; exponent >>= 1, value = value * value % Mod)
        if (exponent & 1) result = result * value % Mod;
    return result;
}
} // namespace graph_combinatorics_detail

// Prime Mod<=INT_MAX; empty determinant=1. Copies and normalizes its matrix.
template<int Mod = 998244353>
int determinant_mod(std::vector<std::vector<long long>> matrix) {
    static_assert(graph_combinatorics_detail::is_prime_mod(Mod), "Mod must be a prime int");
    const int n = static_cast<int>(matrix.size());
    for (auto& row : matrix) {
        assert(static_cast<int>(row.size()) == n);
        for (auto& value : row) { value %= Mod; if (value < 0) value += Mod; }
    }
    long long result = 1;
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        while (pivot < n && matrix[pivot][col] == 0) ++pivot;
        if (pivot == n) return 0;
        if (pivot != col) { std::swap(matrix[pivot], matrix[col]); result = (Mod - result) % Mod; }
        result = result * matrix[col][col] % Mod;
        const long long inverse = graph_combinatorics_detail::power_mod<Mod>(matrix[col][col], Mod - 2);
        for (int row = col + 1; row < n; ++row) {
            const long long factor = matrix[row][col] * inverse % Mod;
            for (int j = col; j < n; ++j) {
                matrix[row][j] = (matrix[row][j] - factor * matrix[col][j]) % Mod;
                if (matrix[row][j] < 0) matrix[row][j] += Mod;
            }
        }
    }
    return static_cast<int>(result);
}
} // namespace cp
