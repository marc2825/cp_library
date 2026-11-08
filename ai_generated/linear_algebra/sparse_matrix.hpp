#pragma once
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "detail/arithmetic.hpp"
#include "../linear_recurrence/detail/common.hpp"

namespace cp {
// CSR。重複座標は別項として保持し、積で加算。零も保持する。
template <class T> class SparseMatrix {
    static_assert(!std::is_same_v<T, bool>, "bool sparse matrices are unsupported");
public:
    struct Entry { std::size_t row, column; T value; };
private:
    std::size_t rows_, cols_;
    std::vector<std::size_t> offsets_, columns_;
    std::vector<T> values_;
public:
    SparseMatrix(std::size_t rows, std::size_t cols, const std::vector<Entry>& entries)
        : rows_(rows), cols_(cols) {
        if (rows >= std::vector<std::size_t>().max_size()) throw std::length_error("sparse row count too large");
        offsets_.resize(rows + 1); columns_.resize(entries.size()); values_.resize(entries.size());
        for (const auto& e : entries) {
            if (e.row >= rows || e.column >= cols) throw std::out_of_range("sparse entry index");
            linear_algebra_detail::input(e.value); ++offsets_[e.row + 1];
        }
        for (std::size_t i = 0; i < rows; ++i) offsets_[i + 1] += offsets_[i];
        auto next = offsets_;
        for (const auto& e : entries) { const auto index = next[e.row]++; columns_[index] = e.column; values_[index] = e.value; }
    }
    std::size_t rows() const { return rows_; }
    std::size_t cols() const { return cols_; }
    std::size_t nonzeros() const { return values_.size(); } // 格納項数（零を含む）
    const std::vector<std::size_t>& row_offsets() const { return offsets_; }
    const std::vector<std::size_t>& column_indices() const { return columns_; }
    const std::vector<T>& values() const { return values_; }
    std::vector<T> multiply(const std::vector<T>& x) const {
        if (x.size() != cols_) throw std::invalid_argument("sparse vector dimension mismatch");
        for (const auto& value : x) linear_algebra_detail::input(value);
        std::vector<T> result(rows_, T(0));
        for (std::size_t i = 0; i < rows_; ++i) for (auto k = offsets_[i]; k < offsets_[i + 1]; ++k)
            linear_algebra_detail::add(result[i], linear_algebra_detail::multiply(values_[k], x[columns_[k]]));
        return result;
    }
};
template <int Mod = 998244353> std::vector<int> sparse_matrix_vector_multiply_mod(const SparseMatrix<int>& a, const std::vector<int>& x) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    if (x.size() != a.cols()) throw std::invalid_argument("sparse vector dimension mismatch");
    const auto input = linear_recurrence_detail::normalize<Mod>(x);
    std::vector<int> result(a.rows());
    for (std::size_t i = 0; i < a.rows(); ++i) for (auto k = a.row_offsets()[i]; k < a.row_offsets()[i + 1]; ++k)
        result[i] = F::add(result[i], F::mul(F::norm(a.values()[k]), input[a.column_indices()[k]]));
    return result;
}
} // namespace cp
