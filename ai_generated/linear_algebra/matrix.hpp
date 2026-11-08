#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace cp {
// 行優先の長方形行列。0×m / n×0も形を保持。要素アクセスは(r,c)、0始まり。
template <class T> class Matrix {
    static_assert(!std::is_same_v<T, bool>, "use a field type or integers instead of bool");
    std::size_t rows_ = 0, cols_ = 0;
    std::vector<T> values_;
    static std::size_t count(std::size_t rows, std::size_t cols) {
        if (cols && rows > std::vector<T>().max_size() / cols) throw std::length_error("matrix size exceeds vector capacity");
        return rows * cols;
    }
public:
    Matrix() = default;
    Matrix(const Matrix&) = default;
    Matrix& operator=(const Matrix&) = default;
    Matrix(Matrix&& other) noexcept : rows_(std::exchange(other.rows_, 0)), cols_(std::exchange(other.cols_, 0)), values_(std::move(other.values_)) {}
    Matrix& operator=(Matrix&& other) noexcept {
        if (this != &other) { rows_ = std::exchange(other.rows_, 0); cols_ = std::exchange(other.cols_, 0); values_ = std::move(other.values_); }
        return *this;
    }
    Matrix(std::size_t rows, std::size_t cols, const T& value = T(0))
        : rows_(rows), cols_(cols), values_(count(rows, cols), value) {}
    explicit Matrix(const std::vector<std::vector<T>>& values) : Matrix(values.size(), values.empty() ? 0 : values.front().size()) {
        for (std::size_t i = 0; i < rows_; ++i) {
            if (values[i].size() != cols_) throw std::invalid_argument("matrix rows must have equal lengths");
            std::copy(values[i].begin(), values[i].end(), values_.begin() + i * cols_);
        }
    }
    Matrix(std::initializer_list<std::initializer_list<T>> values) : Matrix(values.size(), values.size() ? values.begin()->size() : 0) {
        std::size_t row = 0;
        for (const auto& line : values) {
            if (line.size() != cols_) throw std::invalid_argument("matrix rows must have equal lengths");
            std::copy(line.begin(), line.end(), values_.begin() + row++ * cols_);
        }
    }
    std::size_t rows() const { return rows_; }
    std::size_t cols() const { return cols_; }
    T& operator()(std::size_t row, std::size_t col) { assert(row < rows_ && col < cols_); return values_[row * cols_ + col]; }
    const T& operator()(std::size_t row, std::size_t col) const { assert(row < rows_ && col < cols_); return values_[row * cols_ + col]; }
    T& at(std::size_t row, std::size_t col) {
        if (row >= rows_ || col >= cols_) throw std::out_of_range("matrix index out of range");
        return values_[row * cols_ + col];
    }
    const T& at(std::size_t row, std::size_t col) const {
        if (row >= rows_ || col >= cols_) throw std::out_of_range("matrix index out of range");
        return values_[row * cols_ + col];
    }
    const std::vector<T>& data() const { return values_; }
    static Matrix identity(std::size_t n) {
        Matrix result(n, n); for (std::size_t i = 0; i < n; ++i) result(i, i) = T(1); return result;
    }
    Matrix transposed() const {
        Matrix result(cols_, rows_);
        if (values_.empty()) return result;
        for (std::size_t i = 0; i < rows_; ++i) for (std::size_t j = 0; j < cols_; ++j) result(j, i) = (*this)(i, j);
        return result;
    }
    friend bool operator==(const Matrix& a, const Matrix& b) { return a.rows_ == b.rows_ && a.cols_ == b.cols_ && a.values_ == b.values_; }
    friend bool operator!=(const Matrix& a, const Matrix& b) { return !(a == b); }
};
} // namespace cp
