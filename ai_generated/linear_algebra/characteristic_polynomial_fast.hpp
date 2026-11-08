#pragma once
#include "../linear_recurrence/berlekamp_massey.hpp"
#include "characteristic_polynomial.hpp"
#include "detail/strassen.hpp"
#include <random>

namespace cp {
struct FastCharacteristicPolynomialOptions {
    std::size_t attempts = 2;
    std::uint64_t seed = 20261027;
    std::size_t strassen_cutoff = 32;
};
struct FastCharacteristicPolynomialResult {
    std::vector<int> coefficients;
    bool used_krylov = false;
    std::size_t attempts = 0;
};
// Certified cyclic fast path O(n^log2(7) log n). If no degree-n projection is found, use Hessenberg.
// Result is always exact; the general worst case remains O(n^3), with a bounded number of trials.
template <int Mod = 998244353>
FastCharacteristicPolynomialResult
characteristic_polynomial_fast(const Matrix<int> &input, FastCharacteristicPolynomialOptions options = {}) {
    using F = typename linear_algebra_detail::PrimeField<Mod>::R;
    const auto n = input.rows();
    if (n != input.cols())
        throw std::invalid_argument("characteristic polynomial needs a square matrix");
    if (!options.strassen_cutoff)
        throw std::invalid_argument("Strassen cutoff must be positive");
    if (n > std::vector<int>().max_size() / 2)
        throw std::length_error("Krylov sequence too long");
    if (!n || n <= options.strassen_cutoff || !options.attempts)
        return {characteristic_polynomial_mod<Mod>(input), false, 0};
    Matrix<int> a(n, n);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            a(i, j) = F::norm(input(i, j));
    std::mt19937_64 random(options.seed);
    std::uniform_int_distribution<int> distribution(0, Mod - 1);
    for (std::size_t trial = 0; trial < options.attempts; ++trial) {
        std::vector<int> left(n), sequence(2 * n);
        Matrix<int> krylov(n, 2 * n);
        for (std::size_t i = 0; i < n; ++i) {
            left[i] = distribution(random);
            krylov(i, 0) = distribution(random);
        }
        auto power = a;
        for (std::size_t count = 1; count < 2 * n;) {
            const auto width = std::min(count, 2 * n - count);
            Matrix<int> block(n, width);
            for (std::size_t i = 0; i < n; ++i)
                for (std::size_t j = 0; j < width; ++j)
                    block(i, j) = krylov(i, j);
            auto next = strassen_detail::product<Mod>(power, block, options.strassen_cutoff);
            for (std::size_t i = 0; i < n; ++i)
                for (std::size_t j = 0; j < width; ++j)
                    krylov(i, count + j) = next(i, j);
            count += width;
            if (count < 2 * n)
                power = strassen_detail::product<Mod>(power, power, options.strassen_cutoff);
        }
        for (std::size_t k = 0; k < 2 * n; ++k)
            for (std::size_t i = 0; i < n; ++i)
                sequence[k] = F::add(sequence[k], F::mul(left[i], krylov(i, k)));
        auto relation = berlekamp_massey<Mod>(sequence);
        // A scalar minimal polynomial divides char(A). 2n exact terms suffice; degree n certifies equality.
        if (relation.size() == n) {
            std::vector<int> polynomial(n + 1, 1);
            for (std::size_t i = 0; i < n; ++i)
                polynomial[n - 1 - i] = F::sub(0, relation[i]);
            return {std::move(polynomial), true, trial + 1};
        }
    }
    return {characteristic_polynomial_mod<Mod>(a), false, options.attempts};
}
} // namespace cp
