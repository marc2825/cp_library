#pragma once
#include "detail/prime_field.hpp"
#include "matrix.hpp"

namespace cp {
// Alternating matrix over F_p, including characteristic 2. O(n^3); Pf(empty)=1.
template <int Mod = 998244353> int pfaffian_mod(Matrix<int> a) {
    using F = typename linear_algebra_detail::PrimeField<Mod>::R;
    const auto n = a.rows();
    if (n != a.cols())
        throw std::invalid_argument("Pfaffian needs a square matrix");
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            a(i, j) = F::norm(a(i, j));
    for (std::size_t i = 0; i < n; ++i) {
        if (a(i, i))
            throw std::invalid_argument("alternating matrix diagonal must be zero");
        for (std::size_t j = 0; j < i; ++j)
            if (F::add(a(i, j), a(j, i)))
                throw std::invalid_argument("matrix must be alternating");
    }
    if (n & 1)
        return 0;
    int answer = 1;
    for (std::size_t k = 0; k < n; k += 2) {
        auto pivot = k + 1;
        while (pivot < n && !a(k, pivot))
            ++pivot;
        if (pivot == n)
            return 0;
        if (pivot != k + 1) {
            for (std::size_t i = 0; i < n; ++i)
                std::swap(a(i, k + 1), a(i, pivot));
            for (std::size_t j = 0; j < n; ++j)
                std::swap(a(k + 1, j), a(pivot, j));
            answer = F::sub(0, answer);
        }
        answer = F::mul(answer, a(k, k + 1));
        const int inverse = F::inverse(a(k, k + 1));
        for (std::size_t i = k + 2; i < n; ++i)
            for (std::size_t j = i + 1; j < n; ++j) {
                const int cross = F::sub(F::mul(a(k, i), a(k + 1, j)), F::mul(a(k, j), a(k + 1, i)));
                a(i, j) = F::sub(a(i, j), F::mul(inverse, cross));
                a(j, i) = F::sub(0, a(i, j));
            }
    }
    return answer;
}
} // namespace cp
