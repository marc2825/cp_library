#pragma once
#include <cstdint>
#include <vector>
#include "../number_theory_basic/detail/advanced_arithmetic.hpp"
#include "../number_theory_convolution/pollard_rho.hpp"

namespace cp {
// Huge uint64 n,k, fixed arbitrary positive uint64 modulus. Tables for each prime power.
class PrimePowerBinomial {
    using U = std::uint64_t;
    struct Part { U prime, modulus, coefficient; int exponent; std::vector<U> prefix; };
    U modulus_;
    std::vector<Part> parts_;
    static U valuation(U n, U p) { U r = 0; while (n) { n /= p; r += n; } return r; }
    static U unit_factorial(U n, const Part& part) {
        U result = 1;
        while (n) {
            result = nt_detail::mul(result, nt_detail::pow(part.prefix.back(), n / part.modulus, part.modulus), part.modulus);
            result = nt_detail::mul(result, part.prefix[n % part.modulus], part.modulus);
            n /= part.prime;
        }
        return result;
    }
public:
    explicit PrimePowerBinomial(U modulus, U max_table_entries = 2000000) : modulus_(modulus) {
        if (!modulus) throw std::invalid_argument("zero binomial modulus");
        U used = 0;
        for (auto [p, e] : prime_factorization_64(modulus)) {
            U q = 1;
            for (int i = 0; i < e; ++i) q *= p;
            if (q >= max_table_entries - used || q >= std::vector<U>().max_size())
                throw std::length_error("prime-power binomial table budget");
            used += q + 1;
            Part part{p, q, 0, e, std::vector<U>(static_cast<std::size_t>(q) + 1, 1)};
            for (U i = 1; i <= q; ++i) part.prefix[i] = i % p == 0 ? part.prefix[i - 1] : nt_detail::mul(part.prefix[i - 1], i, q);
            const U cofactor = modulus / q;
            part.coefficient = nt_detail::mul(cofactor, nt_detail::inverse(cofactor % q, q), modulus);
            parts_.push_back(std::move(part));
        }
    }
    U modulus() const { return modulus_; }
    U choose(U n, U k) const {
        if (k > n || modulus_ == 1) return 0;
        U result = 0;
        for (const auto& part : parts_) {
            const U e = valuation(n, part.prime) - valuation(k, part.prime) - valuation(n - k, part.prime);
            if (e >= static_cast<U>(part.exponent)) continue;
            const U denominator = nt_detail::mul(unit_factorial(k, part), unit_factorial(n - k, part), part.modulus);
            U value = nt_detail::mul(unit_factorial(n, part), nt_detail::inverse(denominator, part.modulus), part.modulus);
            value = nt_detail::mul(value, nt_detail::pow(part.prime, e, part.modulus), part.modulus);
            value = nt_detail::mul(value, part.coefficient, modulus_);
            result = result >= modulus_ - value ? result - (modulus_ - value) : result + value;
        }
        return result;
    }
};
} // namespace cp
